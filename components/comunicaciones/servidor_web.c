#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "comunicaciones_interno.h"
#include "lwip/sockets.h"
#include "protocolo.h"
#include "seguridad.h"
#include "servidor_web.h"
#include "wifi_satcp.h"

static const char *TAG = "WIFI_APP";

#define MAX_CLIENTES        4

static httpd_handle_t s_servidor;

typedef struct {
    int fd;
    char *json;
} envio_ws_t;

// Corre dentro de la tarea del servidor HTTP: es la forma segura de escribir en un socket desde otra tarea
static void enviar_en_servidor(void *arg)
{
    envio_ws_t *envio = arg;
    httpd_ws_frame_t trama = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)envio->json,
        .len = strlen(envio->json),
        .final = true,
    };
    if (httpd_ws_send_frame_async(s_servidor, envio->fd, &trama) != ESP_OK) {
        // Un cliente que ya no recibe se cierra de inmediato: si no, cada "estado" esperaría en él
        // y frenaría a los demás clientes y a las conexiones nuevas
        ESP_LOGW(TAG, "No se pudo enviar al cliente %d: se cierra su conexión", envio->fd);
        httpd_sess_trigger_close(s_servidor, envio->fd);
    }
    free(envio->json);
    free(envio);
}

void servidor_web_enviar(int fd, const char *json)
{
    if (!s_servidor || !json) {
        return;
    }
    envio_ws_t *envio = malloc(sizeof(*envio));
    char *copia = strdup(json);
    if (!envio || !copia) {
        free(envio);
        free(copia);
        return;
    }
    envio->fd = fd;
    envio->json = copia;
    if (httpd_queue_work(s_servidor, enviar_en_servidor, envio) != ESP_OK) {
        free(copia);
        free(envio);
    }
}

size_t servidor_web_clientes(int *fds, size_t maximo)
{
    if (!s_servidor) {
        return 0;
    }
    size_t cantidad = MAX_CLIENTES;
    int clientes[MAX_CLIENTES];
    if (httpd_get_client_list(s_servidor, &cantidad, clientes) != ESP_OK) {
        return 0;
    }
    size_t websockets = 0;
    for (size_t i = 0; i < cantidad && websockets < maximo; i++) {
        if (httpd_ws_get_fd_info(s_servidor, clientes[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
            fds[websockets++] = clientes[i];
        }
    }
    return websockets;
}

// El servidor cierra un socket (el cliente se fue o se purgó): su sesión cifrada termina con él
static void al_cerrar_socket(httpd_handle_t servidor, int fd)
{
    comunicaciones_cliente_desconectado(TRANSPORTE_WIFI, fd);
    close(fd);
}

// Las rutas HTTP no van cifradas: piden el token de un solo propósito que da el comando cifrado
static bool token_valido(httpd_req_t *req, uso_token_t uso)
{
    char consulta[64];
    char token[SEGURIDAD_TAM_TOKEN];
    return httpd_req_get_url_query_str(req, consulta, sizeof(consulta)) == ESP_OK &&
           httpd_query_key_value(consulta, "t", token, sizeof(token)) == ESP_OK &&
           seguridad_validar_token(uso, token);
}

static esp_err_t manejar_ws(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        // Fin del handshake: la app debe enviar "hola" para abrir su sesión cifrada
        ESP_LOGI(TAG, "App conectada por Wi-Fi (cliente %d)", httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    httpd_ws_frame_t trama = { 0 };
    ESP_RETURN_ON_ERROR(httpd_ws_recv_frame(req, &trama, 0), TAG, "no se pudo leer la trama");
    if (trama.type != HTTPD_WS_TYPE_TEXT || trama.len == 0) {
        return ESP_OK;
    }

    char mensaje[TAM_MAX_MENSAJE];
    if (trama.len >= sizeof(mensaje)) {
        // Se descarta sin leer el resto: el servidor cierra la conexión si queda a medias
        ESP_LOGW(TAG, "Mensaje demasiado largo (%u bytes)", (unsigned)trama.len);
        return ESP_FAIL;
    }
    trama.payload = (uint8_t *)mensaje;
    ESP_RETURN_ON_ERROR(httpd_ws_recv_frame(req, &trama, trama.len), TAG, "no se pudo leer el mensaje");
    mensaje[trama.len] = '\0';

    comunicaciones_recibir(mensaje, trama.len, TRANSPORTE_WIFI, httpd_req_to_sockfd(req));
    return ESP_OK;
}

// Cada descarga queda en el log: si la app falla y aquí no aparece nada, la petición nunca llegó
static esp_err_t enviar_ultima_foto(httpd_req_t *req)
{
    if (!token_valido(req, TOKEN_FOTO)) {
        ESP_LOGW(TAG, "Descarga de foto rechazada (403): falta el token, es incorrecto o venció (dura 2 min)");
        return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Token invalido o vencido");
    }
    size_t bytes = 0;
    const uint8_t *foto = protocolo_tomar_ultima_foto(&bytes);
    if (!foto) {
        ESP_LOGW(TAG, "Descarga de foto sin foto disponible (404)");
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Aun no se ha tomado ninguna foto");
    }
    const int64_t inicio = esp_timer_get_time();
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const esp_err_t ret = httpd_resp_send(req, (const char *)foto, bytes);
    protocolo_soltar_ultima_foto();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Foto descargada: %u bytes en %lld ms", (unsigned)bytes, (esp_timer_get_time() - inicio) / 1000);
    } else {
        ESP_LOGW(TAG, "La descarga de la foto se cortó (%s): la app cerró la conexión o se perdió la señal",
                 esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t enviar_info(httpd_req_t *req)
{
    char *json = protocolo_info();
    if (!json) {
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_type(req, "application/json");
    const esp_err_t ret = httpd_resp_sendstr(req, json);
    free(json);
    return ret;
}

esp_err_t servidor_web_iniciar(void)
{
    ESP_RETURN_ON_FALSE(!s_servidor, ESP_ERR_INVALID_STATE, TAG, "el servidor ya está iniciado");

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WIFI_PUERTO;
    config.max_open_sockets = MAX_CLIENTES;
    config.lru_purge_enable = true;
    config.close_fn = al_cerrar_socket;
    config.core_id = NUCLEO_COMUNICACIONES;
    // Un teléfono que desaparece sin cerrar (sin señal, apagado, app reconectando) se detecta en ~9 s:
    // así se libera su conexión, termina su sesión y el Bluetooth vuelve a anunciar
    config.keep_alive_enable = true;
    config.keep_alive_idle = 3;
    config.keep_alive_interval = 2;
    config.keep_alive_count = 3;
    // Esta única tarea atiende a todos los clientes: un envío trabado no debe frenarla más de 2 s
    config.send_wait_timeout = 2;
    ESP_RETURN_ON_ERROR(httpd_start(&s_servidor, &config), TAG, "no se pudo iniciar el servidor");

    const httpd_uri_t rutas[] = {
        { .uri = "/ws", .method = HTTP_GET, .handler = manejar_ws, .is_websocket = true },
        { .uri = "/foto/ultima.jpg", .method = HTTP_GET, .handler = enviar_ultima_foto },
        { .uri = "/info", .method = HTTP_GET, .handler = enviar_info },
    };
    for (size_t i = 0; i < sizeof(rutas) / sizeof(rutas[0]); i++) {
        httpd_register_uri_handler(s_servidor, &rutas[i]);
    }
    ESP_LOGI(TAG, "Listo para la app: ws://<ip>/ws");
    return ESP_OK;
}
