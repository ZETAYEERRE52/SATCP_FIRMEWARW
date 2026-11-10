#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camara_vision.h"
#include "comunicaciones_interno.h"
#include "seguridad.h"
#include "servidor_vivo.h"
#include "wifi_satcp.h"

static const char *TAG = "VIVO";

#define PERIODO_CUADRO_MS       100             // Hasta ~10 cuadros por segundo con los cuadros QVGA (~10 KB)
#define DURACION_MAXIMA_US      (10 * 60 * 1000000LL)  // Corta sola si la app olvida cerrarla
#define ESPERA_CAMARA_MS        1000
#define TAM_MAX_CUADRO          (160 * 1024)    // Un JPEG VGA de calidad 12 ronda los 40 KB
#define SEPARADOR               "satcpcuadro"
#define CTRL_PORT_VIVO          (ESP_HTTPD_DEF_CTRL_PORT + 1)   // Cada servidor necesita su propio puerto de control
#define PERIODO_DIAGNOSTICO_US  (5 * 1000000LL)  // Resumen de tiempos del video en el log
#define TIEMPO_ESPERA_ENVIO_S   15      // Con señal débil un cuadro puede tardar; 5 s (por defecto) cortaba el video

static httpd_handle_t s_servidor;
static volatile bool s_transmitiendo;

static esp_err_t transmitir_vivo(httpd_req_t *req)
{
    // El video no va cifrado: solo lo abre quien pidió el token con el comando cifrado "vivo"
    char consulta[64];
    char token[SEGURIDAD_TAM_TOKEN];
    if (httpd_req_get_url_query_str(req, consulta, sizeof(consulta)) != ESP_OK ||
        httpd_query_key_value(consulta, "t", token, sizeof(token)) != ESP_OK ||
        !seguridad_validar_token(TOKEN_VIVO, token)) {
        return httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "Token invalido o vencido");
    }
    if (s_transmitiendo) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "Ya hay alguien viendo la camara");
    }

    uint8_t *cuadro_copia = heap_caps_malloc(TAM_MAX_CUADRO, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!cuadro_copia) {
        return httpd_resp_send_500(req);
    }
    s_transmitiendo = true;
    if (camara_modo_vivo(true, pdMS_TO_TICKS(ESPERA_CAMARA_MS * 3)) != ESP_OK) {
        // Si la cámara está ocupada se transmite igual, en la resolución normal
        ESP_LOGW(TAG, "No se pudo bajar la resolución para el video");
    }
    ESP_LOGI(TAG, "Vista en vivo iniciada");

    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" SEPARADOR);
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    const int64_t inicio = esp_timer_get_time();
    TickType_t ultimo_cuadro = xTaskGetTickCount();
    uint32_t enviados = 0;
    uint64_t bytes_enviados = 0;
    esp_err_t ret = ESP_OK;
    // Diagnóstico del retraso: si domina la captura, el límite es la cámara; si domina el envío, la red o la app
    int64_t captura_us = 0;
    int64_t envio_us = 0;
    int64_t envio_max_us = 0;
    uint32_t cuadros_tramo = 0;
    int64_t inicio_tramo = inicio;

    while (ret == ESP_OK && esp_timer_get_time() - inicio < DURACION_MAXIMA_US) {
        // La cámara se suelta en cuanto se copia el cuadro: la fase visual tiene más prioridad
        // que esta tarea y toma sus fotos entre un cuadro y otro aunque haya alguien mirando
        const int64_t antes_captura = esp_timer_get_time();
        camera_fb_t *cuadro = camara_capturar(pdMS_TO_TICKS(ESPERA_CAMARA_MS));
        if (!cuadro) {
            vTaskDelay(pdMS_TO_TICKS(PERIODO_CUADRO_MS));
            continue;
        }
        const size_t largo = cuadro->len <= TAM_MAX_CUADRO ? cuadro->len : 0;
        if (largo > 0) {
            memcpy(cuadro_copia, cuadro->buf, largo);
        }
        camara_liberar(cuadro);
        if (largo == 0) {
            continue;
        }
        const int64_t antes_envio = esp_timer_get_time();
        captura_us += antes_envio - antes_captura;

        char cabecera[96];
        const int largo_cabecera = snprintf(cabecera, sizeof(cabecera),
                                            "\r\n--" SEPARADOR "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                                            (unsigned)largo);
        ret = httpd_resp_send_chunk(req, cabecera, largo_cabecera);
        if (ret == ESP_OK) {
            ret = httpd_resp_send_chunk(req, (const char *)cuadro_copia, largo);
        }
        if (ret == ESP_OK) {
            enviados++;
            bytes_enviados += largo;
        }
        const int64_t duracion_envio = esp_timer_get_time() - antes_envio;
        envio_us += duracion_envio;
        envio_max_us = duracion_envio > envio_max_us ? duracion_envio : envio_max_us;
        cuadros_tramo++;

        const int64_t ahora = esp_timer_get_time();
        if (ahora - inicio_tramo >= PERIODO_DIAGNOSTICO_US) {
            ESP_LOGI(TAG, "Últimos %lld s: %.1f cuadros/s | captura %lld ms | envío %lld ms (máx. %lld ms)",
                     (ahora - inicio_tramo) / 1000000, cuadros_tramo * 1e6f / (float)(ahora - inicio_tramo),
                     captura_us / cuadros_tramo / 1000, envio_us / cuadros_tramo / 1000, envio_max_us / 1000);
            captura_us = 0;
            envio_us = 0;
            envio_max_us = 0;
            cuadros_tramo = 0;
            inicio_tramo = ahora;
        }
        // Tras un envío lento, DelayUntil "recuperaría" los cuadros atrasados enviándolos seguidos
        // (ráfagas de ~17 cuadros/s): si ya pasó el periodo, se reinicia el ritmo desde ahora
        if (xTaskDelayUntil(&ultimo_cuadro, pdMS_TO_TICKS(PERIODO_CUADRO_MS)) == pdFALSE) {
            ultimo_cuadro = xTaskGetTickCount();
        }
    }

    // Si el cliente cerró la conexión, el envío ya falló; si se cumplió el tiempo, se cierra la respuesta
    if (ret == ESP_OK) {
        httpd_resp_send_chunk(req, NULL, 0);
    }
    heap_caps_free(cuadro_copia);
    // Las fotos de la vigilancia y del dataset vuelven a la resolución completa
    if (camara_modo_vivo(false, portMAX_DELAY) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo restaurar la resolución de la cámara");
    }
    s_transmitiendo = false;
    // El envío falla si la app cerró el video (lo normal al salir de la pantalla), si dejó de leer
    // durante TIEMPO_ESPERA_ENVIO_S o si se perdió la conexión; el httpd_sock_err previo dice cuál (104 = la app cerró)
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Envío del video cortado: la app cerró la conexión, dejó de leer o se perdió la señal");
    }
    ESP_LOGI(TAG, "Vista en vivo terminada: %lu cuadros completos en %lld s (%lu KB promedio)",
             (unsigned long)enviados, (esp_timer_get_time() - inicio) / 1000000,
             (unsigned long)(enviados ? bytes_enviados / enviados / 1024 : 0));
    return ESP_OK;
}

esp_err_t servidor_vivo_iniciar(void)
{
    ESP_RETURN_ON_FALSE(!s_servidor, ESP_ERR_INVALID_STATE, TAG, "el servidor ya está iniciado");

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WIFI_PUERTO_VIVO;
    config.ctrl_port = CTRL_PORT_VIVO;
    config.max_open_sockets = 2;
    config.lru_purge_enable = true;
    config.send_wait_timeout = TIEMPO_ESPERA_ENVIO_S;
    config.core_id = NUCLEO_COMUNICACIONES;
    ESP_RETURN_ON_ERROR(httpd_start(&s_servidor, &config), TAG, "no se pudo iniciar el servidor de video");

    const httpd_uri_t ruta = { .uri = "/vivo", .method = HTTP_GET, .handler = transmitir_vivo };
    httpd_register_uri_handler(s_servidor, &ruta);
    ESP_LOGI(TAG, "Vista en vivo disponible en http://<ip>:%d/vivo", WIFI_PUERTO_VIVO);
    return ESP_OK;
}
