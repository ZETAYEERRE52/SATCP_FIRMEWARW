#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ble_satcp.h"
#include "comunicaciones.h"
#include "comunicaciones_interno.h"
#include "maquina_estados.h"
#include "protocolo.h"
#include "seguridad.h"
#include "servidor_vivo.h"
#include "servidor_web.h"
#include "wifi_satcp.h"

static const char *TAG = "COMUNICACIONES";

#define TAM_COLA                6
#define TAM_PILA_TAREA          8192    // La foto se toma y clasifica en esta tarea
#define PRIORIDAD_TAREA         4
#define PERIODO_ESTADO_MS       5000    // Latido: la app sabe que la conexión sigue viva
#define MAX_CLIENTES_WS         4
#define ESPERA_CAMBIO_CLAVE_MS  1000    // Margen para que la respuesta salga antes de cortar la red SATCP

typedef struct {
    transporte_t transporte;
    int fd;
    char mensaje[TAM_MAX_MENSAJE];
} mensaje_entrante_t;

static QueueHandle_t s_cola;
static volatile bool s_forzar_estado;

bool comunicaciones_recibir(const char *mensaje, size_t largo, transporte_t transporte, int fd)
{
    if (!s_cola || largo >= TAM_MAX_MENSAJE) {
        return false;
    }
    mensaje_entrante_t entrada = { .transporte = transporte, .fd = fd };
    memcpy(entrada.mensaje, mensaje, largo);
    entrada.mensaje[largo] = '\0';
    if (xQueueSend(s_cola, &entrada, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Cola llena: comando descartado");
        return false;
    }
    return true;
}

void comunicaciones_forzar_estado(void)
{
    s_forzar_estado = true;
}

// La app usa un solo transporte a la vez: mientras alguien usa el Wi-Fi, el Bluetooth no anuncia
static void actualizar_anuncio_ble(void)
{
    ble_satcp_pausar_anuncio(seguridad_sesiones_activas(TRANSPORTE_WIFI) > 0);
}

void comunicaciones_cliente_desconectado(transporte_t transporte, int fd)
{
    seguridad_cerrar(transporte, fd);
    if (transporte == TRANSPORTE_WIFI) {
        actualizar_anuncio_ble();
    }
}

static void enviar_texto(transporte_t transporte, int fd, const char *texto)
{
    if (transporte == TRANSPORTE_BLE) {
        ble_satcp_enviar_json(texto);
    } else {
        servidor_web_enviar(fd, texto);
    }
}

// Solo lo leen los clientes con sesión: a los demás no se les envía nada cifrado
static void enviar_cifrado(transporte_t transporte, int fd, const char *json)
{
    char *sobre = seguridad_cifrar(transporte, fd, json);
    if (sobre) {
        enviar_texto(transporte, fd, sobre);
        free(sobre);
    }
}

// Eventos de la transferencia de la foto por Bluetooth (imagen_inicio, imagen_fin...)
static void enviar_cifrado_ble(const char *json)
{
    enviar_cifrado(TRANSPORTE_BLE, 0, json);
}

static void difundir(const char *json)
{
    if (ble_satcp_conectado()) {
        enviar_cifrado(TRANSPORTE_BLE, 0, json);
    }
    int clientes[MAX_CLIENTES_WS];
    const size_t cantidad = servidor_web_clientes(clientes, MAX_CLIENTES_WS);
    for (size_t i = 0; i < cantidad; i++) {
        enviar_cifrado(TRANSPORTE_WIFI, clientes[i], json);
    }
}

// Errores de la capa de seguridad: van en claro porque el cliente aún no tiene (o no usa bien) la sesión
static void enviar_error_seguridad(transporte_t transporte, int fd, const char *codigo, const char *mensaje)
{
    char texto[200];
    snprintf(texto, sizeof(texto), "{\"tipo\":\"respuesta\",\"ok\":false,\"error\":\"%s\",\"mensaje\":\"%s\"}",
             codigo, mensaje);
    enviar_texto(transporte, fd, texto);
}

static void atender_mensaje(mensaje_entrante_t *entrada)
{
    cJSON *raiz = cJSON_Parse(entrada->mensaje);
    const cJSON *sobre = cJSON_GetObjectItemCaseSensitive(raiz, "c");
    const cJSON *cmd = cJSON_GetObjectItemCaseSensitive(raiz, "cmd");

    // "hola" es lo único que se acepta en claro: abre la sesión cifrada de esta conexión
    if (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "hola") == 0) {
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(raiz, "id");
        char id_json[24] = "null";
        if (cJSON_IsNumber(id)) {
            snprintf(id_json, sizeof(id_json), "%d", id->valueint);
        }
        cJSON_Delete(raiz);
        char *respuesta = seguridad_hola(entrada->transporte, entrada->fd, id_json);
        if (respuesta) {
            enviar_texto(entrada->transporte, entrada->fd, respuesta);
            free(respuesta);
        }
        ESP_LOGI(TAG, "%s: sesión abierta", entrada->transporte == TRANSPORTE_BLE ? "BLE" : "Wi-Fi");
        if (entrada->transporte == TRANSPORTE_WIFI) {
            actualizar_anuncio_ble();
        }
        s_forzar_estado = true;
        return;
    }
    if (!cJSON_IsString(sobre)) {
        cJSON_Delete(raiz);
        enviar_error_seguridad(entrada->transporte, entrada->fd, "no_autorizado",
                               "Primero envia hola y luego mensajes cifrados");
        return;
    }

    // Se descifra sobre el mismo búfer: el sobre ya está copiado en el árbol de cJSON
    const resultado_seguridad_t resultado = seguridad_descifrar(entrada->transporte, entrada->fd, sobre->valuestring,
                                                                entrada->mensaje, sizeof(entrada->mensaje));
    cJSON_Delete(raiz);
    switch (resultado) {
    case SEGURIDAD_OK:
        break;
    case SEGURIDAD_SIN_SESION:
        enviar_error_seguridad(entrada->transporte, entrada->fd, "sin_sesion", "Envia hola para abrir la sesion");
        return;
    case SEGURIDAD_REPETIDO:
        ESP_LOGW(TAG, "Mensaje repetido descartado");
        enviar_error_seguridad(entrada->transporte, entrada->fd, "repetido", "Contador n ya usado");
        return;
    case SEGURIDAD_NO_AUTORIZADO:
    case SEGURIDAD_FORMATO:
        ESP_LOGW(TAG, "Mensaje que no descifra: clave equivocada o alterado");
        enviar_error_seguridad(entrada->transporte, entrada->fd, "no_autorizado", "Clave incorrecta");
        return;
    }

    imagen_pendiente_t imagen;
    bool clave_cambiada = false;
    char *respuesta = protocolo_procesar(entrada->mensaje, entrada->transporte, &imagen, &clave_cambiada);
    if (respuesta) {
        enviar_cifrado(entrada->transporte, entrada->fd, respuesta);
        free(respuesta);
    }
    if (imagen.datos) {
        size_t bytes_cifrados = 0;
        uint8_t *cifrada = seguridad_cifrar_binario(TRANSPORTE_BLE, 0, imagen.datos, imagen.bytes, &bytes_cifrados);
        free(imagen.datos);
        if (cifrada) {
            ble_satcp_enviar_imagen(imagen.id, cifrada, bytes_cifrados, enviar_cifrado_ble);
            free(cifrada);
        }
    }
    if (clave_cambiada) {
        // Con la clave nueva ninguna sesión vieja sigue valiendo, y la red SATCP pide la clave nueva
        vTaskDelay(pdMS_TO_TICKS(ESPERA_CAMBIO_CLAVE_MS));
        seguridad_cerrar_todas();
        actualizar_anuncio_ble();
        wifi_satcp_actualizar_clave_ap();
    }
}

esp_err_t comunicaciones_restaurar_clave_fabrica(void)
{
    ESP_RETURN_ON_ERROR(seguridad_restaurar_clave_fabrica(), TAG, "no se pudo restaurar la clave");
    actualizar_anuncio_ble();
    wifi_satcp_actualizar_clave_ap();
    return ESP_OK;
}

// Atiende los comandos de ambos transportes en orden y emite el evento de estado:
// al cambiar de fase, cuando se abre una sesión y como latido cada 5 s
static void tarea_comunicaciones(void *arg)
{
    mensaje_entrante_t entrada;
    estado_sistema_t fase_previa = maquina_estados_obtener_estado();
    TickType_t ultimo_estado = 0;

    for (;;) {
        if (xQueueReceive(s_cola, &entrada, pdMS_TO_TICKS(250)) == pdTRUE) {
            atender_mensaje(&entrada);
        }

        const estado_sistema_t fase = maquina_estados_obtener_estado();
        const TickType_t ahora = xTaskGetTickCount();
        if (s_forzar_estado || fase != fase_previa || ahora - ultimo_estado >= pdMS_TO_TICKS(PERIODO_ESTADO_MS)) {
            s_forzar_estado = false;
            char *evento = protocolo_evento_estado();
            if (evento) {
                difundir(evento);
                free(evento);
            }
            fase_previa = fase;
            ultimo_estado = ahora;
        }
    }
}

esp_err_t comunicaciones_iniciar(void)
{
    ESP_RETURN_ON_ERROR(seguridad_iniciar(), TAG, "seguridad");
    ESP_RETURN_ON_ERROR(protocolo_iniciar(), TAG, "protocolo");
    s_cola = xQueueCreate(TAM_COLA, sizeof(mensaje_entrante_t));
    ESP_RETURN_ON_FALSE(s_cola, ESP_ERR_NO_MEM, TAG, "sin memoria para la cola");

    // Cada transporte falla por separado: con uno solo la app ya puede operar
    esp_err_t resultado = ESP_OK;
    esp_err_t ret = wifi_satcp_iniciar();
    if (ret == ESP_OK) {
        ret = servidor_web_iniciar();
    }
    if (ret == ESP_OK && servidor_vivo_iniciar() != ESP_OK) {
        // Sin video en vivo los comandos y alertas siguen funcionando
        ESP_LOGW(TAG, "Vista en vivo no disponible");
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Transporte Wi-Fi no disponible (%s)", esp_err_to_name(ret));
        resultado = ret;
    }
    ret = ble_satcp_iniciar();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Transporte Bluetooth no disponible (%s)", esp_err_to_name(ret));
        resultado = ret;
    }

    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(tarea_comunicaciones, "comunicaciones", TAM_PILA_TAREA, NULL,
                                                PRIORIDAD_TAREA, NULL, NUCLEO_COMUNICACIONES) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "no se pudo crear la tarea");
    return resultado;
}
