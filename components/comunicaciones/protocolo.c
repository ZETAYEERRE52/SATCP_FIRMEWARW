#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "almacenamiento_sd.h"
#include "ble_satcp.h"
#include "foto_clasificada.h"
#include "horarios.h"
#include "ld2410.h"
#include "maquina_estados.h"
#include "monitor_temperatura.h"
#include "protocolo.h"
#include "seguridad.h"
#include "valvula.h"
#include "wifi_satcp.h"

static const char *TAG = "PROTOCOLO";

#define URL_ULTIMA_FOTO     "/foto/ultima.jpg"
#define URL_VIVO            "/vivo"

static const char *DESCRIPCION_RADAR[] = { "nadie", "en_movimiento", "quieta", "en_movimiento_y_quieta" };

static struct {
    SemaphoreHandle_t mutex_foto;
    uint8_t *ultima_foto;
    size_t bytes_ultima_foto;
} s_protocolo;

// cJSON imprime los float con todos sus decimales (24.510000228...): se redondean antes
static double redondear(double valor, int decimales)
{
    const double factor = pow(10, decimales);
    return round(valor * factor) / factor;
}

static cJSON *datos_estado(void)
{
    cJSON *datos = cJSON_CreateObject();
    cJSON_AddStringToObject(datos, "fase", maquina_estados_nombre(maquina_estados_obtener_estado()));

    cJSON *temperatura = cJSON_AddObjectToObject(datos, "temperatura");
    cJSON_AddBoolToObject(temperatura, "sensor_ok", monitor_temperatura_sensor_ok());
    muestra_temperatura_t muestra;
    if (monitor_temperatura_obtener_ultima_muestra(&muestra)) {
        cJSON_AddNumberToObject(temperatura, "medida", redondear(muestra.temp_objeto_c, 2));
        cJSON_AddNumberToObject(temperatura, "estimada", redondear(monitor_temperatura_estimar(&muestra), 1));
        cJSON_AddNumberToObject(temperatura, "sensor", redondear(muestra.temp_ambiente_c, 1));
    } else {
        cJSON_AddNullToObject(temperatura, "medida");
        cJSON_AddNullToObject(temperatura, "estimada");
        cJSON_AddNullToObject(temperatura, "sensor");
    }
    float linea_base;
    if (monitor_temperatura_obtener_linea_base(&linea_base)) {
        cJSON_AddNumberToObject(temperatura, "linea_base", redondear(linea_base, 1));
    } else {
        cJSON_AddNullToObject(temperatura, "linea_base");
    }

    cJSON *presencia = cJSON_AddObjectToObject(datos, "presencia");
    ld2410_lectura_t radar;
    if (ld2410_obtener_lectura(&radar)) {
        cJSON_AddStringToObject(presencia, "estado", DESCRIPCION_RADAR[radar.objetivo & 0x03]);
        cJSON_AddNumberToObject(presencia, "distancia_movimiento_cm", radar.distancia_movimiento_cm);
        cJSON_AddNumberToObject(presencia, "distancia_quieta_cm", radar.distancia_estatico_cm);
    } else {
        cJSON_AddStringToObject(presencia, "estado", "sin_datos");
    }

    alerta_t alerta;
    uint32_t cierre_en;
    if (maquina_estados_obtener_alerta(&alerta, &cierre_en)) {
        cJSON *datos_alerta = cJSON_AddObjectToObject(datos, "alerta");
        cJSON_AddStringToObject(datos_alerta, "vision", accion_nombre_vision(alerta.vision));
        cJSON_AddNumberToObject(datos_alerta, "temperatura", redondear(alerta.temperatura_c, 1));
        cJSON_AddStringToObject(datos_alerta, "origen", accion_nombre_origen(alerta.origen));
        cJSON_AddNumberToObject(datos_alerta, "cierre_en_s", cierre_en);
        cJSON_AddBoolToObject(datos_alerta, "admite_respuesta",
                              maquina_estados_obtener_estado() == ESTADO_SISTEMA_ALERTA_USUARIO);
        cJSON_AddBoolToObject(datos_alerta, "requiere_rearme",
                              maquina_estados_obtener_estado() == ESTADO_SISTEMA_BLOQUEADO);
    } else {
        cJSON_AddNullToObject(datos, "alerta");
    }

    cJSON *valvula = cJSON_AddObjectToObject(datos, "valvula");
    cJSON_AddStringToObject(valvula, "estado", valvula_cerrada() ? "cerrada" : "abierta");
    cJSON_AddBoolToObject(valvula, "sensor", valvula_tiene_sensor());

    cJSON *conexion = cJSON_AddObjectToObject(datos, "conexion");
    char ip[24];
    if (wifi_satcp_ip(ip, sizeof(ip))) {
        cJSON_AddStringToObject(conexion, "wifi_casa_ip", ip);
    } else {
        cJSON_AddNullToObject(conexion, "wifi_casa_ip");
    }
    cJSON_AddBoolToObject(conexion, "ble_conectado", ble_satcp_conectado());
    return datos;
}

static cJSON *datos_info(void)
{
    cJSON *datos = cJSON_CreateObject();
    cJSON_AddStringToObject(datos, "dispositivo", "SATCP");
    cJSON_AddStringToObject(datos, "firmware", SATCP_VERSION_FIRMWARE);
    cJSON_AddNumberToObject(datos, "protocolo", SATCP_VERSION_PROTOCOLO);

    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char texto_mac[18];
    snprintf(texto_mac, sizeof(texto_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON_AddStringToObject(datos, "mac", texto_mac);

    cJSON *wifi = cJSON_AddObjectToObject(datos, "wifi");
    cJSON_AddStringToObject(wifi, "ap_ssid", WIFI_AP_SSID);
    cJSON_AddStringToObject(wifi, "ap_ip", WIFI_AP_IP);
    cJSON_AddStringToObject(wifi, "mdns", WIFI_MDNS_NOMBRE ".local");
    cJSON_AddNumberToObject(wifi, "puerto_vivo", WIFI_PUERTO_VIVO);
    char ip[24];
    if (wifi_satcp_ip(ip, sizeof(ip))) {
        cJSON_AddStringToObject(wifi, "casa_ip", ip);
    } else {
        cJSON_AddNullToObject(wifi, "casa_ip");
    }
    cJSON_AddStringToObject(datos, "ble_nombre", BLE_NOMBRE_DISPOSITIVO);

    cJSON *clases = cJSON_AddArrayToObject(datos, "clases");
    for (int i = 0; i < NUM_CLASES; i++) {
        cJSON_AddItemToArray(clases, cJSON_CreateString(clasificador_nombre_clase((clase_vision_t)i)));
    }
    return datos;
}

static void guardar_ultima_foto(uint8_t *jpeg, size_t bytes)
{
    xSemaphoreTake(s_protocolo.mutex_foto, portMAX_DELAY);
    free(s_protocolo.ultima_foto);
    s_protocolo.ultima_foto = jpeg;
    s_protocolo.bytes_ultima_foto = bytes;
    xSemaphoreGive(s_protocolo.mutex_foto);
}

static uint8_t *copiar_en_psram(const uint8_t *datos, size_t bytes)
{
    uint8_t *copia = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (copia) {
        memcpy(copia, datos, bytes);
    }
    return copia;
}

// Arma la respuesta con el mismo "cmd" e "id" que trajo el comando, para que la app la empareje
static cJSON *crear_respuesta(const char *cmd, const cJSON *id, bool ok)
{
    cJSON *respuesta = cJSON_CreateObject();
    cJSON_AddStringToObject(respuesta, "tipo", "respuesta");
    cJSON_AddStringToObject(respuesta, "cmd", cmd ? cmd : "");
    if (id) {
        cJSON_AddItemToObject(respuesta, "id", cJSON_Duplicate(id, true));
    } else {
        cJSON_AddNullToObject(respuesta, "id");
    }
    cJSON_AddBoolToObject(respuesta, "ok", ok);
    return respuesta;
}

static cJSON *crear_error(const char *cmd, const cJSON *id, const char *codigo, const char *mensaje)
{
    cJSON *respuesta = crear_respuesta(cmd, id, false);
    cJSON_AddStringToObject(respuesta, "error", codigo);
    cJSON_AddStringToObject(respuesta, "mensaje", mensaje);
    return respuesta;
}

static cJSON *comando_foto(const cJSON *raiz, const cJSON *id, transporte_t transporte, imagen_pendiente_t *imagen)
{
    foto_clasificada_t foto;
    uint8_t *jpeg = NULL;
    const esp_err_t ret = foto_clasificada_tomar(SD_CARPETA_TELEFONO, &foto, &jpeg);
    if (ret == ESP_ERR_TIMEOUT) {
        return crear_error("foto", id, "camara_ocupada", "La cámara está ocupada, reintenta en unos segundos");
    }
    if (ret != ESP_OK) {
        return crear_error("foto", id, "camara_no_disponible", "La cámara no está disponible");
    }

    cJSON *respuesta = crear_respuesta("foto", id, true);
    cJSON *datos = cJSON_AddObjectToObject(respuesta, "datos");
    cJSON_AddBoolToObject(datos, "clasificada", foto.clasificada);
    if (foto.clasificada) {
        const resultado_clasificacion_t *c = &foto.clasificacion;
        cJSON_AddStringToObject(datos, "clase", clasificador_nombre_clase(c->clase));
        cJSON *probabilidades = cJSON_AddObjectToObject(datos, "probabilidades");
        for (int i = 0; i < NUM_CLASES; i++) {
            cJSON_AddNumberToObject(probabilidades, clasificador_nombre_clase((clase_vision_t)i),
                                    redondear(c->probabilidad[i], 3));
        }
        cJSON_AddNumberToObject(datos, "tiempo_ms", c->tiempo_ms);
    } else {
        cJSON_AddNullToObject(datos, "clase");
    }
    cJSON_AddBoolToObject(datos, "guardada", foto.guardada);
    if (foto.guardada) {
        cJSON_AddStringToObject(datos, "ruta_sd", foto.ruta);
    } else {
        cJSON_AddNullToObject(datos, "ruta_sd");
    }
    cJSON_AddNumberToObject(datos, "bytes", foto.bytes);

    // La imagen no viaja dentro del JSON: por Wi-Fi se descarga por HTTP y por Bluetooth va por su característica
    const cJSON *pide_imagen = cJSON_GetObjectItemCaseSensitive(raiz, "imagen");
    cJSON *datos_imagen = cJSON_AddObjectToObject(datos, "imagen");
    if (!jpeg) {
        cJSON_AddStringToObject(datos_imagen, "entrega", "no_disponible");
    } else if (transporte == TRANSPORTE_WIFI) {
        // La descarga HTTP no va cifrada: el token (válido 2 min) evita que otro equipo de la red la pida
        char token[SEGURIDAD_TAM_TOKEN];
        char url[sizeof(URL_ULTIMA_FOTO) + 3 + SEGURIDAD_TAM_TOKEN];
        seguridad_nuevo_token(TOKEN_FOTO, token);
        snprintf(url, sizeof(url), URL_ULTIMA_FOTO "?t=%s", token);
        cJSON_AddStringToObject(datos_imagen, "entrega", "http");
        cJSON_AddStringToObject(datos_imagen, "url", url);
    } else if (cJSON_IsTrue(pide_imagen)) {
        imagen->datos = copiar_en_psram(jpeg, foto.bytes);
        imagen->bytes = foto.bytes;
        imagen->id = cJSON_IsNumber(id) ? (int32_t)id->valuedouble : 0;
        cJSON_AddStringToObject(datos_imagen, "entrega", imagen->datos ? "ble" : "no_disponible");
    } else {
        cJSON_AddStringToObject(datos_imagen, "entrega", "no_solicitada");
    }

    if (jpeg) {
        guardar_ultima_foto(jpeg, foto.bytes);
    }
    return respuesta;
}

static cJSON *comando_wifi(const cJSON *raiz, const cJSON *id)
{
    const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(raiz, "ssid");
    const cJSON *clave = cJSON_GetObjectItemCaseSensitive(raiz, "clave");
    if (!cJSON_IsString(ssid) || ssid->valuestring[0] == '\0' || (clave && !cJSON_IsString(clave))) {
        return crear_error("wifi", id, "parametros_invalidos", "Se requiere \"ssid\" (y \"clave\" si la red tiene)");
    }
    // La clave no se muestra: solo su largo, para comprobar que llegó completa
    ESP_LOGI(TAG, "Red recibida desde la app: \"%s\" (clave de %u caracteres)", ssid->valuestring,
             (unsigned)(clave ? strlen(clave->valuestring) : 0));
    const esp_err_t ret = wifi_satcp_configurar_red(ssid->valuestring, clave ? clave->valuestring : "");
    if (ret != ESP_OK) {
        return crear_error("wifi", id, "wifi_error", "No se pudo guardar la red (nombre o clave demasiado largos)");
    }
    cJSON *respuesta = crear_respuesta("wifi", id, true);
    cJSON_AddStringToObject(respuesta, "mensaje", "Conectando; la IP llegará en el evento de estado");
    return respuesta;
}

static cJSON *comando_horarios(const cJSON *id)
{
    cJSON *respuesta = crear_respuesta("horarios", id, true);
    cJSON *datos = cJSON_AddObjectToObject(respuesta, "datos");
    cJSON_AddBoolToObject(datos, "hora_valida", horarios_hora_valida());
    cJSON_AddNumberToObject(datos, "maximo", HORARIOS_MAXIMO);
    cJSON *lista = cJSON_AddArrayToObject(datos, "horarios");
    for (int i = 0; i < HORARIOS_MAXIMO; i++) {
        horario_t h;
        if (!horarios_obtener(i, &h)) {
            continue;
        }
        cJSON *item = cJSON_CreateObject();
        cJSON_AddNumberToObject(item, "indice", i);
        cJSON_AddNumberToObject(item, "hora", h.hora);
        cJSON_AddNumberToObject(item, "minuto", h.minuto);
        cJSON_AddNumberToObject(item, "dias", h.dias);
        cJSON_AddBoolToObject(item, "activo", h.activo);
        cJSON_AddItemToArray(lista, item);
    }
    return respuesta;
}

// Entero en [minimo, maximo]; los decimales o los textos no se aceptan
static bool leer_entero(const cJSON *raiz, const char *campo, int minimo, int maximo, int *valor)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(raiz, campo);
    if (!cJSON_IsNumber(item) || item->valuedouble != (double)(int)item->valuedouble ||
        item->valueint < minimo || item->valueint > maximo) {
        return false;
    }
    *valor = item->valueint;
    return true;
}

static cJSON *comando_horario_guardar(const cJSON *raiz, const cJSON *id)
{
    // Sin "indice" (o null) se usa el primer lugar libre
    const cJSON *item_indice = cJSON_GetObjectItemCaseSensitive(raiz, "indice");
    const cJSON *item_activo = cJSON_GetObjectItemCaseSensitive(raiz, "activo");
    int indice = -1, hora, minuto, dias;
    const bool indice_ok = !item_indice || cJSON_IsNull(item_indice) ||
                           leer_entero(raiz, "indice", 0, HORARIOS_MAXIMO - 1, &indice);
    if (!indice_ok || !leer_entero(raiz, "hora", 0, 23, &hora) || !leer_entero(raiz, "minuto", 0, 59, &minuto) ||
        !leer_entero(raiz, "dias", 1, HORARIOS_TODOS_LOS_DIAS, &dias) || (item_activo && !cJSON_IsBool(item_activo))) {
        return crear_error("horario_guardar", id, "parametros_invalidos",
                           "Se requieren hora (0-23), minuto (0-59) y dias (1-127); indice 0-7 y activo opcionales");
    }

    const horario_t horario = {
        .hora = (uint8_t)hora, .minuto = (uint8_t)minuto, .dias = (uint8_t)dias,
        .activo = !item_activo || cJSON_IsTrue(item_activo),
    };
    int indice_guardado;
    const esp_err_t ret = horarios_guardar(indice, &horario, &indice_guardado);
    if (ret == ESP_ERR_NO_MEM) {
        return crear_error("horario_guardar", id, "sin_espacio", "Ya hay 8 horarios; borra uno primero");
    }
    if (ret != ESP_OK) {
        return crear_error("horario_guardar", id, "parametros_invalidos", "No se pudo guardar el horario");
    }
    cJSON *respuesta = crear_respuesta("horario_guardar", id, true);
    cJSON *datos = cJSON_AddObjectToObject(respuesta, "datos");
    cJSON_AddNumberToObject(datos, "indice", indice_guardado);
    cJSON_AddBoolToObject(datos, "hora_valida", horarios_hora_valida());
    return respuesta;
}

static cJSON *comando_horario_borrar(const cJSON *raiz, const cJSON *id)
{
    int indice;
    if (!leer_entero(raiz, "indice", 0, HORARIOS_MAXIMO - 1, &indice) || horarios_borrar(indice) != ESP_OK) {
        return crear_error("horario_borrar", id, "parametros_invalidos", "Se requiere \"indice\" entre 0 y 7");
    }
    return crear_respuesta("horario_borrar", id, true);
}

static cJSON *comando_vivo(const cJSON *id, transporte_t transporte)
{
    if (transporte != TRANSPORTE_WIFI) {
        return crear_error("vivo", id, "solo_wifi", "La vista en vivo solo funciona por Wi-Fi");
    }
    char token[SEGURIDAD_TAM_TOKEN];
    char url[sizeof(URL_VIVO) + 3 + SEGURIDAD_TAM_TOKEN];
    seguridad_nuevo_token(TOKEN_VIVO, token);
    snprintf(url, sizeof(url), URL_VIVO "?t=%s", token);
    cJSON *respuesta = crear_respuesta("vivo", id, true);
    cJSON *datos = cJSON_AddObjectToObject(respuesta, "datos");
    cJSON_AddNumberToObject(datos, "puerto", WIFI_PUERTO_VIVO);
    cJSON_AddStringToObject(datos, "url", url);
    cJSON_AddNumberToObject(datos, "vigencia_s", 30);
    return respuesta;
}

static cJSON *comando_cambiar_clave(const cJSON *raiz, const cJSON *id, bool *clave_cambiada)
{
    const cJSON *nueva = cJSON_GetObjectItemCaseSensitive(raiz, "clave_nueva");
    if (!cJSON_IsString(nueva) || seguridad_cambiar_clave(nueva->valuestring) != ESP_OK) {
        return crear_error("cambiar_clave", id, "parametros_invalidos",
                           "\"clave_nueva\" debe tener de 8 a 63 caracteres ASCII imprimibles");
    }
    *clave_cambiada = true;
    cJSON *respuesta = crear_respuesta("cambiar_clave", id, true);
    cJSON_AddStringToObject(respuesta, "mensaje",
                            "Clave cambiada: reconecta con la clave nueva (tambien es la de la red SATCP)");
    return respuesta;
}

char *protocolo_procesar(const char *mensaje, transporte_t transporte, imagen_pendiente_t *imagen,
                         bool *clave_cambiada)
{
    const int64_t inicio = esp_timer_get_time();
    memset(imagen, 0, sizeof(*imagen));
    *clave_cambiada = false;
    cJSON *raiz = cJSON_Parse(mensaje);
    if (!cJSON_IsObject(raiz)) {
        cJSON_Delete(raiz);
        cJSON *error = crear_error(NULL, NULL, "json_invalido", "El mensaje no es un objeto JSON");
        char *texto = cJSON_PrintUnformatted(error);
        cJSON_Delete(error);
        return texto;
    }

    const cJSON *item_cmd = cJSON_GetObjectItemCaseSensitive(raiz, "cmd");
    const cJSON *id = cJSON_GetObjectItemCaseSensitive(raiz, "id");
    const char *cmd = cJSON_IsString(item_cmd) ? item_cmd->valuestring : NULL;
    cJSON *respuesta;

    if (!cmd) {
        respuesta = crear_error(NULL, id, "parametros_invalidos", "Falta el campo \"cmd\"");
    } else if (strcmp(cmd, "info") == 0) {
        respuesta = crear_respuesta(cmd, id, true);
        cJSON_AddItemToObject(respuesta, "datos", datos_info());
    } else if (strcmp(cmd, "estado") == 0) {
        respuesta = crear_respuesta(cmd, id, true);
        cJSON_AddItemToObject(respuesta, "datos", datos_estado());
    } else if (strcmp(cmd, "foto") == 0) {
        respuesta = comando_foto(raiz, id, transporte, imagen);
    } else if (strcmp(cmd, "responder") == 0) {
        respuesta = maquina_estados_notificar_respuesta_usuario() == ESP_OK
                    ? crear_respuesta(cmd, id, true)
                    : crear_error(cmd, id, "sin_alerta", "No hay una alerta esperando respuesta");
    } else if (strcmp(cmd, "rearmar") == 0) {
        const esp_err_t ret = maquina_estados_rearmar();
        if (ret == ESP_OK) {
            respuesta = crear_respuesta(cmd, id, true);
        } else if (ret == ESP_ERR_NOT_FINISHED) {
            respuesta = crear_error(cmd, id, "valvula_cerrada", "Abre la válvula a mano antes de rearmar");
        } else {
            respuesta = crear_error(cmd, id, "no_bloqueado", "El sistema no está bloqueado");
        }
    } else if (strcmp(cmd, "cerrar") == 0) {
        respuesta = maquina_estados_cerrar_manual() == ESP_OK
                    ? crear_respuesta(cmd, id, true)
                    : crear_error(cmd, id, "ya_bloqueado", "El suministro ya está cerrado; rearma primero");
    } else if (strcmp(cmd, "horarios") == 0) {
        respuesta = comando_horarios(id);
    } else if (strcmp(cmd, "horario_guardar") == 0) {
        respuesta = comando_horario_guardar(raiz, id);
    } else if (strcmp(cmd, "horario_borrar") == 0) {
        respuesta = comando_horario_borrar(raiz, id);
    } else if (strcmp(cmd, "vivo") == 0) {
        respuesta = comando_vivo(id, transporte);
    } else if (strcmp(cmd, "cambiar_clave") == 0) {
        respuesta = comando_cambiar_clave(raiz, id, clave_cambiada);
    } else if (strcmp(cmd, "wifi") == 0) {
        respuesta = comando_wifi(raiz, id);
    } else {
        respuesta = crear_error(cmd, id, "comando_desconocido", "Comando no reconocido");
    }

    // El tiempo es solo el del ESP32: si la app tarda mucho más, la demora está en la red o en la app
    ESP_LOGI(TAG, "%s: %s (%s) en %lld ms", transporte == TRANSPORTE_BLE ? "BLE" : "Wi-Fi", cmd ? cmd : "?",
             cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(respuesta, "ok")) ? "ok" : "error",
             (esp_timer_get_time() - inicio) / 1000);
    char *texto = cJSON_PrintUnformatted(respuesta);
    cJSON_Delete(respuesta);
    cJSON_Delete(raiz);
    return texto;
}

char *protocolo_evento_estado(void)
{
    cJSON *evento = cJSON_CreateObject();
    cJSON_AddStringToObject(evento, "tipo", "evento");
    cJSON_AddStringToObject(evento, "evento", "estado");
    cJSON_AddItemToObject(evento, "datos", datos_estado());
    char *texto = cJSON_PrintUnformatted(evento);
    cJSON_Delete(evento);
    return texto;
}

char *protocolo_info(void)
{
    // GET /info no va cifrado: solo lo necesario para saber que es un SATCP y su versión
    cJSON *info = cJSON_CreateObject();
    cJSON_AddStringToObject(info, "dispositivo", "SATCP");
    cJSON_AddStringToObject(info, "firmware", SATCP_VERSION_FIRMWARE);
    cJSON_AddNumberToObject(info, "protocolo", SATCP_VERSION_PROTOCOLO);
    char *texto = cJSON_PrintUnformatted(info);
    cJSON_Delete(info);
    return texto;
}

esp_err_t protocolo_iniciar(void)
{
    s_protocolo.mutex_foto = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_protocolo.mutex_foto, ESP_ERR_NO_MEM, TAG, "sin memoria");
    return ESP_OK;
}

const uint8_t *protocolo_tomar_ultima_foto(size_t *bytes)
{
    xSemaphoreTake(s_protocolo.mutex_foto, portMAX_DELAY);
    if (!s_protocolo.ultima_foto) {
        xSemaphoreGive(s_protocolo.mutex_foto);
        return NULL;
    }
    *bytes = s_protocolo.bytes_ultima_foto;
    return s_protocolo.ultima_foto;
}

void protocolo_soltar_ultima_foto(void)
{
    xSemaphoreGive(s_protocolo.mutex_foto);
}
