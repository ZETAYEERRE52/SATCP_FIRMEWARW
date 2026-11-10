#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bootloader_random.h"
#include "cJSON.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mbedtls/base64.h"
#include "nvs.h"
#include "psa/crypto.h"
#include "seguridad.h"

static const char *TAG = "SEGURIDAD";

#define NVS_ESPACIO             "satcp"
#define NVS_CLAVE_ACTUAL        "clave"
#define NVS_CLAVE_FABRICA       "clave_fab"
#define LARGO_CLAVE_FABRICA     12
// Sin 0/o, 1/l/i: la clave de fábrica se copia a mano desde una etiqueta
#define ALFABETO_CLAVE          "23456789abcdefghjkmnpqrstuvwxyz"
#define TAM_RETO                16
#define TAM_NONCE               12
#define TAM_ETIQUETA            16
#define TAM_CLAVE               32
#define MAX_SESIONES            6       // 4 WebSocket + 1 Bluetooth + margen
#define FALLOS_PARA_CERRAR      3       // Mensajes que no descifran antes de exigir un "hola" nuevo
#define CONTEXTO_SESION         "SATCP-sesion"
#define AAD_APP                 "app"   // Sentido app -> ESP32
#define AAD_ESP                 "esp"   // Sentido ESP32 -> app
#define AAD_IMAGEN              "img"
#define VIGENCIA_TOKEN_FOTO_US  (120 * 1000000LL)
#define VIGENCIA_TOKEN_VIVO_US  (30 * 1000000LL)

typedef struct {
    bool activa;
    transporte_t transporte;
    int fd;
    psa_key_id_t clave;
    uint64_t n_recibido;    // Último contador aceptado de la app
    uint64_t n_enviado;
    uint8_t fallos;
} sesion_t;

static struct {
    SemaphoreHandle_t mutex;
    char clave[SEGURIDAD_CLAVE_MAX + 1];
    psa_key_id_t clave_maestra;
    sesion_t sesiones[MAX_SESIONES];
    char token[NUM_TOKENS][SEGURIDAD_TAM_TOKEN];
    int64_t vence_token_us[NUM_TOKENS];
} s_seg;

static bool clave_valida(const char *clave)
{
    const size_t largo = clave ? strlen(clave) : 0;
    if (largo < SEGURIDAD_CLAVE_MIN || largo > SEGURIDAD_CLAVE_MAX) {
        return false;
    }
    for (size_t i = 0; i < largo; i++) {
        if (clave[i] < 0x20 || clave[i] > 0x7E) {
            return false;   // WPA2 solo admite ASCII imprimible
        }
    }
    return true;
}

static void a_hexadecimal(const uint8_t *datos, size_t bytes, char *texto)
{
    static const char DIGITOS[] = "0123456789abcdef";
    for (size_t i = 0; i < bytes; i++) {
        texto[2 * i] = DIGITOS[datos[i] >> 4];
        texto[2 * i + 1] = DIGITOS[datos[i] & 0x0F];
    }
    texto[2 * bytes] = '\0';
}

static void generar_clave_fabrica(char *clave)
{
    // Al arrancar aún no hay radio encendida: se activa la fuente de entropía del ADC
    uint8_t aleatorio[LARGO_CLAVE_FABRICA];
    bootloader_random_enable();
    esp_fill_random(aleatorio, sizeof(aleatorio));
    bootloader_random_disable();
    const size_t tam_alfabeto = sizeof(ALFABETO_CLAVE) - 1;
    for (int i = 0; i < LARGO_CLAVE_FABRICA; i++) {
        clave[i] = ALFABETO_CLAVE[aleatorio[i] % tam_alfabeto];
    }
    clave[LARGO_CLAVE_FABRICA] = '\0';
}

static esp_err_t guardar_nvs(const char *nombre, const char *valor)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_ESPACIO, NVS_READWRITE, &nvs), TAG, "NVS");
    esp_err_t ret = nvs_set_str(nvs, nombre, valor);
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return ret;
}

static bool leer_nvs(const char *nombre, char *valor, size_t tam)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_ESPACIO, NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }
    const bool ok = nvs_get_str(nvs, nombre, valor, &tam) == ESP_OK;
    nvs_close(nvs);
    return ok;
}

static psa_status_t importar_clave(const uint8_t *bytes, psa_key_type_t tipo, psa_key_usage_t uso,
                                   psa_algorithm_t algoritmo, psa_key_id_t *id)
{
    psa_key_attributes_t atributos = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&atributos, tipo);
    psa_set_key_bits(&atributos, TAM_CLAVE * 8);
    psa_set_key_usage_flags(&atributos, uso);
    psa_set_key_algorithm(&atributos, algoritmo);
    return psa_import_key(&atributos, bytes, TAM_CLAVE, id);
}

// Clave maestra = PBKDF2-HMAC-SHA256(clave, sal, iteraciones). Tarda menos de un segundo
static esp_err_t derivar_clave_maestra(const char *clave)
{
    uint8_t bytes[TAM_CLAVE];
    psa_key_derivation_operation_t operacion = PSA_KEY_DERIVATION_OPERATION_INIT;
    psa_status_t estado = psa_key_derivation_setup(&operacion, PSA_ALG_PBKDF2_HMAC(PSA_ALG_SHA_256));
    if (estado == PSA_SUCCESS) {
        estado = psa_key_derivation_input_integer(&operacion, PSA_KEY_DERIVATION_INPUT_COST, SEGURIDAD_ITERACIONES);
    }
    if (estado == PSA_SUCCESS) {
        estado = psa_key_derivation_input_bytes(&operacion, PSA_KEY_DERIVATION_INPUT_SALT,
                                                (const uint8_t *)SEGURIDAD_SAL, strlen(SEGURIDAD_SAL));
    }
    if (estado == PSA_SUCCESS) {
        estado = psa_key_derivation_input_bytes(&operacion, PSA_KEY_DERIVATION_INPUT_PASSWORD,
                                                (const uint8_t *)clave, strlen(clave));
    }
    if (estado == PSA_SUCCESS) {
        estado = psa_key_derivation_output_bytes(&operacion, bytes, sizeof(bytes));
    }
    psa_key_derivation_abort(&operacion);
    ESP_RETURN_ON_FALSE(estado == PSA_SUCCESS, ESP_FAIL, TAG, "PBKDF2 falló (%d)", (int)estado);

    psa_key_id_t nueva = PSA_KEY_ID_NULL;
    estado = importar_clave(bytes, PSA_KEY_TYPE_HMAC, PSA_KEY_USAGE_SIGN_MESSAGE, PSA_ALG_HMAC(PSA_ALG_SHA_256), &nueva);
    memset(bytes, 0, sizeof(bytes));
    ESP_RETURN_ON_FALSE(estado == PSA_SUCCESS, ESP_FAIL, TAG, "no se pudo importar la clave maestra (%d)", (int)estado);

    psa_destroy_key(s_seg.clave_maestra);
    s_seg.clave_maestra = nueva;
    return ESP_OK;
}

static sesion_t *buscar_sesion(transporte_t transporte, int fd)
{
    for (int i = 0; i < MAX_SESIONES; i++) {
        sesion_t *s = &s_seg.sesiones[i];
        if (s->activa && s->transporte == transporte && (transporte == TRANSPORTE_BLE || s->fd == fd)) {
            return s;
        }
    }
    return NULL;
}

static void cerrar_sesion(sesion_t *sesion)
{
    psa_destroy_key(sesion->clave);
    memset(sesion, 0, sizeof(*sesion));
}

esp_err_t seguridad_iniciar(void)
{
    ESP_RETURN_ON_FALSE(!s_seg.mutex, ESP_ERR_INVALID_STATE, TAG, "ya iniciada");
    ESP_RETURN_ON_FALSE(psa_crypto_init() == PSA_SUCCESS, ESP_FAIL, TAG, "no se pudo iniciar PSA Crypto");
    s_seg.mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_seg.mutex, ESP_ERR_NO_MEM, TAG, "sin memoria");

    char fabrica[SEGURIDAD_CLAVE_MAX + 1];
    if (!leer_nvs(NVS_CLAVE_FABRICA, fabrica, sizeof(fabrica))) {
        generar_clave_fabrica(fabrica);
        ESP_RETURN_ON_ERROR(guardar_nvs(NVS_CLAVE_FABRICA, fabrica), TAG, "no se pudo guardar la clave de fábrica");
        ESP_LOGW(TAG, "Primera vez: se generó la clave de fábrica de este equipo");
    }
    if (!leer_nvs(NVS_CLAVE_ACTUAL, s_seg.clave, sizeof(s_seg.clave)) || !clave_valida(s_seg.clave)) {
        strlcpy(s_seg.clave, fabrica, sizeof(s_seg.clave));
    }
    ESP_RETURN_ON_ERROR(derivar_clave_maestra(s_seg.clave), TAG, "clave maestra");

    // Se muestra en el monitor serie (acceso físico) para escribirla en la etiqueta del equipo
    ESP_LOGW(TAG, "Clave de fábrica: %s%s", fabrica,
             strcmp(fabrica, s_seg.clave) == 0 ? " (en uso)" : " (el usuario la cambió desde la app)");
    return ESP_OK;
}

const char *seguridad_clave_equipo(void)
{
    return s_seg.clave;
}

char *seguridad_hola(transporte_t transporte, int fd, const char *id_json)
{
    uint8_t reto[TAM_RETO];
    esp_fill_random(reto, sizeof(reto));
    uint8_t mensaje[sizeof(CONTEXTO_SESION) - 1 + TAM_RETO];
    memcpy(mensaje, CONTEXTO_SESION, sizeof(CONTEXTO_SESION) - 1);
    memcpy(mensaje + sizeof(CONTEXTO_SESION) - 1, reto, TAM_RETO);

    xSemaphoreTake(s_seg.mutex, portMAX_DELAY);
    sesion_t *sesion = buscar_sesion(transporte, fd);
    if (sesion) {
        cerrar_sesion(sesion);
    }
    for (int i = 0; i < MAX_SESIONES && !sesion; i++) {
        if (!s_seg.sesiones[i].activa) {
            sesion = &s_seg.sesiones[i];
        }
    }
    uint8_t clave_sesion[PSA_HASH_LENGTH(PSA_ALG_SHA_256)];
    size_t largo = 0;
    bool ok = sesion &&
              psa_mac_compute(s_seg.clave_maestra, PSA_ALG_HMAC(PSA_ALG_SHA_256), mensaje, sizeof(mensaje),
                              clave_sesion, sizeof(clave_sesion), &largo) == PSA_SUCCESS;
    if (ok) {
        *sesion = (sesion_t){ .activa = true, .transporte = transporte, .fd = fd };
        ok = importar_clave(clave_sesion, PSA_KEY_TYPE_AES, PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT,
                            PSA_ALG_GCM, &sesion->clave) == PSA_SUCCESS;
        if (!ok) {
            memset(sesion, 0, sizeof(*sesion));
        }
    }
    memset(clave_sesion, 0, sizeof(clave_sesion));
    xSemaphoreGive(s_seg.mutex);

    char reto_hex[2 * TAM_RETO + 1];
    a_hexadecimal(reto, sizeof(reto), reto_hex);
    char *texto = malloc(256);
    if (!texto) {
        return NULL;
    }
    if (ok) {
        snprintf(texto, 256, "{\"tipo\":\"hola\",\"id\":%s,\"ok\":true,\"protocolo\":2,\"reto\":\"%s\",\"sal\":\"%s\","
                 "\"iteraciones\":%d}", id_json, reto_hex, SEGURIDAD_SAL, SEGURIDAD_ITERACIONES);
    } else {
        snprintf(texto, 256, "{\"tipo\":\"hola\",\"id\":%s,\"ok\":false,\"error\":\"sin_espacio\","
                 "\"mensaje\":\"Demasiadas conexiones abiertas\"}", id_json);
    }
    return texto;
}

resultado_seguridad_t seguridad_descifrar(transporte_t transporte, int fd, const char *base64,
                                          char *salida, size_t tam_salida)
{
    uint8_t sobre[TAM_MAX_MENSAJE];
    size_t bytes = 0;
    if (mbedtls_base64_decode(sobre, sizeof(sobre), &bytes, (const uint8_t *)base64, strlen(base64)) != 0 ||
        bytes < TAM_NONCE + TAM_ETIQUETA + 2) {
        return SEGURIDAD_FORMATO;
    }

    xSemaphoreTake(s_seg.mutex, portMAX_DELAY);
    sesion_t *sesion = buscar_sesion(transporte, fd);
    if (!sesion) {
        xSemaphoreGive(s_seg.mutex);
        return SEGURIDAD_SIN_SESION;
    }
    size_t largo = 0;
    const psa_status_t estado = psa_aead_decrypt(sesion->clave, PSA_ALG_GCM, sobre, TAM_NONCE,
                                                 (const uint8_t *)AAD_APP, strlen(AAD_APP),
                                                 sobre + TAM_NONCE, bytes - TAM_NONCE,
                                                 (uint8_t *)salida, tam_salida - 1, &largo);
    if (estado != PSA_SUCCESS) {
        if (++sesion->fallos >= FALLOS_PARA_CERRAR) {
            ESP_LOGW(TAG, "Demasiados mensajes inválidos: sesión cerrada");
            cerrar_sesion(sesion);
        }
        xSemaphoreGive(s_seg.mutex);
        return SEGURIDAD_NO_AUTORIZADO;
    }
    salida[largo] = '\0';

    // El contador va dentro del mensaje cifrado: un mensaje repetido trae uno ya usado
    cJSON *raiz = cJSON_Parse(salida);
    const cJSON *n = cJSON_GetObjectItemCaseSensitive(raiz, "n");
    resultado_seguridad_t resultado = SEGURIDAD_FORMATO;
    if (cJSON_IsNumber(n) && n->valuedouble >= 1) {
        const uint64_t contador = (uint64_t)n->valuedouble;
        if (contador > sesion->n_recibido) {
            sesion->n_recibido = contador;
            sesion->fallos = 0;
            resultado = SEGURIDAD_OK;
        } else {
            resultado = SEGURIDAD_REPETIDO;
        }
    }
    cJSON_Delete(raiz);
    xSemaphoreGive(s_seg.mutex);
    return resultado;
}

// Cifra con la sesión ya tomada bajo el mutex; devuelve nonce || cifrado || etiqueta
static uint8_t *cifrar_con_sesion(sesion_t *sesion, const char *aad, const uint8_t *datos, size_t bytes,
                                  size_t *bytes_cifrados, uint32_t capacidades)
{
    const size_t total = TAM_NONCE + bytes + TAM_ETIQUETA;
    uint8_t *salida = heap_caps_malloc(total, capacidades);
    if (!salida) {
        return NULL;
    }
    esp_fill_random(salida, TAM_NONCE);
    size_t largo = 0;
    if (psa_aead_encrypt(sesion->clave, PSA_ALG_GCM, salida, TAM_NONCE, (const uint8_t *)aad, strlen(aad),
                         datos, bytes, salida + TAM_NONCE, total - TAM_NONCE, &largo) != PSA_SUCCESS) {
        free(salida);
        return NULL;
    }
    *bytes_cifrados = TAM_NONCE + largo;
    return salida;
}

char *seguridad_cifrar(transporte_t transporte, int fd, const char *json)
{
    if (!json || json[0] != '{') {
        return NULL;
    }
    xSemaphoreTake(s_seg.mutex, portMAX_DELAY);
    sesion_t *sesion = buscar_sesion(transporte, fd);
    if (!sesion) {
        xSemaphoreGive(s_seg.mutex);
        return NULL;
    }

    // Se antepone el contador al objeto: {"n":5,...resto del JSON}
    const size_t largo_json = strlen(json);
    char *claro = malloc(largo_json + 32);
    uint8_t *cifrado = NULL;
    size_t bytes_cifrados = 0;
    if (claro) {
        const uint64_t n = ++sesion->n_enviado;
        const int prefijo = snprintf(claro, 32, "{\"n\":%" PRIu64 "%s", n, json[1] == '}' ? "" : ",");
        memcpy(claro + prefijo, json + 1, largo_json);  // Incluye el '\0'
        cifrado = cifrar_con_sesion(sesion, AAD_ESP, (const uint8_t *)claro, strlen(claro), &bytes_cifrados,
                                    MALLOC_CAP_DEFAULT);
        free(claro);
    }
    xSemaphoreGive(s_seg.mutex);
    if (!cifrado) {
        return NULL;
    }

    size_t largo_base64 = 0;
    mbedtls_base64_encode(NULL, 0, &largo_base64, cifrado, bytes_cifrados);
    char *sobre = malloc(largo_base64 + 8);
    if (sobre) {
        size_t escrito = 0;
        memcpy(sobre, "{\"c\":\"", 6);
        mbedtls_base64_encode((uint8_t *)sobre + 6, largo_base64, &escrito, cifrado, bytes_cifrados);
        memcpy(sobre + 6 + escrito, "\"}", 3);
    }
    free(cifrado);
    return sobre;
}

uint8_t *seguridad_cifrar_binario(transporte_t transporte, int fd, const uint8_t *datos, size_t bytes,
                                  size_t *bytes_cifrados)
{
    xSemaphoreTake(s_seg.mutex, portMAX_DELAY);
    sesion_t *sesion = buscar_sesion(transporte, fd);
    uint8_t *cifrado = sesion ? cifrar_con_sesion(sesion, AAD_IMAGEN, datos, bytes, bytes_cifrados,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
                              : NULL;
    xSemaphoreGive(s_seg.mutex);
    return cifrado;
}

size_t seguridad_sesiones_activas(transporte_t transporte)
{
    if (!s_seg.mutex) {
        return 0;
    }
    size_t cantidad = 0;
    xSemaphoreTake(s_seg.mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_SESIONES; i++) {
        cantidad += s_seg.sesiones[i].activa && s_seg.sesiones[i].transporte == transporte ? 1 : 0;
    }
    xSemaphoreGive(s_seg.mutex);
    return cantidad;
}

bool seguridad_tiene_sesion(transporte_t transporte, int fd)
{
    xSemaphoreTake(s_seg.mutex, portMAX_DELAY);
    const bool tiene = buscar_sesion(transporte, fd) != NULL;
    xSemaphoreGive(s_seg.mutex);
    return tiene;
}

void seguridad_cerrar(transporte_t transporte, int fd)
{
    if (!s_seg.mutex) {
        return;
    }
    xSemaphoreTake(s_seg.mutex, portMAX_DELAY);
    sesion_t *sesion = buscar_sesion(transporte, fd);
    if (sesion) {
        cerrar_sesion(sesion);
    }
    xSemaphoreGive(s_seg.mutex);
}

void seguridad_cerrar_todas(void)
{
    xSemaphoreTake(s_seg.mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_SESIONES; i++) {
        if (s_seg.sesiones[i].activa) {
            cerrar_sesion(&s_seg.sesiones[i]);
        }
    }
    memset(s_seg.vence_token_us, 0, sizeof(s_seg.vence_token_us));
    xSemaphoreGive(s_seg.mutex);
}

esp_err_t seguridad_cambiar_clave(const char *nueva)
{
    ESP_RETURN_ON_FALSE(clave_valida(nueva), ESP_ERR_INVALID_ARG, TAG, "clave inválida");
    ESP_RETURN_ON_ERROR(guardar_nvs(NVS_CLAVE_ACTUAL, nueva), TAG, "no se pudo guardar la clave");
    xSemaphoreTake(s_seg.mutex, portMAX_DELAY);
    strlcpy(s_seg.clave, nueva, sizeof(s_seg.clave));
    const esp_err_t ret = derivar_clave_maestra(s_seg.clave);
    xSemaphoreGive(s_seg.mutex);
    ESP_LOGW(TAG, "Clave del equipo cambiada desde la app");
    return ret;
}

esp_err_t seguridad_restaurar_clave_fabrica(void)
{
    char fabrica[SEGURIDAD_CLAVE_MAX + 1];
    ESP_RETURN_ON_FALSE(leer_nvs(NVS_CLAVE_FABRICA, fabrica, sizeof(fabrica)), ESP_ERR_NOT_FOUND, TAG,
                        "sin clave de fábrica");
    ESP_RETURN_ON_ERROR(seguridad_cambiar_clave(fabrica), TAG, "no se pudo restaurar");
    seguridad_cerrar_todas();
    ESP_LOGW(TAG, "Clave restaurada a la de fábrica: %s", fabrica);
    return ESP_OK;
}

void seguridad_nuevo_token(uso_token_t uso, char token[SEGURIDAD_TAM_TOKEN])
{
    uint8_t aleatorio[(SEGURIDAD_TAM_TOKEN - 1) / 2];
    esp_fill_random(aleatorio, sizeof(aleatorio));
    a_hexadecimal(aleatorio, sizeof(aleatorio), token);
    xSemaphoreTake(s_seg.mutex, portMAX_DELAY);
    strlcpy(s_seg.token[uso], token, SEGURIDAD_TAM_TOKEN);
    s_seg.vence_token_us[uso] = esp_timer_get_time() +
                                (uso == TOKEN_VIVO ? VIGENCIA_TOKEN_VIVO_US : VIGENCIA_TOKEN_FOTO_US);
    xSemaphoreGive(s_seg.mutex);
}

bool seguridad_validar_token(uso_token_t uso, const char *token)
{
    if (!token || strlen(token) != SEGURIDAD_TAM_TOKEN - 1 || !s_seg.mutex) {
        return false;
    }
    xSemaphoreTake(s_seg.mutex, portMAX_DELAY);
    // Comparación de tiempo constante: no revela cuántos caracteres coinciden
    uint8_t diferencia = 0;
    for (int i = 0; i < SEGURIDAD_TAM_TOKEN - 1; i++) {
        diferencia |= (uint8_t)(token[i] ^ s_seg.token[uso][i]);
    }
    const bool valido = diferencia == 0 && esp_timer_get_time() < s_seg.vence_token_us[uso];
    if (valido && uso == TOKEN_VIVO) {
        s_seg.vence_token_us[uso] = 0;  // El video se abre una sola vez por token
    }
    xSemaphoreGive(s_seg.mutex);
    return valido;
}
