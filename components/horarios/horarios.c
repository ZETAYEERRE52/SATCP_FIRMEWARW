#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "horarios.h"

static const char *TAG = "HORARIOS";

#define NVS_ESPACIO         "satcp"
#define NVS_CLAVE_HORARIOS  "horarios"
#define SERVIDOR_SNTP       "pool.ntp.org"
#define SIN_MINUTO          (-1)

// Formato guardado en NVS: no cambiar sin migrar los datos existentes
typedef struct {
    uint8_t usado;
    uint8_t activo;
    uint8_t hora;
    uint8_t minuto;
    uint8_t dias;
} horario_guardado_t;

static struct {
    SemaphoreHandle_t mutex;
    horario_guardado_t lista[HORARIOS_MAXIMO];
    int64_t minuto_ejecutado[HORARIOS_MAXIMO];  // Minuto (desde 1970) en que se ejecutó cada horario
    int64_t ultimo_minuto_revisado;
    volatile bool hora_valida;
} s_horarios = {
    .ultimo_minuto_revisado = SIN_MINUTO,
};

static void hora_sincronizada(struct timeval *tv)
{
    s_horarios.hora_valida = true;
    const time_t ahora = tv->tv_sec;
    struct tm local;
    localtime_r(&ahora, &local);
    ESP_LOGI(TAG, "Hora sincronizada: %04d-%02d-%02d %02d:%02d:%02d (%s)", local.tm_year + 1900,
             local.tm_mon + 1, local.tm_mday, local.tm_hour, local.tm_min, local.tm_sec, HORARIOS_ZONA_HORARIA);
}

static esp_err_t guardar_en_nvs(void)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_ESPACIO, NVS_READWRITE, &nvs), TAG, "NVS");
    esp_err_t ret = nvs_set_blob(nvs, NVS_CLAVE_HORARIOS, s_horarios.lista, sizeof(s_horarios.lista));
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return ret;
}

esp_err_t horarios_iniciar(void)
{
    ESP_RETURN_ON_FALSE(!s_horarios.mutex, ESP_ERR_INVALID_STATE, TAG, "ya iniciado");
    s_horarios.mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_horarios.mutex, ESP_ERR_NO_MEM, TAG, "sin memoria");

    setenv("TZ", HORARIOS_ZONA_HORARIA, 1);
    tzset();
    for (int i = 0; i < HORARIOS_MAXIMO; i++) {
        s_horarios.minuto_ejecutado[i] = SIN_MINUTO;
    }

    nvs_handle_t nvs;
    int cargados = 0;
    if (nvs_open(NVS_ESPACIO, NVS_READONLY, &nvs) == ESP_OK) {
        size_t tam = sizeof(s_horarios.lista);
        if (nvs_get_blob(nvs, NVS_CLAVE_HORARIOS, s_horarios.lista, &tam) != ESP_OK ||
            tam != sizeof(s_horarios.lista)) {
            memset(s_horarios.lista, 0, sizeof(s_horarios.lista));
        }
        nvs_close(nvs);
    }
    for (int i = 0; i < HORARIOS_MAXIMO; i++) {
        cargados += s_horarios.lista[i].usado ? 1 : 0;
    }
    ESP_LOGI(TAG, "%d horarios guardados; se ejecutan cuando haya hora por SNTP", cargados);
    return ESP_OK;
}

void horarios_sincronizar_hora(void)
{
    if (esp_sntp_enabled()) {
        if (!s_horarios.hora_valida) {
            esp_sntp_restart();
        }
        return;
    }
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, SERVIDOR_SNTP);
    sntp_set_time_sync_notification_cb(hora_sincronizada);
    esp_sntp_init();
    ESP_LOGI(TAG, "Sincronizando la hora con %s...", SERVIDOR_SNTP);
}

bool horarios_hora_valida(void)
{
    return s_horarios.hora_valida;
}

bool horarios_obtener(int indice, horario_t *horario)
{
    if (indice < 0 || indice >= HORARIOS_MAXIMO || !s_horarios.mutex) {
        return false;
    }
    xSemaphoreTake(s_horarios.mutex, portMAX_DELAY);
    const horario_guardado_t g = s_horarios.lista[indice];
    xSemaphoreGive(s_horarios.mutex);
    if (!g.usado) {
        return false;
    }
    *horario = (horario_t){ .hora = g.hora, .minuto = g.minuto, .dias = g.dias, .activo = g.activo };
    return true;
}

esp_err_t horarios_guardar(int indice, const horario_t *horario, int *indice_guardado)
{
    ESP_RETURN_ON_FALSE(s_horarios.mutex, ESP_ERR_INVALID_STATE, TAG, "no iniciado");
    if (!horario || indice >= HORARIOS_MAXIMO || horario->hora > 23 || horario->minuto > 59 ||
        horario->dias == 0 || horario->dias > HORARIOS_TODOS_LOS_DIAS) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_horarios.mutex, portMAX_DELAY);
    if (indice < 0) {
        for (int i = 0; i < HORARIOS_MAXIMO && indice < 0; i++) {
            if (!s_horarios.lista[i].usado) {
                indice = i;
            }
        }
    }
    esp_err_t ret = ESP_ERR_NO_MEM;
    if (indice >= 0) {
        const horario_guardado_t previo = s_horarios.lista[indice];
        s_horarios.lista[indice] = (horario_guardado_t){
            .usado = 1, .activo = horario->activo, .hora = horario->hora,
            .minuto = horario->minuto, .dias = horario->dias,
        };
        s_horarios.minuto_ejecutado[indice] = SIN_MINUTO;
        ret = guardar_en_nvs();
        if (ret != ESP_OK) {
            s_horarios.lista[indice] = previo;
        }
    }
    xSemaphoreGive(s_horarios.mutex);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Horario %d: %02u:%02u, días 0x%02X, %s", indice, horario->hora, horario->minuto,
                 horario->dias, horario->activo ? "activo" : "inactivo");
        if (indice_guardado) {
            *indice_guardado = indice;
        }
    }
    return ret;
}

esp_err_t horarios_borrar(int indice)
{
    ESP_RETURN_ON_FALSE(s_horarios.mutex, ESP_ERR_INVALID_STATE, TAG, "no iniciado");
    if (indice < 0 || indice >= HORARIOS_MAXIMO) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_horarios.mutex, portMAX_DELAY);
    const horario_guardado_t previo = s_horarios.lista[indice];
    memset(&s_horarios.lista[indice], 0, sizeof(s_horarios.lista[indice]));
    esp_err_t ret = guardar_en_nvs();
    if (ret != ESP_OK) {
        s_horarios.lista[indice] = previo;
    }
    xSemaphoreGive(s_horarios.mutex);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Horario %d borrado", indice);
    }
    return ret;
}

bool horarios_toca_cierre(int *indice)
{
    if (!s_horarios.hora_valida || !s_horarios.mutex) {
        return false;
    }
    const time_t ahora = time(NULL);
    const int64_t minuto_actual = (int64_t)ahora / 60;
    if (minuto_actual == s_horarios.ultimo_minuto_revisado) {
        return false;
    }
    s_horarios.ultimo_minuto_revisado = minuto_actual;

    struct tm local;
    localtime_r(&ahora, &local);
    const uint8_t bit_dia = 1u << ((local.tm_wday + 6) % 7);   // tm_wday: 0 = domingo; aquí bit 0 = lunes

    bool toca = false;
    xSemaphoreTake(s_horarios.mutex, portMAX_DELAY);
    for (int i = 0; i < HORARIOS_MAXIMO; i++) {
        const horario_guardado_t *h = &s_horarios.lista[i];
        // Si la hora se reajusta hacia atrás, el mismo minuto no vuelve a ejecutar el horario
        if (h->usado && h->activo && h->hora == local.tm_hour && h->minuto == local.tm_min &&
            (h->dias & bit_dia) && s_horarios.minuto_ejecutado[i] != minuto_actual) {
            s_horarios.minuto_ejecutado[i] = minuto_actual;
            if (!toca) {
                toca = true;
                *indice = i;
            }
        }
    }
    xSemaphoreGive(s_horarios.mutex);
    return toca;
}
