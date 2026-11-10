#include <inttypes.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "valvula.h"

static const char *TAG = "VALVULA";

#define NVS_ESPACIO             "satcp"
#define NVS_CLAVE_CERRADA       "valv_cerrada"
#define ESPERA_POSICION_MS      1500    // Tiempo para que la válvula termine de moverse antes de leer el sensor

static struct {
    valvula_config_t config;
    bool iniciada;
    bool cerrada_supuesta;
} s_valvula;

static bool hay_sensor(void)
{
    return s_valvula.config.pin_posicion != GPIO_NUM_NC;
}

static bool leer_sensor_cerrada(void)
{
    return gpio_get_level(s_valvula.config.pin_posicion) == s_valvula.config.nivel_cerrada;
}

static void guardar_supuesta(bool cerrada)
{
    s_valvula.cerrada_supuesta = cerrada;
    nvs_handle_t nvs;
    if (nvs_open(NVS_ESPACIO, NVS_READWRITE, &nvs) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo guardar el estado de la válvula");
        return;
    }
    if (nvs_set_u8(nvs, NVS_CLAVE_CERRADA, cerrada) != ESP_OK || nvs_commit(nvs) != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo guardar el estado de la válvula");
    }
    nvs_close(nvs);
}

static void pulsar_boton(void)
{
    ESP_LOGW(TAG, "Pulsando el botón de la válvula (%" PRIu32 " ms)", s_valvula.config.duracion_pulso_ms);
    gpio_set_level(s_valvula.config.pin_pulso, 1);
    vTaskDelay(pdMS_TO_TICKS(s_valvula.config.duracion_pulso_ms));
    gpio_set_level(s_valvula.config.pin_pulso, 0);
}

esp_err_t valvula_iniciar(const valvula_config_t *config)
{
    ESP_RETURN_ON_FALSE(config && config->duracion_pulso_ms > 0, ESP_ERR_INVALID_ARG, TAG, "configuración inválida");
    s_valvula.config = *config;

    // Primero el nivel bajo y luego la salida: el optoacoplador no debe parpadear al arrancar
    gpio_set_level(config->pin_pulso, 0);
    const gpio_config_t config_pulso = {
        .pin_bit_mask = 1ULL << config->pin_pulso,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config_pulso), TAG, "pin del optoacoplador");

    if (hay_sensor()) {
        const gpio_config_t config_posicion = {
            .pin_bit_mask = 1ULL << config->pin_posicion,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&config_posicion), TAG, "pin del sensor de posición");
    }

    uint8_t cerrada = 0;
    nvs_handle_t nvs;
    if (nvs_open(NVS_ESPACIO, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, NVS_CLAVE_CERRADA, &cerrada);
        nvs_close(nvs);
    }
    s_valvula.cerrada_supuesta = cerrada;
    s_valvula.iniciada = true;

    ESP_LOGI(TAG, "Válvula en GPIO %d (pulso %" PRIu32 " ms), %s: %s", config->pin_pulso,
             config->duracion_pulso_ms, hay_sensor() ? "con sensor de posición" : "sin sensor (estado supuesto)",
             valvula_cerrada() ? "CERRADA" : "abierta");
    return ESP_OK;
}

esp_err_t valvula_cerrar(void)
{
    ESP_RETURN_ON_FALSE(s_valvula.iniciada, ESP_ERR_INVALID_STATE, TAG, "válvula no iniciada");
    if (valvula_cerrada()) {
        // El botón alterna: pulsar ahora abriría el gas
        ESP_LOGW(TAG, "La válvula ya está cerrada: no se pulsa");
        return ESP_OK;
    }

    pulsar_boton();
    if (!hay_sensor()) {
        guardar_supuesta(true);
        return ESP_OK;
    }

    vTaskDelay(pdMS_TO_TICKS(ESPERA_POSICION_MS));
    if (!leer_sensor_cerrada()) {
        ESP_LOGE(TAG, "El sensor indica que la válvula sigue abierta");
        return ESP_FAIL;
    }
    guardar_supuesta(true);
    return ESP_OK;
}

bool valvula_cerrada(void)
{
    return hay_sensor() ? leer_sensor_cerrada() : s_valvula.cerrada_supuesta;
}

bool valvula_tiene_sensor(void)
{
    return hay_sensor();
}

esp_err_t valvula_registrar_apertura(void)
{
    ESP_RETURN_ON_FALSE(s_valvula.iniciada, ESP_ERR_INVALID_STATE, TAG, "válvula no iniciada");
    if (hay_sensor() && leer_sensor_cerrada()) {
        return ESP_ERR_INVALID_STATE;
    }
    guardar_supuesta(false);
    ESP_LOGI(TAG, "Válvula registrada como abierta (abierta a mano por el usuario)");
    return ESP_OK;
}
