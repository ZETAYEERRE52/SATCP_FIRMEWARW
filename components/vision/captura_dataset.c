#include <inttypes.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "almacenamiento_sd.h"
#include "camara_vision.h"
#include "captura_dataset.h"

static const char *TAG = "DATASET";

#define TAM_PILA_TAREA      4096
#define PRIORIDAD_TAREA     3       // Por debajo del monitor y la máquina de estados
#define NUCLEO_TAREA        0       // Núcleo del sistema; el 1 es de las comunicaciones
#define ESPERA_CAMARA_MS    2000    // Si la fase visual está usando la cámara, se salta esta captura

static uint32_t s_intervalo_s;

static void tarea_captura_dataset(void *arg)
{
    uint32_t guardadas = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(s_intervalo_s * 1000));
        if (!sd_disponible()) {
            continue;
        }

        camera_fb_t *cuadro = camara_capturar(pdMS_TO_TICKS(ESPERA_CAMARA_MS));
        if (!cuadro) {
            continue;
        }
        const esp_err_t ret = sd_guardar_imagen(SD_CARPETA_DATASET, "img", cuadro->buf, cuadro->len, NULL, 0);
        camara_liberar(cuadro);

        if (ret == ESP_OK && ++guardadas % 20 == 0) {
            ESP_LOGI(TAG, "%" PRIu32 " imágenes guardadas para el dataset", guardadas);
        }
    }
}

esp_err_t captura_dataset_iniciar(uint32_t intervalo_s)
{
    ESP_RETURN_ON_FALSE(intervalo_s > 0, ESP_ERR_INVALID_ARG, TAG, "intervalo debe ser > 0");
    ESP_RETURN_ON_FALSE(camara_disponible(), ESP_ERR_INVALID_STATE, TAG, "la cámara no está iniciada");

    s_intervalo_s = intervalo_s;
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(tarea_captura_dataset, "captura_dataset", TAM_PILA_TAREA, NULL,
                                                PRIORIDAD_TAREA, NULL, NUCLEO_TAREA) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "no se pudo crear la tarea");
    ESP_LOGI(TAG, "Captura para dataset cada %" PRIu32 " s en /sdcard/%s", intervalo_s, SD_CARPETA_DATASET);
    return ESP_OK;
}
