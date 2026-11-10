#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "almacenamiento_sd.h"
#include "camara_vision.h"
#include "foto_clasificada.h"

static const char *TAG = "FOTO";

#define ESPERA_CAMARA_MS    3000

esp_err_t foto_clasificada_tomar(const char *carpeta, foto_clasificada_t *foto, uint8_t **jpeg)
{
    ESP_RETURN_ON_FALSE(carpeta && foto, ESP_ERR_INVALID_ARG, TAG, "argumentos inválidos");
    ESP_RETURN_ON_FALSE(camara_disponible(), ESP_ERR_INVALID_STATE, TAG, "cámara no disponible");
    memset(foto, 0, sizeof(*foto));
    if (jpeg) {
        *jpeg = NULL;
    }

    camera_fb_t *cuadro = camara_capturar(pdMS_TO_TICKS(ESPERA_CAMARA_MS));
    ESP_RETURN_ON_FALSE(cuadro, ESP_ERR_TIMEOUT, TAG, "cámara ocupada o sin cuadro");
    foto->bytes = cuadro->len;

    foto->clasificada = clasificador_disponible() &&
                        clasificador_clasificar_jpeg(cuadro->buf, cuadro->len, &foto->clasificacion) == ESP_OK;

    if (sd_disponible()) {
        char etiqueta[24];
        if (foto->clasificada) {
            const resultado_clasificacion_t *c = &foto->clasificacion;
            snprintf(etiqueta, sizeof(etiqueta), "%s_%d", clasificador_nombre_clase(c->clase),
                     (int)(c->probabilidad[c->clase] * 100 + 0.5f));
        } else {
            snprintf(etiqueta, sizeof(etiqueta), "sinclasificar");
        }
        foto->guardada = sd_guardar_imagen(carpeta, etiqueta, cuadro->buf, cuadro->len,
                                           foto->ruta, sizeof(foto->ruta)) == ESP_OK;
    }

    // Copia para enviarla por la red sin retener la cámara durante la transmisión
    if (jpeg) {
        *jpeg = heap_caps_malloc(cuadro->len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (*jpeg) {
            memcpy(*jpeg, cuadro->buf, cuadro->len);
        }
    }
    camara_liberar(cuadro);

    if (foto->clasificada) {
        ESP_LOGI(TAG, "Foto: %s %.0f%% (%lu ms)%s%s", clasificador_nombre_clase(foto->clasificacion.clase),
                 foto->clasificacion.probabilidad[foto->clasificacion.clase] * 100,
                 (unsigned long)foto->clasificacion.tiempo_ms, foto->guardada ? " -> " : "",
                 foto->guardada ? foto->ruta : "");
    }
    return ESP_OK;
}
