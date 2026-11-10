#pragma once

#include "esp_err.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int pin_xclk;
    int pin_sccb_sda;
    int pin_sccb_scl;
    int pin_d[8];               // D0..D7 (Y2..Y9 en la nomenclatura de la placa)
    int pin_vsync;
    int pin_href;
    int pin_pclk;
    int xclk_hz;
    framesize_t resolucion;
    int calidad_jpeg;           // 0-63, menor = mejor calidad
} camara_config_t;

/**
 * Inicializa la cámara en JPEG con los buffers en PSRAM y descarta los primeros cuadros.
 */
esp_err_t camara_iniciar(const camara_config_t *config);

/**
 * Captura un cuadro JPEG. Bloquea la cámara para otras tareas hasta camara_liberar().
 * Devuelve NULL si la cámara no está lista o falla la captura.
 */
camera_fb_t *camara_capturar(TickType_t espera);

void camara_liberar(camera_fb_t *cuadro);

#define CAMARA_RESOLUCION_VIVO  FRAMESIZE_QVGA  // 320x240: ~4 veces menos datos que VGA, menos retraso

/**
 * Vista en vivo: baja la resolución mientras alguien mira y la restaura al terminar. Las fotos que
 * se tomen mientras tanto (fase visual, dataset) salen en esta resolución; el clasificador lo admite.
 */
esp_err_t camara_modo_vivo(bool activo, TickType_t espera);

bool camara_disponible(void);

#ifdef __cplusplus
}
#endif
