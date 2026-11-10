#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Toma una foto cada `intervalo_s` y la guarda en /sdcard/dataset/ para reentrenar el modelo
 * con imágenes de la cocina real (estufa apagada, hornillas encendidas, recipientes, etc.).
 */
esp_err_t captura_dataset_iniciar(uint32_t intervalo_s);

#ifdef __cplusplus
}
#endif
