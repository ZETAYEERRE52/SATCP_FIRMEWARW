#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SD_PUNTO_MONTAJE    "/sdcard"

// Carpetas para el reentrenamiento del modelo
#define SD_CARPETA_DATASET      "dataset"       // Capturas periódicas sin etiquetar
#define SD_CARPETA_EVIDENCIA    "evidencia"     // Fotos de la fase visual, nombradas con el resultado
#define SD_CARPETA_TELEFONO     "telefono"      // Fotos pedidas desde el teléfono, nombradas con el resultado

typedef struct {
    gpio_num_t pin_clk;
    gpio_num_t pin_cmd;
    gpio_num_t pin_d0;
} sd_config_t;

/**
 * Monta la SD en modo de 1 línea. Cada arranque guarda en su propia subcarpeta (sNNNN),
 * numerada con un contador en NVS, porque aún no hay reloj de tiempo real.
 */
esp_err_t sd_iniciar(const sd_config_t *config);

bool sd_disponible(void);

/**
 * Guarda un JPEG en /sdcard/<carpeta>/sNNNN/<etiqueta>_NNNNNN.jpg.
 * Si `ruta` no es NULL, devuelve ahí la ruta del archivo.
 */
esp_err_t sd_guardar_imagen(const char *carpeta, const char *etiqueta, const uint8_t *datos, size_t longitud,
                            char *ruta, size_t tam_ruta);

#ifdef __cplusplus
}
#endif
