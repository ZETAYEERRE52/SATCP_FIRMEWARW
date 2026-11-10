#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "clasificador.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool clasificada;
    resultado_clasificacion_t clasificacion;
    bool guardada;
    char ruta[128];             // Ruta en la SD si se guardó
    size_t bytes;               // Tamaño del JPEG
} foto_clasificada_t;

/**
 * Toma una foto, la clasifica y la guarda en /sdcard/<carpeta>/ con la clase predicha en el nombre.
 * Si `jpeg` no es NULL, devuelve una copia del JPEG en PSRAM que el llamador libera con free().
 * Devuelve ESP_ERR_TIMEOUT si la cámara está ocupada (por ejemplo, en plena fase visual).
 */
esp_err_t foto_clasificada_tomar(const char *carpeta, foto_clasificada_t *foto, uint8_t **jpeg);

#ifdef __cplusplus
}
#endif
