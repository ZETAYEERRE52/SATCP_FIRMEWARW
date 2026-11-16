#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Mismo orden que CLASES en el notebook (modelo/parametros.json)
typedef enum {
    CLASE_FUEGO = 0,
    CLASE_HUMO,
    CLASE_NORMAL,
    NUM_CLASES,
} clase_vision_t;

typedef struct {
    clase_vision_t clase;               // La de mayor probabilidad (la vigilancia aplica además VISION_PROBABILIDAD_MINIMA)
    float probabilidad[NUM_CLASES];
    uint32_t tiempo_ms;                 // Decodificación + inferencia
} resultado_clasificacion_t;

/**
 * Carga el modelo en TensorFlow Lite Micro con la memoria de trabajo en PSRAM.
 */
esp_err_t clasificador_iniciar(void);

bool clasificador_disponible(void);

/**
 * Decodifica el JPEG, lo reduce a la entrada del modelo y lo clasifica.
 * Se puede llamar desde varias tareas: las llamadas se atienden de una en una.
 */
esp_err_t clasificador_clasificar_jpeg(const uint8_t *jpeg, size_t longitud, resultado_clasificacion_t *resultado);

const char *clasificador_nombre_clase(clase_vision_t clase);

#ifdef __cplusplus
}
#endif
