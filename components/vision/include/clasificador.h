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

// Probabilidad de fuego a partir de la cual un cuadro cuenta como fuego aunque gane otra clase.
// La salida int8 del modelo avanza en pasos de 1/256 (0,39 %): 0.01 equivale a 3 pasos
#define CLASIFICADOR_UMBRAL_FUEGO   0.01f

typedef struct {
    clase_vision_t clase;               // Fuego si supera CLASIFICADOR_UMBRAL_FUEGO; si no, la de mayor probabilidad
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
