#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int64_t marca_us;           // esp_timer_get_time() al momento de la lectura
    float temp_objeto_c;        // Superficie apuntada (sartén, olla, hornilla)
    float temp_ambiente_c;      // Temperatura interna del sensor
} muestra_temperatura_t;

/**
 * Buffer circular de muestras. Cuando se llena, la muestra más nueva
 * sobrescribe a la más antigua. No es thread-safe: el llamador protege el acceso.
 */
typedef struct {
    muestra_temperatura_t *datos;
    size_t capacidad;
    size_t cabeza;              // Próxima posición de escritura
    size_t cantidad;            // Muestras válidas almacenadas
} buffer_circular_t;

esp_err_t buffer_circular_crear(buffer_circular_t *buffer, size_t capacidad);
void buffer_circular_liberar(buffer_circular_t *buffer);
void buffer_circular_vaciar(buffer_circular_t *buffer);

void buffer_circular_agregar(buffer_circular_t *buffer, const muestra_temperatura_t *muestra);

size_t buffer_circular_cantidad(const buffer_circular_t *buffer);
bool buffer_circular_esta_lleno(const buffer_circular_t *buffer);

/**
 * Obtiene una muestra por antigüedad: 0 = la más reciente, 1 = la anterior, etc.
 */
bool buffer_circular_obtener(const buffer_circular_t *buffer, size_t antiguedad, muestra_temperatura_t *muestra);

/**
 * Promedia `cantidad` muestras empezando en `antiguedad` y yendo hacia atrás.
 * Devuelve las temperaturas promedio (objeto y ambiente) y la marca de tiempo promedio.
 */
bool buffer_circular_promediar(const buffer_circular_t *buffer, size_t antiguedad, size_t cantidad,
                               float *temp_promedio_c, float *ambiente_promedio_c, int64_t *marca_promedio_us);

/**
 * Copia hasta `maximo` muestras en orden cronológico (la más antigua primero).
 * Devuelve cuántas copió.
 */
size_t buffer_circular_copiar(const buffer_circular_t *buffer, muestra_temperatura_t *destino, size_t maximo);

#ifdef __cplusplus
}
#endif
