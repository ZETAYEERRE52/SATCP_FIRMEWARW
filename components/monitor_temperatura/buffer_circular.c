#include <stdlib.h>
#include <string.h>
#include "buffer_circular.h"

// Índice físico de la muestra con la antigüedad indicada (0 = más reciente)
static size_t indice_por_antiguedad(const buffer_circular_t *buffer, size_t antiguedad)
{
    return (buffer->cabeza + buffer->capacidad - 1 - antiguedad) % buffer->capacidad;
}

esp_err_t buffer_circular_crear(buffer_circular_t *buffer, size_t capacidad)
{
    if (!buffer || capacidad == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    buffer->datos = calloc(capacidad, sizeof(muestra_temperatura_t));
    if (!buffer->datos) {
        return ESP_ERR_NO_MEM;
    }
    buffer->capacidad = capacidad;
    buffer->cabeza = 0;
    buffer->cantidad = 0;
    return ESP_OK;
}

void buffer_circular_liberar(buffer_circular_t *buffer)
{
    free(buffer->datos);
    memset(buffer, 0, sizeof(*buffer));
}

void buffer_circular_vaciar(buffer_circular_t *buffer)
{
    buffer->cabeza = 0;
    buffer->cantidad = 0;
}

void buffer_circular_agregar(buffer_circular_t *buffer, const muestra_temperatura_t *muestra)
{
    buffer->datos[buffer->cabeza] = *muestra;
    buffer->cabeza = (buffer->cabeza + 1) % buffer->capacidad;
    if (buffer->cantidad < buffer->capacidad) {
        buffer->cantidad++;
    }
}

size_t buffer_circular_cantidad(const buffer_circular_t *buffer)
{
    return buffer->cantidad;
}

bool buffer_circular_esta_lleno(const buffer_circular_t *buffer)
{
    return buffer->cantidad == buffer->capacidad;
}

bool buffer_circular_obtener(const buffer_circular_t *buffer, size_t antiguedad, muestra_temperatura_t *muestra)
{
    if (antiguedad >= buffer->cantidad) {
        return false;
    }
    *muestra = buffer->datos[indice_por_antiguedad(buffer, antiguedad)];
    return true;
}

bool buffer_circular_promediar(const buffer_circular_t *buffer, size_t antiguedad, size_t cantidad,
                               float *temp_promedio_c, float *ambiente_promedio_c, int64_t *marca_promedio_us)
{
    if (cantidad == 0 || antiguedad + cantidad > buffer->cantidad) {
        return false;
    }

    float suma_temp = 0.0f;
    float suma_ambiente = 0.0f;
    int64_t suma_marca = 0;
    for (size_t i = 0; i < cantidad; i++) {
        const muestra_temperatura_t *m = &buffer->datos[indice_por_antiguedad(buffer, antiguedad + i)];
        suma_temp += m->temp_objeto_c;
        suma_ambiente += m->temp_ambiente_c;
        suma_marca += m->marca_us;
    }
    *temp_promedio_c = suma_temp / (float)cantidad;
    *ambiente_promedio_c = suma_ambiente / (float)cantidad;
    *marca_promedio_us = suma_marca / (int64_t)cantidad;
    return true;
}

size_t buffer_circular_copiar(const buffer_circular_t *buffer, muestra_temperatura_t *destino, size_t maximo)
{
    size_t total = buffer->cantidad < maximo ? buffer->cantidad : maximo;
    // Las `total` más recientes, de la más antigua a la más nueva
    for (size_t i = 0; i < total; i++) {
        destino[i] = buffer->datos[indice_por_antiguedad(buffer, total - 1 - i)];
    }
    return total;
}
