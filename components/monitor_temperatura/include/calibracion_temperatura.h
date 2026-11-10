#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "buffer_circular.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Compensación del campo de visión. El sensor reporta el promedio radiométrico (T^4)
 * de toda la escena, así que una olla pequeña a 2 m apenas mueve la lectura:
 *
 *   T_medida^4 = f * T_olla^4 + (1 - f) * T_fondo^4
 *
 * La calibración obtiene T_fondo (estufa apagada) y f (con una referencia de temperatura
 * conocida, p. ej. agua hirviendo) para invertir la ecuación y estimar T_olla.
 */
typedef struct {
    uint8_t version;
    bool fondo_valido;
    bool referencia_valida;
    float fondo_c;              // Lectura de la escena sin fuente caliente
    float ambiente_fondo_c;     // Temperatura interna del sensor durante esa medición
    float ruido_1s_c;           // Desviación estándar del promedio de 1 s (lectura cruda)
    float factor_llenado;       // f: fracción efectiva del campo de visión ocupada por la olla (0, 1]
    float ganancia_ruido;       // dT_estimada/dT_medida en la referencia (cuánto se amplifica el ruido)
} calibracion_temperatura_t;

/**
 * Paso 1, estufa apagada: obtiene la temperatura del fondo y el ruido del sensor.
 */
esp_err_t calibracion_calcular_fondo(const muestra_temperatura_t *muestras, size_t cantidad,
                                     size_t muestras_por_segundo, calibracion_temperatura_t *cal);

/**
 * Paso 2, recipiente a temperatura conocida y estable: obtiene el factor de llenado.
 */
esp_err_t calibracion_calcular_referencia(const muestra_temperatura_t *muestras, size_t cantidad,
                                          size_t muestras_por_segundo, float temperatura_referencia_c,
                                          calibracion_temperatura_t *cal);

/**
 * Convierte una lectura cruda en la temperatura estimada del recipiente.
 * Sin calibración completa devuelve la lectura sin cambios.
 */
float calibracion_estimar_temperatura(const calibracion_temperatura_t *cal, float medida_c, float ambiente_c);

esp_err_t calibracion_guardar(const calibracion_temperatura_t *cal);
esp_err_t calibracion_cargar(calibracion_temperatura_t *cal);
esp_err_t calibracion_borrar(void);

#ifdef __cplusplus
}
#endif
