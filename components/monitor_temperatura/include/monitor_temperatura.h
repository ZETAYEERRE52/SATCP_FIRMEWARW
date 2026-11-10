#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "buffer_circular.h"
#include "calibracion_temperatura.h"
#include "mlx90614.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MONITOR_FRECUENCIA_HZ       10
#define MONITOR_PERIODO_MS          (1000 / MONITOR_FRECUENCIA_HZ)                  // 100 ms
#define MONITOR_VENTANA_S           60
#define MONITOR_TAM_BUFFER          (MONITOR_VENTANA_S * MONITOR_FRECUENCIA_HZ)     // 600 muestras

typedef enum {
    EVENTO_INCREMENTO_ANOMALO,      // Subida rápida sostenida: validar presencia humana
    EVENTO_TEMPERATURA_CRITICA,     // Superó el umbral absoluto
    EVENTO_TEMPERATURA_NORMALIZADA, // La condición anómala desapareció
    EVENTO_FALLA_SENSOR,            // Sin lecturas válidas (falla segura: tratar como alerta)
    EVENTO_SENSOR_RECUPERADO,
} tipo_evento_temperatura_t;

typedef struct {
    tipo_evento_temperatura_t tipo;
    float temperatura_c;            // Promedio del último segundo
    float referencia_c;             // Promedio de hace `ventana_tendencia_s` (NAN si no hay datos)
    float tasa_c_por_min;           // Velocidad de subida
    int64_t marca_us;
} evento_temperatura_t;

typedef struct {
    gpio_num_t pin_sda;
    gpio_num_t pin_scl;
    i2c_port_num_t puerto_i2c;
    uint8_t direccion_sensor;
    uint32_t velocidad_i2c_hz;      // 10-100 kHz; con cable largo conviene bajarla

    float umbral_critico_c;         // Por encima: evento crítico inmediato (tras confirmación)
    float umbral_minimo_c;          // Por debajo no se evalúa la tasa (evita alertas con calor leve)
    float tasa_maxima_c_por_min;    // Subida considerada anómala
    uint16_t ventana_tendencia_s;   // Separación entre los dos promedios que forman la tasa
    float umbral_salto_c;           // Subida brusca de la lectura del sensor que dispara la alerta
    uint16_t ventana_salto_s;       // En cuánto tiempo debe ocurrir esa subida
    uint16_t muestras_confirmacion; // Muestras seguidas en condición anómala antes de alertar
    uint16_t muestras_normalizacion;// Muestras seguidas en normalidad antes de declarar normalizado

    UBaseType_t prioridad_tarea;
    BaseType_t nucleo_tarea;

    float margen_superacion_c;      // Con línea base: superarla por este margen es anómalo
    float margen_descarte_c;        // Con línea base: bajar este margen la descarta (cocción terminada)
} monitor_temperatura_config_t;

#define MONITOR_TEMPERATURA_CONFIG_POR_DEFECTO() {          \
    .pin_sda = GPIO_NUM_41,                                 \
    .pin_scl = GPIO_NUM_42,                                 \
    .puerto_i2c = I2C_NUM_0,                                \
    .direccion_sensor = MLX90614_DIRECCION_POR_DEFECTO,     \
    .velocidad_i2c_hz = 50000,                              \
    .umbral_critico_c = 250.0f,                             \
    .umbral_minimo_c = 50.0f,                               \
    .tasa_maxima_c_por_min = 20.0f,                         \
    .ventana_tendencia_s = 30,                              \
    .umbral_salto_c = 4.0f,                                 \
    .ventana_salto_s = 10,                                  \
    .muestras_confirmacion = 5,                             \
    .muestras_normalizacion = 300,                          \
    .prioridad_tarea = 10,                                  \
    .nucleo_tarea = 0,      /* Núcleo del sistema; el 1 es de las comunicaciones */                                      \
    .margen_superacion_c = 10.0f,                           \
    .margen_descarte_c = 20.0f,                             \
}

/**
 * Crea el bus I2C, inicializa el MLX90614 y arranca la tarea de muestreo a 10 Hz.
 */
esp_err_t monitor_temperatura_iniciar(const monitor_temperatura_config_t *config);

/**
 * Bloquea hasta recibir un evento o hasta que pase `espera`. Devuelve true si hubo evento.
 */
bool monitor_temperatura_esperar_evento(evento_temperatura_t *evento, TickType_t espera);

/**
 * Copia la muestra más reciente. Devuelve false si todavía no hay lecturas o el sensor está en falla.
 */
bool monitor_temperatura_obtener_ultima_muestra(muestra_temperatura_t *muestra);

/**
 * false si el MLX90614 dejó de responder (o nunca respondió desde el arranque). El monitor sigue
 * reintentando y vuelve a true solo cuando el sensor responde de nuevo.
 */
bool monitor_temperatura_sensor_ok(void);

/**
 * La máquina empezó a revisar la anomalía (radar y cámara). Si después la lectura sube
 * umbral_salto_c sobre la más baja desde este momento, se publica otro EVENTO_INCREMENTO_ANOMALO
 * aunque la anomalía siga activa.
 */
void monitor_temperatura_marcar_revision(void);

/**
 * Copia el historial (hasta 60 s) en orden cronológico. Devuelve cuántas muestras copió.
 */
size_t monitor_temperatura_copiar_historial(muestra_temperatura_t *destino, size_t maximo);

/**
 * Temperatura estimada del recipiente para una muestra (compensa el campo de visión).
 * Sin calibración completa devuelve la lectura cruda.
 */
float monitor_temperatura_estimar(const muestra_temperatura_t *muestra);

/**
 * Promedio del último segundo, ya convertido a temperatura estimada del recipiente.
 */
bool monitor_temperatura_obtener_estimada(float *temperatura_c);

/**
 * Línea base: temperatura estable ya verificada sin fuego ni humo. Desde ese momento solo
 * se considera anómalo superarla por `margen_superacion_c` (o subir rápido por encima de ella).
 * Se descarta sola si la temperatura baja `margen_descarte_c` por debajo.
 */
void monitor_temperatura_establecer_linea_base(float temperatura_c);
void monitor_temperatura_borrar_linea_base(void);
bool monitor_temperatura_obtener_linea_base(float *temperatura_c);

/**
 * Paso 1 (bloquea 60 s): estufa apagada y sin personas frente al sensor.
 * Mide el fondo y el ruido; el umbral de tasa se ajusta para no disparar con el ruido.
 */
esp_err_t monitor_temperatura_calibrar_fondo(void);

/**
 * Paso 2 (bloquea 30 s): recipiente a temperatura conocida y ya estable (p. ej. agua hirviendo).
 * Calcula qué fracción del campo de visión ocupa. Se guarda en NVS.
 */
esp_err_t monitor_temperatura_calibrar_referencia(float temperatura_referencia_c);

esp_err_t monitor_temperatura_borrar_calibracion(void);
bool monitor_temperatura_obtener_calibracion(calibracion_temperatura_t *cal);

#ifdef __cplusplus
}
#endif
