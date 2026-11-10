#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MLX90614_DIRECCION_POR_DEFECTO  0x5A
#define MLX90614_VELOCIDAD_MAXIMA_HZ    100000   // El MLX90614 es SMBus: máximo 100 kHz

// Comandos SMBus (RAM = 0x00 | dir, EEPROM = 0x20 | dir)
#define MLX90614_RAM_TA                 0x06     // Temperatura ambiente (del encapsulado)
#define MLX90614_RAM_TOBJ1              0x07     // Temperatura del objeto, zona 1
#define MLX90614_RAM_TOBJ2              0x08     // Temperatura del objeto, zona 2 (solo modelos de doble zona)
#define MLX90614_EEPROM_EMISIVIDAD      0x24

typedef struct mlx90614_t *mlx90614_handle_t;

typedef struct {
    uint8_t direccion;          // Dirección I2C de 7 bits (normalmente 0x5A)
    uint32_t velocidad_hz;      // Frecuencia SCL, <= MLX90614_VELOCIDAD_MAXIMA_HZ
    bool permitir_sin_respuesta;    // Crear el sensor aunque no responda (las lecturas se reintentan después)
} mlx90614_config_t;

#define MLX90614_CONFIG_POR_DEFECTO() {             \
    .direccion = MLX90614_DIRECCION_POR_DEFECTO,    \
    .velocidad_hz = MLX90614_VELOCIDAD_MAXIMA_HZ,   \
}

/**
 * Registra el sensor en un bus I2C ya creado y verifica que responda. Con `permitir_sin_respuesta`,
 * si no responde devuelve ESP_ERR_NOT_FOUND pero deja el sensor creado en `sensor`.
 */
esp_err_t mlx90614_crear(i2c_master_bus_handle_t bus, const mlx90614_config_t *config, mlx90614_handle_t *sensor);

/**
 * Quita el sensor del bus y libera su memoria.
 */
esp_err_t mlx90614_eliminar(mlx90614_handle_t sensor);

/**
 * Lee una palabra de 16 bits (RAM o EEPROM) validando el PEC (CRC-8).
 * Devuelve ESP_ERR_INVALID_CRC si la trama llegó corrupta.
 */
esp_err_t mlx90614_leer_registro(mlx90614_handle_t sensor, uint8_t comando, uint16_t *valor);

/**
 * Temperatura del objeto apuntado (zona 1), en °C.
 * Devuelve ESP_ERR_INVALID_STATE si el sensor marca la lectura como inválida
 * y ESP_ERR_INVALID_RESPONSE si no responde en el bus (NACK).
 */
esp_err_t mlx90614_leer_temperatura_objeto(mlx90614_handle_t sensor, float *celsius);

/**
 * Temperatura interna del sensor (ambiente), en °C.
 */
esp_err_t mlx90614_leer_temperatura_ambiente(mlx90614_handle_t sensor, float *celsius);

/**
 * Emisividad configurada en la EEPROM (0.1 a 1.0). Solo lectura.
 */
esp_err_t mlx90614_leer_emisividad(mlx90614_handle_t sensor, float *emisividad);

#ifdef __cplusplus
}
#endif
