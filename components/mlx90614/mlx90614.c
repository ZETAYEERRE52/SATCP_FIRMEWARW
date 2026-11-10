#include <stdlib.h>
#include "esp_check.h"
#include "esp_log.h"
#include "mlx90614.h"

static const char *TAG = "MLX90614";

#define MLX90614_TIMEOUT_MS         20
#define MLX90614_REINTENTOS         3
#define MLX90614_BANDERA_ERROR      0x8000      // Bit 15 en RAM = lectura inválida
#define MLX90614_KELVIN_POR_LSB     0.02f
#define KELVIN_A_CELSIUS            273.15f

struct mlx90614_t {
    i2c_master_dev_handle_t dispositivo;
    uint8_t direccion;
};

// CRC-8 SMBus (polinomio x^8 + x^2 + x + 1, valor inicial 0)
static uint8_t calcular_crc8(const uint8_t *datos, size_t longitud)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < longitud; i++) {
        crc ^= datos[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static esp_err_t leer_temperatura(mlx90614_handle_t sensor, uint8_t comando, float *celsius)
{
    ESP_RETURN_ON_FALSE(celsius, ESP_ERR_INVALID_ARG, TAG, "puntero nulo");

    uint16_t crudo;
    esp_err_t ret = mlx90614_leer_registro(sensor, comando, &crudo);
    if (ret != ESP_OK) {
        return ret;
    }
    if (crudo & MLX90614_BANDERA_ERROR) {
        // No usar ESP_ERR_INVALID_RESPONSE: el driver I2C ya lo devuelve para NACK
        return ESP_ERR_INVALID_STATE;
    }
    *celsius = (float)crudo * MLX90614_KELVIN_POR_LSB - KELVIN_A_CELSIUS;
    return ESP_OK;
}

esp_err_t mlx90614_crear(i2c_master_bus_handle_t bus, const mlx90614_config_t *config, mlx90614_handle_t *sensor)
{
    ESP_RETURN_ON_FALSE(bus && config && sensor, ESP_ERR_INVALID_ARG, TAG, "argumentos inválidos");
    ESP_RETURN_ON_FALSE(config->velocidad_hz > 0 && config->velocidad_hz <= MLX90614_VELOCIDAD_MAXIMA_HZ,
                        ESP_ERR_INVALID_ARG, TAG, "velocidad I2C fuera de rango (máx. 100 kHz)");

    struct mlx90614_t *nuevo = calloc(1, sizeof(*nuevo));
    ESP_RETURN_ON_FALSE(nuevo, ESP_ERR_NO_MEM, TAG, "sin memoria");
    nuevo->direccion = config->direccion;

    const i2c_device_config_t config_dispositivo = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->direccion,
        .scl_speed_hz = config->velocidad_hz,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &config_dispositivo, &nuevo->dispositivo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo agregar el dispositivo al bus (%s)", esp_err_to_name(ret));
        free(nuevo);
        return ret;
    }

    // Una lectura completa con PEC confirma que el sensor está presente y la línea es estable
    float ambiente;
    ret = mlx90614_leer_temperatura_ambiente(nuevo, &ambiente);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "El sensor no responde en 0x%02X (%s)", config->direccion, esp_err_to_name(ret));
        if (config->permitir_sin_respuesta) {
            *sensor = nuevo;
            return ESP_ERR_NOT_FOUND;
        }
        i2c_master_bus_rm_device(nuevo->dispositivo);
        free(nuevo);
        return ret;
    }

    ESP_LOGI(TAG, "Sensor detectado en 0x%02X, temperatura ambiente %.2f °C", config->direccion, ambiente);
    *sensor = nuevo;
    return ESP_OK;
}

esp_err_t mlx90614_eliminar(mlx90614_handle_t sensor)
{
    ESP_RETURN_ON_FALSE(sensor, ESP_ERR_INVALID_ARG, TAG, "sensor nulo");
    esp_err_t ret = i2c_master_bus_rm_device(sensor->dispositivo);
    free(sensor);
    return ret;
}

esp_err_t mlx90614_leer_registro(mlx90614_handle_t sensor, uint8_t comando, uint16_t *valor)
{
    ESP_RETURN_ON_FALSE(sensor && valor, ESP_ERR_INVALID_ARG, TAG, "argumentos inválidos");

    esp_err_t ret = ESP_FAIL;
    for (int intento = 0; intento < MLX90614_REINTENTOS; intento++) {
        // Read Word SMBus: [comando] + repeated start + [LSB, MSB, PEC]
        uint8_t rx[3];
        ret = i2c_master_transmit_receive(sensor->dispositivo, &comando, 1, rx, sizeof(rx), MLX90614_TIMEOUT_MS);
        if (ret != ESP_OK) {
            continue;
        }

        // El PEC cubre toda la transacción, incluidas las direcciones de escritura y lectura
        const uint8_t trama[5] = {
            (uint8_t)(sensor->direccion << 1),
            comando,
            (uint8_t)((sensor->direccion << 1) | 1),
            rx[0],
            rx[1],
        };
        if (calcular_crc8(trama, sizeof(trama)) != rx[2]) {
            ret = ESP_ERR_INVALID_CRC;
            continue;
        }

        *valor = (uint16_t)((rx[1] << 8) | rx[0]);
        return ESP_OK;
    }
    return ret;
}

esp_err_t mlx90614_leer_temperatura_objeto(mlx90614_handle_t sensor, float *celsius)
{
    return leer_temperatura(sensor, MLX90614_RAM_TOBJ1, celsius);
}

esp_err_t mlx90614_leer_temperatura_ambiente(mlx90614_handle_t sensor, float *celsius)
{
    return leer_temperatura(sensor, MLX90614_RAM_TA, celsius);
}

esp_err_t mlx90614_leer_emisividad(mlx90614_handle_t sensor, float *emisividad)
{
    ESP_RETURN_ON_FALSE(emisividad, ESP_ERR_INVALID_ARG, TAG, "puntero nulo");

    uint16_t crudo;
    esp_err_t ret = mlx90614_leer_registro(sensor, MLX90614_EEPROM_EMISIVIDAD, &crudo);
    if (ret == ESP_OK) {
        *emisividad = (float)crudo / 65535.0f;
    }
    return ret;
}
