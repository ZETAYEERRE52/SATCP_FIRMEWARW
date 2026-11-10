#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LD2410_BAUDIOS_POR_DEFECTO  256000

// Estado del objetivo tal como lo reporta el radar (bits: 1 = movimiento, 2 = estático)
typedef enum {
    LD2410_SIN_OBJETIVO = 0,
    LD2410_EN_MOVIMIENTO = 1,
    LD2410_ESTATICO = 2,
    LD2410_MOVIMIENTO_Y_ESTATICO = 3,
} ld2410_objetivo_t;

typedef struct {
    ld2410_objetivo_t objetivo;
    uint16_t distancia_movimiento_cm;
    uint8_t energia_movimiento;         // 0-100
    uint16_t distancia_estatico_cm;
    uint8_t energia_estatico;           // 0-100
    uint16_t distancia_deteccion_cm;
    int64_t marca_us;                   // Momento de la trama
} ld2410_lectura_t;

typedef struct {
    uart_port_t puerto;
    gpio_num_t pin_tx;          // ESP32 TX -> RX del radar
    gpio_num_t pin_rx;          // ESP32 RX <- TX del radar
    gpio_num_t pin_out;         // Salida digital de presencia (GPIO_NUM_NC si no se conecta)
    uint32_t baudios;
} ld2410_config_t;

/**
 * Configura la UART y arranca la tarea que decodifica las tramas del radar.
 */
esp_err_t ld2410_iniciar(const ld2410_config_t *config);

/**
 * Última lectura recibida. Devuelve false si no llegan tramas desde hace más de 1 s.
 */
bool ld2410_obtener_lectura(ld2410_lectura_t *lectura);

/**
 * Diagnóstico: bytes crudos recibidos por la UART y tramas decodificadas desde el arranque.
 * Bytes sin tramas = velocidad incorrecta; ningún byte = cableado o alimentación.
 */
void ld2410_obtener_contadores(uint32_t *bytes_recibidos, uint32_t *tramas_validas);

#define LD2410_TAM_MUESTRA  16

typedef struct {
    uint32_t bytes_recibidos;
    uint32_t cabeceras;         // Inicios de trama F4 F3 F2 F1 encontrados
    uint32_t tramas_validas;
    uint32_t desbordes;         // Datos perdidos porque el firmware no los leyó a tiempo
    uint32_t cortes;            // La línea RX se quedó en bajo más de un carácter
    char muestra_hex[3 * LD2410_TAM_MUESTRA + 1];   // Últimos bytes recibidos
} ld2410_diagnostico_t;

/**
 * Diagnóstico detallado de la UART, para cuando el radar no entrega tramas válidas.
 */
void ld2410_obtener_diagnostico(ld2410_diagnostico_t *diagnostico);

/**
 * Nivel del pin OUT (1 = presencia). Devuelve -1 si no está configurado.
 */
int ld2410_leer_salida(void);

#ifdef __cplusplus
}
#endif
