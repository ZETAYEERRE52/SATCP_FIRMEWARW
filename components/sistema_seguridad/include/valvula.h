#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Válvula de gas accionada con un optoacoplador que simula la pulsación de su botón.
 * El botón ALTERNA (abre/cierra), así que nunca se pulsa sin saber el estado: con la válvula
 * ya cerrada, un pulso la abriría. El estado sale del sensor de posición si existe; si no, se
 * supone (guardado en NVS) y solo cambia por un cierre del sistema o por un rearme.
 */
typedef struct {
    gpio_num_t pin_pulso;           // Ánodo del LED del optoacoplador (con su resistencia); activo en alto
    uint32_t duracion_pulso_ms;     // Cuánto se "mantiene presionado" el botón
    gpio_num_t pin_posicion;        // Final de carrera o sensor de posición; GPIO_NUM_NC si no hay
    int nivel_cerrada;              // Nivel que lee pin_posicion con la válvula cerrada
} valvula_config_t;

esp_err_t valvula_iniciar(const valvula_config_t *config);

/**
 * Cierra la válvula: pulsa solo si está abierta. Con sensor comprueba que quedó cerrada.
 * ESP_FAIL si el sensor sigue indicando abierta tras el pulso.
 */
esp_err_t valvula_cerrar(void);

/**
 * true si la válvula está (o se supone) cerrada.
 */
bool valvula_cerrada(void);

/**
 * true si hay sensor de posición y el estado es medido, no supuesto.
 */
bool valvula_tiene_sensor(void);

/**
 * Rearme: el usuario ya abrió la válvula a mano. Sin sensor se pasa a suponer "abierta";
 * con sensor devuelve ESP_ERR_INVALID_STATE si sigue cerrada.
 */
esp_err_t valvula_registrar_apertura(void);

#ifdef __cplusplus
}
#endif
