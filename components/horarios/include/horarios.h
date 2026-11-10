#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HORARIOS_MAXIMO         8
#define HORARIOS_ZONA_HORARIA   "CST6"      // Ciudad de México: UTC-6 sin horario de verano
#define HORARIOS_TODOS_LOS_DIAS 0x7F

typedef struct {
    uint8_t hora;       // 0..23
    uint8_t minuto;     // 0..59
    uint8_t dias;       // Máscara: bit 0 = lunes ... bit 6 = domingo
    bool activo;
} horario_t;

/**
 * Fija la zona horaria y carga los horarios guardados en NVS. NVS ya debe estar iniciado.
 */
esp_err_t horarios_iniciar(void);

/**
 * Arranca la sincronización de la hora por SNTP. Se llama al obtener IP en la red de la casa;
 * llamarla de nuevo solo fuerza una sincronización si la hora aún no es válida.
 */
void horarios_sincronizar_hora(void);

/**
 * true si la hora ya se sincronizó por SNTP desde el arranque (el ESP32 no tiene reloj con batería).
 */
bool horarios_hora_valida(void);

/**
 * Copia el horario `indice`. Devuelve false si el lugar está vacío o el índice no existe.
 */
bool horarios_obtener(int indice, horario_t *horario);

/**
 * Guarda un horario en NVS. Con `indice` < 0 usa el primer lugar libre y lo devuelve en
 * `indice_guardado`. Errores: ESP_ERR_INVALID_ARG (parámetros fuera de rango) y
 * ESP_ERR_NO_MEM (no hay lugar libre).
 */
esp_err_t horarios_guardar(int indice, const horario_t *horario, int *indice_guardado);

/**
 * Borra el horario `indice` (borrar un lugar vacío no es error).
 */
esp_err_t horarios_borrar(int indice);

/**
 * Revisa los horarios una vez por minuto: devuelve true y el índice si un horario activo coincide
 * con la hora actual. Sin hora válida nunca coincide, y un mismo horario no se repite en el mismo minuto.
 */
bool horarios_toca_cierre(int *indice);

#ifdef __cplusplus
}
#endif
