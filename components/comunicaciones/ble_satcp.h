#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define BLE_NOMBRE_DISPOSITIVO  "SATCP"

/**
 * Transporte Bluetooth para la app:
 *   Servicio UART de Nordic (6E400001-...): la app escribe JSON en RX y recibe JSON por TX,
 *   cada mensaje terminado en '\n' y partido según el MTU.
 *   Servicio de imagen (8D6F0001-...): la foto en binario, por notificaciones.
 */
esp_err_t ble_satcp_iniciar(void);

bool ble_satcp_conectado(void);

/**
 * Pausa el anuncio mientras haya una app usando el Wi-Fi: la app usa un solo transporte a la vez,
 * y el anuncio le quita tiempo de radio al Wi-Fi. No afecta a una conexión Bluetooth ya abierta.
 */
void ble_satcp_pausar_anuncio(bool pausar);

/**
 * Envía un JSON por la característica TX (si hay una app suscrita).
 */
void ble_satcp_enviar_json(const char *json);

/**
 * Envía la foto (ya cifrada) por la característica de imagen, entre los eventos imagen_inicio e
 * imagen_fin, que salen por `enviar_evento` (cifrados como el resto de los mensajes).
 */
void ble_satcp_enviar_imagen(int32_t id, const uint8_t *datos, size_t bytes, void (*enviar_evento)(const char *json));
