#pragma once

#include "esp_err.h"

/**
 * Transporte Wi-Fi para la app:
 *   ws://<ip>/ws               comandos y eventos JSON cifrados (un mensaje por trama de texto)
 *   GET /foto/ultima.jpg?t=..  última foto tomada (token que da el comando "foto")
 *   GET /info                  dispositivo y versión de protocolo, sin datos sensibles
 */
esp_err_t servidor_web_iniciar(void);

/**
 * Envía un texto a un cliente WebSocket concreto. Se puede llamar desde cualquier tarea.
 */
void servidor_web_enviar(int fd, const char *json);

/**
 * Llena `fds` con los clientes WebSocket conectados y devuelve cuántos hay.
 */
size_t servidor_web_clientes(int *fds, size_t maximo);
