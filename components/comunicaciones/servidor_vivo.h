#pragma once

#include "esp_err.h"

/**
 * Vista en vivo de la cámara para ajustar la posición del equipo:
 *   GET http://<ip>:81/vivo   video MJPEG (multipart/x-mixed-replace), ~5 cuadros por segundo
 * Va en un servidor aparte porque la transmisión ocupa su tarea mientras dura, y así no
 * bloquea el WebSocket de comandos y alertas. Un solo espectador a la vez, máximo 10 min.
 */
esp_err_t servidor_vivo_iniciar(void);
