#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "comunicaciones_interno.h"

// Protocolo descrito en docs/PROTOCOLO_APP.md: subir la versión al cambiar el formato de los mensajes
#define SATCP_VERSION_FIRMWARE      "2.0.0"
#define SATCP_VERSION_PROTOCOLO     2     // 2: mensajes cifrados (sesión con "hola")

typedef struct {
    uint8_t *datos;     // JPEG a enviar por Bluetooth (se libera con free()); NULL si no hay
    size_t bytes;
    int32_t id;         // "id" del comando que la pidió
} imagen_pendiente_t;

esp_err_t protocolo_iniciar(void);

/**
 * Procesa un comando JSON ya descifrado y devuelve la respuesta JSON en claro (liberar con free()).
 * Si el comando pide la foto por Bluetooth, la deja en `imagen` para enviarla aparte.
 * `clave_cambiada` queda en true tras "cambiar_clave": hay que cerrar las sesiones y cambiar la red SATCP.
 */
char *protocolo_procesar(const char *mensaje, transporte_t transporte, imagen_pendiente_t *imagen,
                         bool *clave_cambiada);

/**
 * Evento de estado JSON (liberar con free()).
 */
char *protocolo_evento_estado(void);

/**
 * Información pública del dispositivo para GET /info: nombre, firmware y protocolo (liberar con free()).
 */
char *protocolo_info(void);

/**
 * Acceso a la última foto tomada, para descargarla por HTTP. Devuelve NULL si no hay ninguna.
 * Toda llamada que devuelva datos debe cerrarse con protocolo_soltar_ultima_foto().
 */
const uint8_t *protocolo_tomar_ultima_foto(size_t *bytes);
void protocolo_soltar_ultima_foto(void);
