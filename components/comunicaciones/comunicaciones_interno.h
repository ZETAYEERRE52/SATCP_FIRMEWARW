#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef enum {
    TRANSPORTE_BLE,
    TRANSPORTE_WIFI,
} transporte_t;

// Mayor mensaje aceptado: el sobre cifrado (base64) del comando más largo, el de Wi-Fi con red y clave largas
#define TAM_MAX_MENSAJE     768

// Todo lo de comunicaciones va en el núcleo 1, junto con la radio Wi-Fi/Bluetooth (sdkconfig.defaults);
// el núcleo 0 queda libre para el sistema de seguridad
#define NUCLEO_COMUNICACIONES   1

/**
 * Encola un comando JSON recibido por un transporte. `fd` identifica al cliente WebSocket.
 * Lo llaman los transportes desde sus propias tareas; se procesa en la tarea de comunicaciones.
 */
bool comunicaciones_recibir(const char *mensaje, size_t largo, transporte_t transporte, int fd);

/**
 * Pide enviar un evento de estado en el próximo ciclo (por ejemplo, al conectarse una app).
 */
void comunicaciones_forzar_estado(void);

/**
 * El cliente cerró la conexión: su sesión cifrada deja de valer.
 */
void comunicaciones_cliente_desconectado(transporte_t transporte, int fd);
