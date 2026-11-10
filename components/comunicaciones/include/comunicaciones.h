#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Arranca los transportes para la app Android: Wi-Fi (WebSocket + HTTP, red propia "SATCP"
 * y red de la casa) y Bluetooth LE. Ambos usan el mismo protocolo JSON (docs/PROTOCOLO_APP.md).
 * Todos los mensajes van cifrados con la clave del equipo, que también es la contraseña de la red SATCP.
 * Requiere NVS iniciada. Si un transporte falla, el otro sigue disponible.
 */
esp_err_t comunicaciones_iniciar(void);

/**
 * Vuelve a la clave de fábrica del equipo (la de la etiqueta) si el usuario olvidó la suya.
 * Cierra todas las sesiones y cambia la contraseña de la red SATCP.
 */
esp_err_t comunicaciones_restaurar_clave_fabrica(void);

#ifdef __cplusplus
}
#endif
