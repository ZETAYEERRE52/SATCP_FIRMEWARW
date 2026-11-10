#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#define WIFI_AP_SSID    "SATCP"
#define WIFI_AP_IP      "192.168.4.1"

// En la red de la casa la app lo encuentra como satcp.local o por el servicio mDNS _satcp._tcp
#define WIFI_MDNS_NOMBRE    "satcp"
#define WIFI_MDNS_SERVICIO  "_satcp"
#define WIFI_PUERTO         80
#define WIFI_PUERTO_VIVO    81      // Video de la cámara en vivo (servidor aparte)

/**
 * Punto de acceso propio (siempre activo, para usarlo sin router; su contraseña es la clave del
 * equipo, así que la seguridad ya debe estar iniciada) y, si hay una red guardada
 * en NVS, conexión a la red de la casa al mismo tiempo.
 */
esp_err_t wifi_satcp_iniciar(void);

/**
 * Aplica la clave del equipo actual como contraseña de la red SATCP (tras cambiarla o restaurarla).
 */
esp_err_t wifi_satcp_actualizar_clave_ap(void);

/**
 * Guarda la red de la casa en NVS y se conecta a ella.
 */
esp_err_t wifi_satcp_configurar_red(const char *ssid, const char *clave);

/**
 * IP obtenida en la red de la casa. Devuelve false si no está conectado.
 */
bool wifi_satcp_ip(char *ip, size_t tam);
