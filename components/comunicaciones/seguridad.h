#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "comunicaciones_interno.h"

/**
 * Seguridad del protocolo 2 (detalle en docs/PROTOCOLO_APP.md, §1b):
 *   - Una clave del equipo (8-63 caracteres): es la contraseña de la red SATCP y la base de cifrado.
 *     Clave maestra = PBKDF2-HMAC-SHA256(clave, SEGURIDAD_SAL, SEGURIDAD_ITERACIONES, 32 bytes).
 *   - Cada conexión abre una sesión con "hola": el ESP32 elige un reto aleatorio y
 *     clave de sesión = HMAC-SHA256(clave maestra, "SATCP-sesion" || reto).
 *   - Todo lo demás viaja como {"c":"<base64(nonce 12 || cifrado || etiqueta 16)>"} en AES-256-GCM
 *     con un contador "n" creciente por sentido: un mensaje capturado no se puede repetir.
 */

#define SEGURIDAD_SAL           "SATCP-clave-v1"
#define SEGURIDAD_ITERACIONES   10000
#define SEGURIDAD_CLAVE_MIN     8       // Límites de una contraseña WPA2
#define SEGURIDAD_CLAVE_MAX     63
#define SEGURIDAD_TAM_TOKEN     33      // 32 caracteres hexadecimales y el terminador

typedef enum {
    SEGURIDAD_OK,
    SEGURIDAD_SIN_SESION,       // El cliente no envió "hola" (o la sesión se cerró)
    SEGURIDAD_NO_AUTORIZADO,    // No descifra: clave equivocada o mensaje alterado
    SEGURIDAD_REPETIDO,         // Contador "n" no creciente: mensaje repetido
    SEGURIDAD_FORMATO,          // No es un sobre válido
} resultado_seguridad_t;

typedef enum {
    TOKEN_FOTO,     // GET /foto/ultima.jpg?t=...
    TOKEN_VIVO,     // GET :81/vivo?t=...
    NUM_TOKENS,
} uso_token_t;

/**
 * Carga la clave del equipo de NVS o, la primera vez, genera una aleatoria (queda también como
 * clave de fábrica para la etiqueta) y deriva la clave maestra. NVS ya debe estar iniciado.
 */
esp_err_t seguridad_iniciar(void);

/**
 * Clave actual del equipo (contraseña de la red SATCP).
 */
const char *seguridad_clave_equipo(void);

/**
 * Abre (o renueva) la sesión del cliente y devuelve la respuesta "hola" en claro (liberar con free()).
 */
char *seguridad_hola(transporte_t transporte, int fd, const char *id_json);

/**
 * Descifra un sobre {"c":...} y comprueba el contador. Deja el JSON en claro en `salida`.
 */
resultado_seguridad_t seguridad_descifrar(transporte_t transporte, int fd, const char *base64,
                                          char *salida, size_t tam_salida);

/**
 * Cifra un JSON (objeto) para el cliente: devuelve el sobre (liberar con free()) o NULL si no tiene sesión.
 */
char *seguridad_cifrar(transporte_t transporte, int fd, const char *json);

/**
 * Cifra datos binarios (la foto por Bluetooth). Devuelve nonce || cifrado || etiqueta en PSRAM
 * (liberar con free()) o NULL si no hay sesión.
 */
uint8_t *seguridad_cifrar_binario(transporte_t transporte, int fd, const uint8_t *datos, size_t bytes,
                                  size_t *bytes_cifrados);

bool seguridad_tiene_sesion(transporte_t transporte, int fd);

size_t seguridad_sesiones_activas(transporte_t transporte);

void seguridad_cerrar(transporte_t transporte, int fd);

void seguridad_cerrar_todas(void);

/**
 * Cambia la clave del equipo (guarda en NVS y deriva la nueva clave maestra). Las sesiones abiertas
 * siguen hasta que se llame a seguridad_cerrar_todas(). ESP_ERR_INVALID_ARG si no es válida.
 */
esp_err_t seguridad_cambiar_clave(const char *nueva);

/**
 * Vuelve a la clave de fábrica (la de la etiqueta). Cierra todas las sesiones.
 */
esp_err_t seguridad_restaurar_clave_fabrica(void);

/**
 * Token de un solo propósito y corta duración para las rutas HTTP, que no van cifradas.
 */
void seguridad_nuevo_token(uso_token_t uso, char token[SEGURIDAD_TAM_TOKEN]);
bool seguridad_validar_token(uso_token_t uso, const char *token);
