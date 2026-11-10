#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "monitor_temperatura.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Acciones que la máquina de estados delega en el hardware. Por ahora están simuladas
 * en acciones_simuladas.c; cada etapa real (sensor de presencia, visión, Wi-Fi/BLE,
 * válvula) reemplaza su función sin tocar la máquina de estados.
 */

typedef enum {
    VISION_SIN_RIESGO,
    VISION_HUMO,
    VISION_FUEGO,
    VISION_ERROR,           // La máquina lo trata como positivo (falla segura)
} resultado_vision_t;

typedef enum {
    ORIGEN_ANOMALIA_TEMPERATURA,
    ORIGEN_AUTOCALIBRACION,     // Fuego/humo hallado al verificar una temperatura estable
    ORIGEN_MANUAL,              // El usuario pidió el cierre desde la app (comando "cerrar")
    ORIGEN_HORARIO,             // Cierre por un horario programado
    ORIGEN_REINICIO,            // Arrancó con la válvula cerrada de antes: sigue bloqueado hasta el rearme
} origen_alerta_t;

typedef struct {
    origen_alerta_t origen;
    tipo_evento_temperatura_t evento;   // Válido si origen == ORIGEN_ANOMALIA_TEMPERATURA
    resultado_vision_t vision;
    float temperatura_c;
    float tasa_c_por_min;
    int64_t marca_us;
} alerta_t;

typedef struct {
    alerta_t alerta;
    int64_t marca_cierre_us;
    esp_err_t resultado_cierre;
} reporte_cierre_t;

/**
 * Fase 2: ¿hay una persona atendiendo la cocina? Responde de inmediato si no ve a nadie;
 * si ve a alguien, lo confirma durante `confirmacion_ms`.
 */
esp_err_t accion_detectar_presencia(uint32_t confirmacion_ms, bool *hay_presencia);

/**
 * Fase 3: captura y clasifica imágenes de la estufa.
 */
resultado_vision_t accion_detectar_fuego_humo(void);

/**
 * Fase 3: notifica al celular del usuario (Wi-Fi y Bluetooth). La respuesta llega de forma
 * asíncrona mediante maquina_estados_notificar_respuesta_usuario().
 */
esp_err_t accion_enviar_alerta(const alerta_t *alerta);

/**
 * Fase 4: cierra la válvula de gas (optoacoplador sobre su botón, ver valvula.h).
 */
esp_err_t accion_ejecutar_cierre(void);

/**
 * Fase 4: envía al usuario los datos del cierre.
 */
esp_err_t accion_reportar_cierre(const reporte_cierre_t *reporte);

const char *accion_nombre_vision(resultado_vision_t resultado);

/**
 * Nombre del origen tal como viaja en el protocolo: temperatura, autocalibracion, manual u horario.
 */
const char *accion_nombre_origen(origen_alerta_t origen);

#ifdef __cplusplus
}
#endif
