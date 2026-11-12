#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "acciones_sistema.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESTADO_SISTEMA_MONITOREO,           // Fase 1: temperatura constante
    ESTADO_SISTEMA_VALIDAR_PRESENCIA,   // Fase 2: ¿alguien atiende la cocina?
    ESTADO_SISTEMA_DETECCION_VISUAL,    // Fase 3: cámara busca fuego o humo
    ESTADO_SISTEMA_ALERTA_USUARIO,      // Fase 3: alerta al celular, espera respuesta
    ESTADO_SISTEMA_CIERRE_AUTOMATICO,   // Fase 4: cierre y reporte
    ESTADO_SISTEMA_BLOQUEADO,           // Tras el cierre: requiere rearme manual
    ESTADO_SISTEMA_AUTOCALIBRACION,     // Temperatura estable: verificar y registrar línea base
} estado_sistema_t;

typedef struct {
    uint32_t confirmacion_presencia_ms;     // Si el radar ve a alguien, tiempo para confirmarlo (> retención de 5 s)
    uint32_t espera_respuesta_s;            // Sin respuesta en este tiempo: cierre automático
    uint32_t intervalo_revalidacion_s;      // Con anomalía activa, volver a verificar cada X s
    uint32_t intervalo_minimo_revision_s;   // Entre inicios de revisión por saltos seguidos (la crítica no espera)
    uint32_t silencio_tras_cancelacion_s;   // Tras cancelar una alerta, no repetirla durante X s
    uint32_t estabilidad_autocalibracion_s; // Tiempo con temperatura estable para tomarla como línea base
    float banda_estabilidad_c;              // Variación máxima (máx - mín) considerada "misma temperatura"
    float temperatura_maxima_linea_base_c;  // Solo la temperatura ambiente (estufa apagada) es línea base
    UBaseType_t prioridad_tarea;
    BaseType_t nucleo_tarea;                // 0: junto a la temperatura y la cámara; el 1 es de las comunicaciones
} maquina_estados_config_t;

#define MAQUINA_ESTADOS_CONFIG_POR_DEFECTO() {  \
    .confirmacion_presencia_ms = 7000,          \
    .espera_respuesta_s = 120,                  \
    .intervalo_revalidacion_s = 30,             \
    .intervalo_minimo_revision_s = 15,          \
    .silencio_tras_cancelacion_s = 300,         \
    .estabilidad_autocalibracion_s = 180,       \
    .banda_estabilidad_c = 1.5f,                \
    .temperatura_maxima_linea_base_c = 24.0f,   \
    .prioridad_tarea = 8,                       \
    .nucleo_tarea = 0,                          \
}

/**
 * Arranca la tarea de la máquina de estados. El monitor de temperatura y la válvula ya deben
 * estar iniciados; la máquina pasa a ser la única consumidora de los eventos del monitor.
 * Si la válvula quedó cerrada antes de reiniciar, arranca en BLOQUEADO.
 */
esp_err_t maquina_estados_iniciar(const maquina_estados_config_t *config);

/**
 * El usuario respondió la alerta desde el celular: cancela el cierre automático.
 */
esp_err_t maquina_estados_notificar_respuesta_usuario(void);

/**
 * Sale del estado BLOQUEADO después de un cierre, una vez revisada la cocina y abierta la válvula
 * a mano. ESP_ERR_NOT_FINISHED si el sensor de posición indica que la válvula sigue cerrada.
 */
esp_err_t maquina_estados_rearmar(void);

/**
 * Cierre pedido por el usuario: desde cualquier fase pasa a CIERRE_AUTOMATICO con origen manual.
 * Devuelve ESP_ERR_INVALID_STATE si ya está BLOQUEADO.
 */
esp_err_t maquina_estados_cerrar_manual(void);

estado_sistema_t maquina_estados_obtener_estado(void);

/**
 * Copia la alerta en curso. Devuelve true si hay una alerta vigente (esperando respuesta,
 * cerrando o bloqueado tras un cierre). `segundos_para_cierre` es 0 fuera de ALERTA_USUARIO.
 */
bool maquina_estados_obtener_alerta(alerta_t *alerta, uint32_t *segundos_para_cierre);

const char *maquina_estados_nombre(estado_sistema_t estado);

#ifdef __cplusplus
}
#endif
