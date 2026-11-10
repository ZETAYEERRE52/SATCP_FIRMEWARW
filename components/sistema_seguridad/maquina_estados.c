#include <inttypes.h>
#include <math.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "acciones_sistema.h"
#include "horarios.h"
#include "maquina_estados.h"
#include "monitor_temperatura.h"
#include "valvula.h"

static const char *TAG = "MAQUINA_ESTADOS";

#define BIT_RESPUESTA_USUARIO       BIT0
#define BIT_REARMAR                 BIT1
#define BIT_CIERRE_MANUAL           BIT2
#define TAM_PILA_TAREA              6144
#define US_POR_S                    1000000LL
#define AVISO_ESPERA_CADA_S         30
#define SIGMAS_BANDA_ESTABILIDAD    4.0f    // La banda nunca menor que 4 veces el ruido estimado

static struct {
    maquina_estados_config_t config;
    volatile estado_sistema_t estado;
    EventGroupHandle_t eventos;
    TaskHandle_t tarea;

    bool anomalia_activa;               // Entre un evento de alarma y su normalización
    bool anomalia_nueva;                // Llegó un evento de alarma aún no atendido
    evento_temperatura_t ultimo_evento; // Última alarma recibida (datos para la alerta)
    int64_t marca_ultima_validacion_us;
    int64_t silencio_hasta_us;
    alerta_t alerta;                    // Alerta en curso (también va en el reporte de cierre)
    portMUX_TYPE cerrojo_alerta;        // La alerta se lee desde las tareas de Wi-Fi y BLE
    volatile uint32_t segundos_para_cierre;

    bool estabilidad_iniciada;
    int64_t inicio_estabilidad_us;
    float minimo_estable_c;
    float maximo_estable_c;
} s_maquina = {
    .cerrojo_alerta = portMUX_INITIALIZER_UNLOCKED,
};

const char *maquina_estados_nombre(estado_sistema_t estado)
{
    switch (estado) {
    case ESTADO_SISTEMA_MONITOREO:          return "MONITOREO";
    case ESTADO_SISTEMA_VALIDAR_PRESENCIA:  return "VALIDAR_PRESENCIA";
    case ESTADO_SISTEMA_DETECCION_VISUAL:   return "DETECCION_VISUAL";
    case ESTADO_SISTEMA_ALERTA_USUARIO:     return "ALERTA_USUARIO";
    case ESTADO_SISTEMA_CIERRE_AUTOMATICO:  return "CIERRE_AUTOMATICO";
    case ESTADO_SISTEMA_BLOQUEADO:          return "BLOQUEADO";
    case ESTADO_SISTEMA_AUTOCALIBRACION:    return "AUTOCALIBRACION";
    }
    return "DESCONOCIDO";
}

static void cambiar_estado(estado_sistema_t nuevo)
{
    ESP_LOGI(TAG, "%s -> %s", maquina_estados_nombre(s_maquina.estado), maquina_estados_nombre(nuevo));
    s_maquina.estado = nuevo;
}

static void reiniciar_estabilidad(void)
{
    s_maquina.estabilidad_iniciada = false;
}

// Recibe los eventos del monitor sin importar el estado actual, para que la cola nunca se llene
static void atender_eventos(TickType_t espera)
{
    evento_temperatura_t evento;
    while (monitor_temperatura_esperar_evento(&evento, espera)) {
        espera = 0;
        switch (evento.tipo) {
        case EVENTO_INCREMENTO_ANOMALO:
        case EVENTO_TEMPERATURA_CRITICA:
        case EVENTO_FALLA_SENSOR:
            s_maquina.anomalia_activa = true;
            s_maquina.anomalia_nueva = true;
            s_maquina.ultimo_evento = evento;
            reiniciar_estabilidad();
            break;
        case EVENTO_TEMPERATURA_NORMALIZADA:
        case EVENTO_SENSOR_RECUPERADO:
            s_maquina.anomalia_activa = false;
            break;
        }
    }
}

// Autocalibración: la temperatura estimada no se sale de la banda durante el tiempo configurado
static bool temperatura_estable(int64_t ahora_us, float *temperatura_estable_c)
{
    const maquina_estados_config_t *cfg = &s_maquina.config;

    float t;
    if (s_maquina.anomalia_activa || !monitor_temperatura_obtener_estimada(&t)
        || t < cfg->temperatura_minima_autocal_c) {
        reiniciar_estabilidad();
        return false;
    }

    // Si la estimación es ruidosa (sensor lejos), la banda se ensancha para no reiniciarse sola
    float banda = cfg->banda_estabilidad_c;
    calibracion_temperatura_t cal;
    if (monitor_temperatura_obtener_calibracion(&cal) && cal.fondo_valido) {
        banda = fmaxf(banda, SIGMAS_BANDA_ESTABILIDAD * cal.ruido_1s_c * cal.ganancia_ruido);
    }

    float linea_base;
    if (monitor_temperatura_obtener_linea_base(&linea_base) && fabsf(t - linea_base) <= banda) {
        // Esta temperatura ya está verificada y registrada
        reiniciar_estabilidad();
        return false;
    }

    if (!s_maquina.estabilidad_iniciada) {
        s_maquina.estabilidad_iniciada = true;
        s_maquina.inicio_estabilidad_us = ahora_us;
        s_maquina.minimo_estable_c = t;
        s_maquina.maximo_estable_c = t;
        return false;
    }

    const float minimo = fminf(s_maquina.minimo_estable_c, t);
    const float maximo = fmaxf(s_maquina.maximo_estable_c, t);
    if (maximo - minimo > banda) {
        // Cambió demasiado: la ventana empieza de nuevo desde aquí
        s_maquina.inicio_estabilidad_us = ahora_us;
        s_maquina.minimo_estable_c = t;
        s_maquina.maximo_estable_c = t;
        return false;
    }
    s_maquina.minimo_estable_c = minimo;
    s_maquina.maximo_estable_c = maximo;

    if (ahora_us - s_maquina.inicio_estabilidad_us < (int64_t)cfg->estabilidad_autocalibracion_s * US_POR_S) {
        return false;
    }
    *temperatura_estable_c = (minimo + maximo) / 2.0f;
    return true;
}

static void preparar_alerta(origen_alerta_t origen, resultado_vision_t vision, float temperatura_c)
{
    const alerta_t alerta = {
        .origen = origen,
        .evento = s_maquina.ultimo_evento.tipo,
        .vision = vision,
        .temperatura_c = temperatura_c,
        .tasa_c_por_min = origen == ORIGEN_ANOMALIA_TEMPERATURA ? s_maquina.ultimo_evento.tasa_c_por_min : 0.0f,
        .marca_us = esp_timer_get_time(),
    };
    taskENTER_CRITICAL(&s_maquina.cerrojo_alerta);
    s_maquina.alerta = alerta;
    taskEXIT_CRITICAL(&s_maquina.cerrojo_alerta);
}

// Cierre sin anomalía (manual u horario): no hay visión ni tasa, se reporta la temperatura del momento
static void preparar_cierre_directo(origen_alerta_t origen)
{
    float temperatura_c;
    if (!monitor_temperatura_obtener_estimada(&temperatura_c)) {
        temperatura_c = NAN;
    }
    preparar_alerta(origen, VISION_SIN_RIESGO, temperatura_c);
    cambiar_estado(ESTADO_SISTEMA_CIERRE_AUTOMATICO);
}

static void ejecutar_monitoreo(void)
{
    atender_eventos(pdMS_TO_TICKS(1000));
    const int64_t ahora = esp_timer_get_time();

    int horario;
    if (horarios_toca_cierre(&horario)) {
        ESP_LOGW(TAG, "Horario programado %d: cerrando el suministro", horario);
        preparar_cierre_directo(ORIGEN_HORARIO);
        return;
    }

    if (s_maquina.anomalia_nueva) {
        const bool critica = s_maquina.ultimo_evento.tipo == EVENTO_TEMPERATURA_CRITICA;
        // Saltos seguidos no encadenan revisiones sin parar: el aviso queda pendiente hasta cumplir el mínimo.
        // Una temperatura crítica no espera
        const bool muy_pronto = !critica && ahora - s_maquina.marca_ultima_validacion_us <
                                (int64_t)s_maquina.config.intervalo_minimo_revision_s * US_POR_S;
        if (!muy_pronto) {
            s_maquina.anomalia_nueva = false;
            if (ahora >= s_maquina.silencio_hasta_us || critica) {
                cambiar_estado(ESTADO_SISTEMA_VALIDAR_PRESENCIA);
                return;
            }
            ESP_LOGI(TAG, "Anomalía durante el silencio tras una cancelación del usuario; se ignora");
        }
    }

    // Una persona presente puede irse con la anomalía aún activa: se vuelve a verificar periódicamente
    if (s_maquina.anomalia_activa && ahora >= s_maquina.silencio_hasta_us &&
        ahora - s_maquina.marca_ultima_validacion_us >= (int64_t)s_maquina.config.intervalo_revalidacion_s * US_POR_S) {
        ESP_LOGI(TAG, "La anomalía sigue activa: revalidando presencia");
        cambiar_estado(ESTADO_SISTEMA_VALIDAR_PRESENCIA);
        return;
    }

    float temperatura_estable_c;
    if (temperatura_estable(ahora, &temperatura_estable_c)) {
        ESP_LOGI(TAG, "Temperatura estable en %.1f °C durante %" PRIu32 " s sin alertas",
                 temperatura_estable_c, s_maquina.config.estabilidad_autocalibracion_s);
        preparar_alerta(ORIGEN_AUTOCALIBRACION, VISION_SIN_RIESGO, temperatura_estable_c);
        cambiar_estado(ESTADO_SISTEMA_AUTOCALIBRACION);
    }
}

static void ejecutar_validar_presencia(void)
{
    s_maquina.marca_ultima_validacion_us = esp_timer_get_time();
    // Desde aquí, si la temperatura sigue subiendo, el monitor vuelve a avisar sin esperar la normalización
    monitor_temperatura_marcar_revision();

    bool hay_presencia = false;
    const esp_err_t ret = accion_detectar_presencia(s_maquina.config.confirmacion_presencia_ms, &hay_presencia);
    atender_eventos(0);
    if (ret != ESP_OK) {
        // Falla segura: sin poder confirmar a nadie, se asume que la cocina está sola
        ESP_LOGW(TAG, "Detector de presencia falló (%s): se asume ausencia", esp_err_to_name(ret));
        hay_presencia = false;
    }

    if (hay_presencia) {
        ESP_LOGI(TAG, "Hay una persona atendiendo la cocina");
        cambiar_estado(ESTADO_SISTEMA_MONITOREO);
    } else {
        ESP_LOGW(TAG, "Nadie atiende la cocina: activando cámara");
        cambiar_estado(ESTADO_SISTEMA_DETECCION_VISUAL);
    }
}

static void ejecutar_deteccion_visual(void)
{
    const resultado_vision_t resultado = accion_detectar_fuego_humo();
    atender_eventos(0);

    if (resultado == VISION_SIN_RIESGO) {
        ESP_LOGI(TAG, "La cámara no detecta fuego ni humo");
        cambiar_estado(ESTADO_SISTEMA_MONITOREO);
        return;
    }
    if (resultado == VISION_ERROR) {
        ESP_LOGW(TAG, "La visión falló: se trata como positivo (falla segura)");
    }
    preparar_alerta(ORIGEN_ANOMALIA_TEMPERATURA, resultado, s_maquina.ultimo_evento.temperatura_c);
    cambiar_estado(ESTADO_SISTEMA_ALERTA_USUARIO);
}

static void ejecutar_alerta_usuario(void)
{
    const maquina_estados_config_t *cfg = &s_maquina.config;

    xEventGroupClearBits(s_maquina.eventos, BIT_RESPUESTA_USUARIO);
    const esp_err_t ret = accion_enviar_alerta(&s_maquina.alerta);
    if (ret != ESP_OK) {
        // Sin forma de avisar, el temporizador sigue: el cierre es la protección final
        ESP_LOGE(TAG, "No se pudo enviar la alerta (%s)", esp_err_to_name(ret));
    }

    ESP_LOGW(TAG, "Esperando respuesta del usuario (%" PRIu32 " s antes del cierre automático)",
             cfg->espera_respuesta_s);
    for (uint32_t restante = cfg->espera_respuesta_s; restante > 0; restante--) {
        s_maquina.segundos_para_cierre = restante;
        const EventBits_t bits = xEventGroupWaitBits(s_maquina.eventos, BIT_RESPUESTA_USUARIO | BIT_CIERRE_MANUAL,
                                                     pdFALSE, pdFALSE, pdMS_TO_TICKS(1000));
        if (bits & BIT_CIERRE_MANUAL) {
            // La tarea principal atiende el cierre manual antes del siguiente estado
            return;
        }
        if (bits & BIT_RESPUESTA_USUARIO) {
            xEventGroupClearBits(s_maquina.eventos, BIT_RESPUESTA_USUARIO);
            ESP_LOGI(TAG, "El usuario respondió: cierre cancelado, silencio de %" PRIu32 " s",
                     cfg->silencio_tras_cancelacion_s);
            s_maquina.silencio_hasta_us = esp_timer_get_time() +
                                          (int64_t)cfg->silencio_tras_cancelacion_s * US_POR_S;
            s_maquina.anomalia_nueva = false;
            cambiar_estado(ESTADO_SISTEMA_MONITOREO);
            return;
        }
        atender_eventos(0);
        if (restante % AVISO_ESPERA_CADA_S == 0) {
            ESP_LOGW(TAG, "  cierre automático en %" PRIu32 " s", restante);
        }
    }

    ESP_LOGE(TAG, "Sin respuesta del usuario");
    cambiar_estado(ESTADO_SISTEMA_CIERRE_AUTOMATICO);
}

static void ejecutar_cierre_automatico(void)
{
    reporte_cierre_t reporte = {
        .alerta = s_maquina.alerta,
        .resultado_cierre = accion_ejecutar_cierre(),
        .marca_cierre_us = esp_timer_get_time(),
    };
    if (reporte.resultado_cierre != ESP_OK) {
        ESP_LOGE(TAG, "¡FALLÓ EL CIERRE AUTOMÁTICO! (%s)", esp_err_to_name(reporte.resultado_cierre));
    }

    const esp_err_t ret = accion_reportar_cierre(&reporte);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo enviar el reporte de cierre (%s)", esp_err_to_name(ret));
    }

    // Un "cerrar" que llegó durante el cierre ya quedó cumplido
    xEventGroupClearBits(s_maquina.eventos, BIT_REARMAR | BIT_CIERRE_MANUAL);
    cambiar_estado(ESTADO_SISTEMA_BLOQUEADO);
}

static void ejecutar_bloqueado(void)
{
    const EventBits_t bits = xEventGroupWaitBits(s_maquina.eventos, BIT_REARMAR, pdTRUE, pdFALSE,
                                                 pdMS_TO_TICKS(1000));
    atender_eventos(0);
    if (bits & BIT_REARMAR) {
        ESP_LOGI(TAG, "Sistema rearmado por el usuario");
        s_maquina.anomalia_nueva = false;
        s_maquina.silencio_hasta_us = 0;
        monitor_temperatura_borrar_linea_base();
        reiniciar_estabilidad();
        cambiar_estado(ESTADO_SISTEMA_MONITOREO);
    }
}

static void ejecutar_autocalibracion(void)
{
    const float temperatura_c = s_maquina.alerta.temperatura_c;
    const resultado_vision_t resultado = accion_detectar_fuego_humo();
    atender_eventos(0);
    reiniciar_estabilidad();

    switch (resultado) {
    case VISION_SIN_RIESGO:
        monitor_temperatura_establecer_linea_base(temperatura_c);
        cambiar_estado(ESTADO_SISTEMA_MONITOREO);
        break;
    case VISION_ERROR:
        // Sin confirmación visual no se acepta la temperatura como segura; se reintentará
        ESP_LOGW(TAG, "Autocalibración sin confirmación visual: no se registra la línea base");
        cambiar_estado(ESTADO_SISTEMA_MONITOREO);
        break;
    case VISION_HUMO:
    case VISION_FUEGO:
        ESP_LOGW(TAG, "La verificación de temperatura estable encontró %s", accion_nombre_vision(resultado));
        taskENTER_CRITICAL(&s_maquina.cerrojo_alerta);
        s_maquina.alerta.vision = resultado;
        taskEXIT_CRITICAL(&s_maquina.cerrojo_alerta);
        cambiar_estado(ESTADO_SISTEMA_ALERTA_USUARIO);
        break;
    }
}

static void tarea_maquina_estados(void *arg)
{
    for (;;) {
        // El cierre manual interrumpe cualquier fase; las acciones largas (radar, cámara) terminan antes
        if ((xEventGroupClearBits(s_maquina.eventos, BIT_CIERRE_MANUAL) & BIT_CIERRE_MANUAL) &&
            s_maquina.estado != ESTADO_SISTEMA_BLOQUEADO && s_maquina.estado != ESTADO_SISTEMA_CIERRE_AUTOMATICO) {
            ESP_LOGW(TAG, "Cierre manual pedido por el usuario");
            preparar_cierre_directo(ORIGEN_MANUAL);
        }
        switch (s_maquina.estado) {
        case ESTADO_SISTEMA_MONITOREO:          ejecutar_monitoreo(); break;
        case ESTADO_SISTEMA_VALIDAR_PRESENCIA:  ejecutar_validar_presencia(); break;
        case ESTADO_SISTEMA_DETECCION_VISUAL:   ejecutar_deteccion_visual(); break;
        case ESTADO_SISTEMA_ALERTA_USUARIO:     ejecutar_alerta_usuario(); break;
        case ESTADO_SISTEMA_CIERRE_AUTOMATICO:  ejecutar_cierre_automatico(); break;
        case ESTADO_SISTEMA_BLOQUEADO:          ejecutar_bloqueado(); break;
        case ESTADO_SISTEMA_AUTOCALIBRACION:    ejecutar_autocalibracion(); break;
        }
    }
}

esp_err_t maquina_estados_iniciar(const maquina_estados_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "configuración nula");
    ESP_RETURN_ON_FALSE(!s_maquina.tarea, ESP_ERR_INVALID_STATE, TAG, "la máquina ya está iniciada");
    ESP_RETURN_ON_FALSE(config->espera_respuesta_s > 0 && config->estabilidad_autocalibracion_s > 0,
                        ESP_ERR_INVALID_ARG, TAG, "tiempos deben ser > 0");

    s_maquina.config = *config;
    s_maquina.estado = ESTADO_SISTEMA_MONITOREO;
    if (valvula_cerrada()) {
        // La válvula quedó cerrada antes del reinicio: hasta el rearme no se vuelve a pulsar
        s_maquina.alerta = (alerta_t){ .origen = ORIGEN_REINICIO, .vision = VISION_SIN_RIESGO,
                                       .temperatura_c = NAN, .marca_us = esp_timer_get_time() };
        s_maquina.estado = ESTADO_SISTEMA_BLOQUEADO;
    }
    s_maquina.eventos = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_maquina.eventos, ESP_ERR_NO_MEM, TAG, "sin memoria para el grupo de eventos");

    if (xTaskCreatePinnedToCore(tarea_maquina_estados, "maquina_estados", TAM_PILA_TAREA, NULL,
                                config->prioridad_tarea, &s_maquina.tarea, config->nucleo_tarea) != pdPASS) {
        vEventGroupDelete(s_maquina.eventos);
        s_maquina.eventos = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Máquina de estados iniciada en %s", maquina_estados_nombre(s_maquina.estado));
    return ESP_OK;
}

esp_err_t maquina_estados_notificar_respuesta_usuario(void)
{
    ESP_RETURN_ON_FALSE(s_maquina.eventos, ESP_ERR_INVALID_STATE, TAG, "la máquina no está iniciada");
    ESP_RETURN_ON_FALSE(s_maquina.estado == ESTADO_SISTEMA_ALERTA_USUARIO, ESP_ERR_INVALID_STATE, TAG,
                        "no hay una alerta esperando respuesta");
    xEventGroupSetBits(s_maquina.eventos, BIT_RESPUESTA_USUARIO);
    return ESP_OK;
}

esp_err_t maquina_estados_rearmar(void)
{
    ESP_RETURN_ON_FALSE(s_maquina.eventos, ESP_ERR_INVALID_STATE, TAG, "la máquina no está iniciada");
    ESP_RETURN_ON_FALSE(s_maquina.estado == ESTADO_SISTEMA_BLOQUEADO, ESP_ERR_INVALID_STATE, TAG,
                        "el sistema no está bloqueado");
    // El rearme confirma que el usuario abrió la válvula a mano (el sistema nunca abre el gas)
    if (valvula_registrar_apertura() != ESP_OK) {
        ESP_LOGW(TAG, "Rearme rechazado: el sensor indica que la válvula sigue cerrada");
        return ESP_ERR_NOT_FINISHED;
    }
    xEventGroupSetBits(s_maquina.eventos, BIT_REARMAR);
    return ESP_OK;
}

esp_err_t maquina_estados_cerrar_manual(void)
{
    ESP_RETURN_ON_FALSE(s_maquina.eventos, ESP_ERR_INVALID_STATE, TAG, "la máquina no está iniciada");
    if (s_maquina.estado == ESTADO_SISTEMA_BLOQUEADO) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_maquina.estado != ESTADO_SISTEMA_CIERRE_AUTOMATICO) {
        xEventGroupSetBits(s_maquina.eventos, BIT_CIERRE_MANUAL);
    }
    return ESP_OK;
}

estado_sistema_t maquina_estados_obtener_estado(void)
{
    return s_maquina.estado;
}

bool maquina_estados_obtener_alerta(alerta_t *alerta, uint32_t *segundos_para_cierre)
{
    const estado_sistema_t estado = s_maquina.estado;
    const bool hay_alerta = estado == ESTADO_SISTEMA_ALERTA_USUARIO || estado == ESTADO_SISTEMA_CIERRE_AUTOMATICO ||
                            estado == ESTADO_SISTEMA_BLOQUEADO;
    if (alerta) {
        taskENTER_CRITICAL(&s_maquina.cerrojo_alerta);
        *alerta = s_maquina.alerta;
        taskEXIT_CRITICAL(&s_maquina.cerrojo_alerta);
    }
    if (segundos_para_cierre) {
        *segundos_para_cierre = estado == ESTADO_SISTEMA_ALERTA_USUARIO ? s_maquina.segundos_para_cierre : 0;
    }
    return hay_alerta;
}
