// Alerta, cierre y reporte. El cierre acciona la válvula real (valvula.c); el reporte llega a la
// app por el evento de estado. Presencia y visión están en acciones_presencia.c y acciones_vision.c.

#include <inttypes.h>
#include "esp_log.h"
#include "acciones_sistema.h"
#include "valvula.h"

static const char *TAG = "ACCIONES";

const char *accion_nombre_origen(origen_alerta_t origen)
{
    switch (origen) {
    case ORIGEN_ANOMALIA_TEMPERATURA:   return "temperatura";
    case ORIGEN_AUTOCALIBRACION:        return "autocalibracion";
    case ORIGEN_MANUAL:                 return "manual";
    case ORIGEN_HORARIO:                return "horario";
    case ORIGEN_REINICIO:               return "reinicio";
    }
    return "desconocido";
}

// El teléfono recibe la alerta consultando la máquina de estados: la página web la muestra
// (con vibración y sonido) y el Bluetooth la notifica cada 15 s hasta que se responde
esp_err_t accion_enviar_alerta(const alerta_t *alerta)
{
    ESP_LOGW(TAG, "ALERTA publicada para Wi-Fi y Bluetooth: %s, %.1f °C, %.1f °C/min, origen %s",
             accion_nombre_vision(alerta->vision), alerta->temperatura_c, alerta->tasa_c_por_min,
             accion_nombre_origen(alerta->origen));
    ESP_LOGW(TAG, "Para responder: botón en la página web, comando BLE 'responder' o botón BOOT");
    return ESP_OK;
}

esp_err_t accion_ejecutar_cierre(void)
{
    ESP_LOGE(TAG, "CIERRE AUTOMÁTICO: cerrando la válvula de gas");
    return valvula_cerrar();
}

esp_err_t accion_reportar_cierre(const reporte_cierre_t *reporte)
{
    ESP_LOGW(TAG, "[SIMULADO] Reporte de cierre (origen %s): %s a %.1f °C, alerta en t=%" PRId64
             " s, cierre en t=%" PRId64 " s, resultado %s",
             accion_nombre_origen(reporte->alerta.origen),
             accion_nombre_vision(reporte->alerta.vision), reporte->alerta.temperatura_c,
             reporte->alerta.marca_us / 1000000, reporte->marca_cierre_us / 1000000,
             esp_err_to_name(reporte->resultado_cierre));
    return ESP_OK;
}
