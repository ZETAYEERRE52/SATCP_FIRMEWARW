// Fase 2: detección de presencia con el radar HLK-LD2410B

#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "acciones_sistema.h"
#include "ld2410.h"

static const char *TAG = "PRESENCIA";

#define PERIODO_MUESTREO_MS         100
#define INTENTOS_LECTURA_INICIAL    3
#define DISTANCIA_MAXIMA_CM         200     // El radar está a ~1 m de la estufa: más lejos no atiende la cocina
#define RETENCION_RADAR_MS          5000    // El radar sigue reportando presencia ~5 s tras irse la persona
#define FRACCION_CONFIRMACION       0.5f    // Tras la retención, la persona debe aparecer en la mitad de las muestras

static bool objetivo_cercano(const ld2410_lectura_t *lectura)
{
    const bool en_movimiento = (lectura->objetivo & LD2410_EN_MOVIMIENTO) &&
                               lectura->distancia_movimiento_cm <= DISTANCIA_MAXIMA_CM;
    const bool estatico = (lectura->objetivo & LD2410_ESTATICO) &&
                          lectura->distancia_estatico_cm <= DISTANCIA_MAXIMA_CM;
    return en_movimiento || estatico;
}

// Una muestra de presencia. Devuelve false si el radar no entrega datos
static bool muestrear_presencia(bool *hay_persona)
{
    ld2410_lectura_t lectura;
    if (ld2410_obtener_lectura(&lectura)) {
        *hay_persona = objetivo_cercano(&lectura);
        return true;
    }
    // Sin tramas por UART: el pin OUT da presencia sí/no, aunque sin filtro de distancia
    const int salida = ld2410_leer_salida();
    if (salida < 0) {
        return false;
    }
    *hay_persona = salida != 0;
    return true;
}

esp_err_t accion_detectar_presencia(uint32_t confirmacion_ms, bool *hay_presencia)
{
    // 1) Lectura inmediata: si no hay nadie, no se pierde tiempo
    bool hay_persona = false;
    bool hay_dato = false;
    for (int intento = 0; intento < INTENTOS_LECTURA_INICIAL && !hay_dato; intento++) {
        hay_dato = muestrear_presencia(&hay_persona);
        if (!hay_dato) {
            vTaskDelay(pdMS_TO_TICKS(PERIODO_MUESTREO_MS));
        }
    }
    if (!hay_dato) {
        ESP_LOGE(TAG, "El radar no entrega datos");
        return ESP_ERR_TIMEOUT;
    }
    if (!hay_persona) {
        ld2410_lectura_t lectura;
        if (ld2410_obtener_lectura(&lectura)) {
            ESP_LOGW(TAG, "Nadie frente a la estufa (radar: estado %d, mov. %u cm, est. %u cm, máx. %d cm)",
                     lectura.objetivo, lectura.distancia_movimiento_cm, lectura.distancia_estatico_cm,
                     DISTANCIA_MAXIMA_CM);
        } else {
            ESP_LOGW(TAG, "Nadie frente a la estufa (sin tramas UART; pin OUT = %d)", ld2410_leer_salida());
        }
        *hay_presencia = false;
        return ESP_OK;
    }

    // 2) Confirmación: lo detectado al inicio puede ser solo la retención de alguien que acaba de irse,
    //    así que solo cuentan las muestras posteriores a esa retención
    uint32_t descarte_ms = RETENCION_RADAR_MS;
    if (confirmacion_ms <= descarte_ms) {
        ESP_LOGW(TAG, "Confirmación de %" PRIu32 " ms no supera la retención del radar: se evalúa completa",
                 confirmacion_ms);
        descarte_ms = 0;
    }
    ESP_LOGI(TAG, "Persona detectada: confirmando durante %" PRIu32 " ms...", confirmacion_ms);

    uint32_t muestras = 0;
    uint32_t positivas = 0;
    for (uint32_t transcurrido = 0; transcurrido < confirmacion_ms; transcurrido += PERIODO_MUESTREO_MS) {
        vTaskDelay(pdMS_TO_TICKS(PERIODO_MUESTREO_MS));
        if (transcurrido < descarte_ms || !muestrear_presencia(&hay_persona)) {
            continue;
        }
        muestras++;
        positivas += hay_persona ? 1 : 0;
    }

    if (muestras == 0) {
        ESP_LOGE(TAG, "El radar dejó de entregar datos durante la confirmación");
        return ESP_ERR_TIMEOUT;
    }

    const float fraccion = (float)positivas / (float)muestras;
    *hay_presencia = fraccion >= FRACCION_CONFIRMACION;
    ESP_LOGI(TAG, "Confirmación: persona en %.0f %% del tramo final -> %s",
             fraccion * 100.0f, *hay_presencia ? "HAY PERSONA" : "SE FUE (era retención del radar)");
    return ESP_OK;
}
