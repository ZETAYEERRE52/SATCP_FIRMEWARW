// Fase 3: detección visual con la cámara OV3660 y el clasificador fuego / humo / normal

#include <math.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "acciones_sistema.h"
#include "almacenamiento_sd.h"
#include "camara_vision.h"
#include "clasificador.h"

static const char *TAG = "VISION";

#define CUADROS_POR_ANALISIS    5
#define PERIODO_CUADROS_MS      600     // ~3 s en total: capta el parpadeo de la llama y el avance del humo
#define ESPERA_CAMARA_MS        3000
#define CUADROS_MINIMOS         3       // Con menos cuadros clasificados se repite el análisis una vez

const char *accion_nombre_vision(resultado_vision_t resultado)
{
    switch (resultado) {
    case VISION_SIN_RIESGO: return "sin riesgo";
    case VISION_HUMO:       return "HUMO";
    case VISION_FUEGO:      return "FUEGO";
    case VISION_ERROR:      return "error de visión";
    }
    return "desconocido";
}

// Toda foto de la fase visual se guarda con lo que el modelo vio: son las más valiosas para reentrenar
static void guardar_evidencia(const camera_fb_t *cuadro, const resultado_clasificacion_t *resultado)
{
    if (!sd_disponible()) {
        return;
    }
    char etiqueta[24];
    if (resultado) {
        // Ej. fuego_92_000012.jpg: clase predicha y su probabilidad en %
        snprintf(etiqueta, sizeof(etiqueta), "%s_%d", clasificador_nombre_clase(resultado->clase),
                 (int)(resultado->probabilidad[resultado->clase] * 100 + 0.5f));
    } else {
        snprintf(etiqueta, sizeof(etiqueta), "sinclasificar");
    }
    char ruta[128];
    if (sd_guardar_imagen(SD_CARPETA_EVIDENCIA, etiqueta, cuadro->buf, cuadro->len, ruta, sizeof(ruta)) == ESP_OK) {
        ESP_LOGI(TAG, "  evidencia: %s", ruta);
    }
}

// Voto de un cuadro: fuego o humo solo con VISION_PROBABILIDAD_MINIMA; con menos cuenta como normal
static clase_vision_t votar_cuadro(const resultado_clasificacion_t *resultado)
{
    if (resultado->probabilidad[CLASE_FUEGO] >= VISION_PROBABILIDAD_MINIMA) {
        return CLASE_FUEGO;
    }
    if (resultado->probabilidad[CLASE_HUMO] >= VISION_PROBABILIDAD_MINIMA) {
        return CLASE_HUMO;
    }
    return CLASE_NORMAL;
}

// Una pasada de CUADROS_POR_ANALISIS cuadros; acumula los votos en `detalle`
static void analizar_cuadros(int pasada, detalle_vision_t *detalle)
{
    for (int i = 0; i < CUADROS_POR_ANALISIS; i++) {
        if (i > 0) {
            vTaskDelay(pdMS_TO_TICKS(PERIODO_CUADROS_MS));
        }
        camera_fb_t *cuadro = camara_capturar(pdMS_TO_TICKS(ESPERA_CAMARA_MS));
        if (!cuadro) {
            ESP_LOGW(TAG, "Cuadro %d.%d: la cámara no respondió a tiempo", pasada, i + 1);
            continue;
        }

        resultado_clasificacion_t resultado;
        const bool ok = clasificador_disponible() &&
                        clasificador_clasificar_jpeg(cuadro->buf, cuadro->len, &resultado) == ESP_OK;
        if (ok) {
            const clase_vision_t voto = votar_cuadro(&resultado);
            detalle->clasificados++;
            detalle->votos_fuego += voto == CLASE_FUEGO;
            detalle->votos_humo += voto == CLASE_HUMO;
            detalle->votos_normal += voto == CLASE_NORMAL;
            detalle->probabilidad_max = fmaxf(detalle->probabilidad_max,
                                              fmaxf(resultado.probabilidad[CLASE_FUEGO],
                                                    resultado.probabilidad[CLASE_HUMO]));
            ESP_LOGI(TAG, "Cuadro %d.%d: fuego %.3f | humo %.3f | normal %.3f -> voto %s (%lu ms)", pasada, i + 1,
                     resultado.probabilidad[CLASE_FUEGO], resultado.probabilidad[CLASE_HUMO],
                     resultado.probabilidad[CLASE_NORMAL], clasificador_nombre_clase(voto),
                     (unsigned long)resultado.tiempo_ms);
            resultado.clase = voto;     // La evidencia se nombra con el voto
        }
        guardar_evidencia(cuadro, ok ? &resultado : NULL);
        camara_liberar(cuadro);
    }
}

resultado_vision_t accion_detectar_fuego_humo(detalle_vision_t *detalle_salida)
{
    detalle_vision_t detalle = { .valido = true };
    if (detalle_salida) {
        *detalle_salida = detalle;
    }
    if (!camara_disponible()) {
        ESP_LOGE(TAG, "Cámara no disponible");
        return VISION_ERROR;
    }
    if (!clasificador_disponible()) {
        ESP_LOGE(TAG, "Clasificador no disponible");
    }

    // El video en vivo deja de capturar mientras dure el análisis: la vigilancia no espera a la cámara
    camara_prioridad_vision(true);
    analizar_cuadros(1, &detalle);
    if (detalle.clasificados < CUADROS_MINIMOS) {
        ESP_LOGW(TAG, "Solo %u cuadros clasificados: se repite el análisis", detalle.clasificados);
        analizar_cuadros(2, &detalle);
    }
    camara_prioridad_vision(false);
    if (detalle_salida) {
        *detalle_salida = detalle;
    }

    if (detalle.clasificados < CUADROS_MINIMOS) {
        // Sin suficientes cuadros no se descarta el riesgo: la máquina lo trata como positivo
        ESP_LOGE(TAG, "Solo %u cuadros clasificados tras repetir: error de visión", detalle.clasificados);
        return VISION_ERROR;
    }

    // Mayoría estricta: fuego más humo deben superar la mitad de los cuadros
    const int votos_riesgo = detalle.votos_fuego + detalle.votos_humo;
    resultado_vision_t decision = VISION_SIN_RIESGO;
    if (votos_riesgo * 2 > detalle.clasificados) {
        decision = detalle.votos_fuego >= detalle.votos_humo ? VISION_FUEGO : VISION_HUMO;
    }
    ESP_LOGW(TAG, "Votos: fuego %u, humo %u, normal %u de %u (prob. máx. %.2f, mínimo %.2f) -> %s",
             detalle.votos_fuego, detalle.votos_humo, detalle.votos_normal, detalle.clasificados,
             detalle.probabilidad_max, VISION_PROBABILIDAD_MINIMA, accion_nombre_vision(decision));
    return decision;
}
