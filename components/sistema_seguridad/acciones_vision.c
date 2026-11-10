// Fase 3: detección visual con la cámara OV3660 y el clasificador fuego / humo / normal

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

resultado_vision_t accion_detectar_fuego_humo(void)
{
    if (!camara_disponible()) {
        ESP_LOGE(TAG, "Cámara no disponible");
        return VISION_ERROR;
    }
    if (!clasificador_disponible()) {
        ESP_LOGE(TAG, "Clasificador no disponible");
    }

    int votos[NUM_CLASES] = { 0 };
    int clasificados = 0;
    for (int i = 0; i < CUADROS_POR_ANALISIS; i++) {
        if (i > 0) {
            vTaskDelay(pdMS_TO_TICKS(PERIODO_CUADROS_MS));
        }
        camera_fb_t *cuadro = camara_capturar(pdMS_TO_TICKS(ESPERA_CAMARA_MS));
        if (!cuadro) {
            continue;
        }

        resultado_clasificacion_t resultado;
        const bool ok = clasificador_disponible() &&
                        clasificador_clasificar_jpeg(cuadro->buf, cuadro->len, &resultado) == ESP_OK;
        if (ok) {
            clasificados++;
            votos[resultado.clase]++;
            ESP_LOGI(TAG, "Cuadro %d: fuego %.3f | humo %.3f | normal %.3f -> %s (%lu ms)", i + 1,
                     resultado.probabilidad[CLASE_FUEGO], resultado.probabilidad[CLASE_HUMO],
                     resultado.probabilidad[CLASE_NORMAL], clasificador_nombre_clase(resultado.clase),
                     (unsigned long)resultado.tiempo_ms);
        }
        guardar_evidencia(cuadro, ok ? &resultado : NULL);
        camara_liberar(cuadro);
    }

    if (clasificados == 0) {
        // Sin poder clasificar no se descarta el riesgo: la máquina lo trata como positivo
        ESP_LOGE(TAG, "No se pudo clasificar ningún cuadro");
        return VISION_ERROR;
    }

    // Voto por mayoría; un empate cuenta como riesgo (falla segura)
    const int votos_riesgo = votos[CLASE_FUEGO] + votos[CLASE_HUMO];
    resultado_vision_t decision = VISION_SIN_RIESGO;
    if (votos_riesgo * 2 >= clasificados) {
        decision = votos[CLASE_FUEGO] >= votos[CLASE_HUMO] ? VISION_FUEGO : VISION_HUMO;
    }
    ESP_LOGW(TAG, "Votos: fuego %d, humo %d, normal %d de %d -> %s", votos[CLASE_FUEGO], votos[CLASE_HUMO],
             votos[CLASE_NORMAL], clasificados, accion_nombre_vision(decision));
    return decision;
}
