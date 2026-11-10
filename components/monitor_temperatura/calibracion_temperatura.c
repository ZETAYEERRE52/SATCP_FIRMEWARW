#include <math.h>
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "calibracion_temperatura.h"

static const char *TAG = "CALIBRACION";

#define CALIBRACION_VERSION     1
#define NVS_ESPACIO             "satcp"
#define NVS_CLAVE               "cal_temp"
#define KELVIN                  273.15f
#define MIN_BLOQUES             10          // Segundos mínimos de datos para calibrar
#define MAX_BLOQUES             60
#define RUIDO_MINIMO_C          0.02f       // Resolución del MLX90614
#define SIGMAS_SENAL_MINIMA     5.0f        // La referencia debe destacar sobre el ruido

static inline float a_kelvin(float celsius)
{
    return celsius + KELVIN;
}

static inline float cuarta(float x)
{
    const float x2 = x * x;
    return x2 * x2;
}

typedef struct {
    float objeto_c;
    float ambiente_c;
    float ruido_1s_c;
    float deriva_c;         // Diferencia entre el primer y el último tercio de la ventana
} estadistica_ventana_t;

// Promedia la ventana en bloques de 1 s: el ruido se mide sobre esos promedios porque
// es lo que usa el detector, y así se incluye la correlación del filtro interno del sensor
static esp_err_t analizar_ventana(const muestra_temperatura_t *muestras, size_t cantidad,
                                  size_t muestras_por_segundo, estadistica_ventana_t *est)
{
    ESP_RETURN_ON_FALSE(muestras && est && muestras_por_segundo > 0, ESP_ERR_INVALID_ARG, TAG, "argumentos inválidos");

    size_t bloques = cantidad / muestras_por_segundo;
    ESP_RETURN_ON_FALSE(bloques >= MIN_BLOQUES, ESP_ERR_INVALID_SIZE, TAG,
                        "se necesitan al menos %d s de lecturas (hay %u)", MIN_BLOQUES, (unsigned)bloques);
    if (bloques > MAX_BLOQUES) {
        bloques = MAX_BLOQUES;
    }
    // Usar los bloques más recientes
    const muestra_temperatura_t *inicio = muestras + cantidad - bloques * muestras_por_segundo;

    float medias[MAX_BLOQUES];
    float suma_objeto = 0.0f;
    float suma_ambiente = 0.0f;
    for (size_t b = 0; b < bloques; b++) {
        float suma_bloque = 0.0f;
        for (size_t i = 0; i < muestras_por_segundo; i++) {
            const muestra_temperatura_t *m = &inicio[b * muestras_por_segundo + i];
            suma_bloque += m->temp_objeto_c;
            suma_ambiente += m->temp_ambiente_c;
        }
        medias[b] = suma_bloque / (float)muestras_por_segundo;
        suma_objeto += medias[b];
    }
    est->objeto_c = suma_objeto / (float)bloques;
    est->ambiente_c = suma_ambiente / (float)(bloques * muestras_por_segundo);

    // Dos pasadas: con float, sumar cuadrados de valores ~25 °C perdería la varianza (~0.01)
    float suma_cuadrados = 0.0f;
    for (size_t b = 0; b < bloques; b++) {
        const float d = medias[b] - est->objeto_c;
        suma_cuadrados += d * d;
    }
    est->ruido_1s_c = fmaxf(sqrtf(suma_cuadrados / (float)(bloques - 1)), RUIDO_MINIMO_C);

    const size_t tercio = bloques / 3;
    float primero = 0.0f;
    float ultimo = 0.0f;
    for (size_t b = 0; b < tercio; b++) {
        primero += medias[b];
        ultimo += medias[bloques - tercio + b];
    }
    est->deriva_c = (ultimo - primero) / (float)tercio;
    return ESP_OK;
}

esp_err_t calibracion_calcular_fondo(const muestra_temperatura_t *muestras, size_t cantidad,
                                     size_t muestras_por_segundo, calibracion_temperatura_t *cal)
{
    ESP_RETURN_ON_FALSE(cal, ESP_ERR_INVALID_ARG, TAG, "calibración nula");

    estadistica_ventana_t est;
    ESP_RETURN_ON_ERROR(analizar_ventana(muestras, cantidad, muestras_por_segundo, &est), TAG, "ventana inválida");

    const float deriva_maxima = fmaxf(5.0f * est.ruido_1s_c, 0.5f);
    ESP_RETURN_ON_FALSE(fabsf(est.deriva_c) <= deriva_maxima, ESP_ERR_INVALID_STATE, TAG,
                        "la escena cambió %.2f °C durante la medición: ¿estufa encendida o alguien pasando?",
                        est.deriva_c);

    cal->version = CALIBRACION_VERSION;
    cal->fondo_valido = true;
    cal->fondo_c = est.objeto_c;
    cal->ambiente_fondo_c = est.ambiente_c;
    cal->ruido_1s_c = est.ruido_1s_c;
    if (!cal->referencia_valida) {
        cal->factor_llenado = 1.0f;
        cal->ganancia_ruido = 1.0f;
    }

    ESP_LOGI(TAG, "Fondo: %.2f °C (sensor a %.2f °C), ruido del promedio de 1 s: ±%.3f °C",
             cal->fondo_c, cal->ambiente_fondo_c, cal->ruido_1s_c);
    return ESP_OK;
}

esp_err_t calibracion_calcular_referencia(const muestra_temperatura_t *muestras, size_t cantidad,
                                          size_t muestras_por_segundo, float temperatura_referencia_c,
                                          calibracion_temperatura_t *cal)
{
    ESP_RETURN_ON_FALSE(cal, ESP_ERR_INVALID_ARG, TAG, "calibración nula");
    ESP_RETURN_ON_FALSE(cal->fondo_valido, ESP_ERR_INVALID_STATE, TAG, "primero calibre el fondo (estufa apagada)");

    estadistica_ventana_t est;
    ESP_RETURN_ON_ERROR(analizar_ventana(muestras, cantidad, muestras_por_segundo, &est), TAG, "ventana inválida");

    const float deriva_maxima = fmaxf(3.0f * cal->ruido_1s_c, 0.3f);
    ESP_RETURN_ON_FALSE(fabsf(est.deriva_c) <= deriva_maxima, ESP_ERR_INVALID_STATE, TAG,
                        "la lectura aún cambia (%.2f °C): espere a que la referencia se estabilice", est.deriva_c);

    // El fondo se corrige con la deriva de la temperatura interna del sensor desde el paso 1
    const float fondo_k = a_kelvin(cal->fondo_c + (est.ambiente_c - cal->ambiente_fondo_c));
    const float medida_k = a_kelvin(est.objeto_c);
    const float referencia_k = a_kelvin(temperatura_referencia_c);

    const float senal_c = medida_k - fondo_k;
    ESP_RETURN_ON_FALSE(senal_c >= SIGMAS_SENAL_MINIMA * cal->ruido_1s_c, ESP_ERR_INVALID_RESPONSE, TAG,
                        "señal insuficiente (+%.2f °C sobre el fondo, ruido ±%.3f °C): acerque el sensor "
                        "o use uno de campo de visión más estrecho", senal_c, cal->ruido_1s_c);

    const float factor = (cuarta(medida_k) - cuarta(fondo_k)) / (cuarta(referencia_k) - cuarta(fondo_k));
    ESP_RETURN_ON_FALSE(factor > 0.0f && factor <= 1.0f, ESP_ERR_INVALID_ARG, TAG,
                        "factor de llenado fuera de rango (%.4f): ¿temperatura de referencia correcta?", factor);

    cal->referencia_valida = true;
    cal->factor_llenado = factor;
    // Derivada de la inversión: un error de 1 °C en la lectura equivale a esto en la estimación
    cal->ganancia_ruido = (medida_k * medida_k * medida_k) / (factor * referencia_k * referencia_k * referencia_k);

    ESP_LOGI(TAG, "Referencia %.1f °C leída como %.2f °C (+%.2f °C sobre el fondo)",
             temperatura_referencia_c, est.objeto_c, senal_c);
    ESP_LOGI(TAG, "Factor de llenado: %.4f (la olla ocupa ~%.2f %% del campo de visión)", factor, factor * 100.0f);
    ESP_LOGI(TAG, "Resolución estimada: ±%.1f °C (ganancia de ruido x%.1f)",
             cal->ruido_1s_c * cal->ganancia_ruido, cal->ganancia_ruido);
    return ESP_OK;
}

float calibracion_estimar_temperatura(const calibracion_temperatura_t *cal, float medida_c, float ambiente_c)
{
    if (!cal || !cal->fondo_valido || !cal->referencia_valida) {
        return medida_c;
    }

    const float fondo_k = a_kelvin(cal->fondo_c + (ambiente_c - cal->ambiente_fondo_c));
    const float exceso = (cuarta(a_kelvin(medida_c)) - cuarta(fondo_k)) / cal->factor_llenado;
    if (exceso <= 0.0f) {
        // Escena más fría que el fondo: no hay fuente caliente que amplificar
        return medida_c;
    }
    return powf(cuarta(fondo_k) + exceso, 0.25f) - KELVIN;
}

esp_err_t calibracion_guardar(const calibracion_temperatura_t *cal)
{
    ESP_RETURN_ON_FALSE(cal, ESP_ERR_INVALID_ARG, TAG, "calibración nula");

    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_ESPACIO, NVS_READWRITE, &nvs), TAG, "no se pudo abrir NVS");
    esp_err_t ret = nvs_set_blob(nvs, NVS_CLAVE, cal, sizeof(*cal));
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return ret;
}

esp_err_t calibracion_cargar(calibracion_temperatura_t *cal)
{
    ESP_RETURN_ON_FALSE(cal, ESP_ERR_INVALID_ARG, TAG, "calibración nula");

    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_ESPACIO, NVS_READONLY, &nvs);
    if (ret != ESP_OK) {
        return ret;
    }
    calibracion_temperatura_t leida;
    size_t tamano = sizeof(leida);
    ret = nvs_get_blob(nvs, NVS_CLAVE, &leida, &tamano);
    nvs_close(nvs);
    if (ret != ESP_OK) {
        return ret;
    }
    if (tamano != sizeof(leida) || leida.version != CALIBRACION_VERSION) {
        return ESP_ERR_INVALID_VERSION;
    }
    *cal = leida;
    return ESP_OK;
}

esp_err_t calibracion_borrar(void)
{
    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_ESPACIO, NVS_READWRITE, &nvs), TAG, "no se pudo abrir NVS");
    esp_err_t ret = nvs_erase_key(nvs, NVS_CLAVE);
    if (ret == ESP_OK || ret == ESP_ERR_NVS_NOT_FOUND) {
        ret = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return ret;
}
