#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "camara_vision.h"

static const char *TAG = "CAMARA";

#define CUADROS_DESCARTE    3   // Los primeros cuadros salen con exposición sin ajustar

static SemaphoreHandle_t s_mutex;
static framesize_t s_resolucion_normal;

esp_err_t camara_iniciar(const camara_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "configuración nula");
    ESP_RETURN_ON_FALSE(!s_mutex, ESP_ERR_INVALID_STATE, TAG, "la cámara ya está iniciada");

    const camera_config_t config_camara = {
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = config->pin_xclk,
        .pin_sccb_sda = config->pin_sccb_sda,
        .pin_sccb_scl = config->pin_sccb_scl,
        .pin_d7 = config->pin_d[7],
        .pin_d6 = config->pin_d[6],
        .pin_d5 = config->pin_d[5],
        .pin_d4 = config->pin_d[4],
        .pin_d3 = config->pin_d[3],
        .pin_d2 = config->pin_d[2],
        .pin_d1 = config->pin_d[1],
        .pin_d0 = config->pin_d[0],
        .pin_vsync = config->pin_vsync,
        .pin_href = config->pin_href,
        .pin_pclk = config->pin_pclk,
        .xclk_freq_hz = config->xclk_hz,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = config->resolucion,
        .jpeg_quality = config->calidad_jpeg,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,    // Siempre el cuadro más reciente, no uno viejo en cola
    };
    ESP_RETURN_ON_ERROR(esp_camera_init(&config_camara), TAG, "no se pudo inicializar la cámara");
    s_resolucion_normal = config->resolucion;

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        ESP_LOGI(TAG, "Sensor detectado: PID 0x%04X", sensor->id.PID);
    }

    for (int i = 0; i < CUADROS_DESCARTE; i++) {
        camera_fb_t *cuadro = esp_camera_fb_get();
        if (cuadro) {
            esp_camera_fb_return(cuadro);
        }
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        esp_camera_deinit();
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Cámara lista (JPEG, calidad %d)", config->calidad_jpeg);
    return ESP_OK;
}

camera_fb_t *camara_capturar(TickType_t espera)
{
    if (!s_mutex || xSemaphoreTake(s_mutex, espera) != pdTRUE) {
        return NULL;
    }
    camera_fb_t *cuadro = esp_camera_fb_get();
    if (!cuadro) {
        ESP_LOGE(TAG, "Falló la captura");
        xSemaphoreGive(s_mutex);
    }
    return cuadro;
}

void camara_liberar(camera_fb_t *cuadro)
{
    if (!cuadro) {
        return;
    }
    esp_camera_fb_return(cuadro);
    xSemaphoreGive(s_mutex);
}

esp_err_t camara_modo_vivo(bool activo, TickType_t espera)
{
    ESP_RETURN_ON_FALSE(s_mutex, ESP_ERR_INVALID_STATE, TAG, "la cámara no está iniciada");
    if (xSemaphoreTake(s_mutex, espera) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    // Los buffers se reservaron para la resolución normal: una menor siempre cabe
    const framesize_t resolucion = activo && CAMARA_RESOLUCION_VIVO < s_resolucion_normal
                                   ? CAMARA_RESOLUCION_VIVO : s_resolucion_normal;
    sensor_t *sensor = esp_camera_sensor_get();
    esp_err_t ret = sensor && sensor->set_framesize(sensor, resolucion) == 0 ? ESP_OK : ESP_FAIL;
    if (ret == ESP_OK) {
        // El primer cuadro tras el cambio puede salir con el tamaño anterior
        for (int i = 0; i < CUADROS_DESCARTE; i++) {
            camera_fb_t *cuadro = esp_camera_fb_get();
            if (cuadro) {
                esp_camera_fb_return(cuadro);
            }
        }
    }
    xSemaphoreGive(s_mutex);
    return ret;
}

bool camara_disponible(void)
{
    return s_mutex != NULL;
}
