#include <cinttypes>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "almacenamiento_sd.h"
#include "camara_vision.h"
#include "captura_dataset.h"
#include "clasificador.h"
#include "comunicaciones.h"
#include "horarios.h"
#include "ld2410.h"
#include "maquina_estados.h"
#include "monitor_temperatura.h"
#include "pines.h"
#include "valvula.h"

static const char *TAG = "SATCP";

// Temperatura de la referencia del paso 2: agua hirviendo. Baja ~1 °C cada 300 m de altitud
// (nivel del mar: 100 °C, Guadalajara: ~95 °C, CDMX: ~92.5 °C)
#define TEMPERATURA_REFERENCIA_C    100.0f

#define PULSACION_LARGA_MS          3000    // Paso 2: referencia
#define PULSACION_MUY_LARGA_MS      8000    // Borrar calibración
#define PULSACION_CLAVE_FABRICA_MS  15000   // Restaurar la clave del equipo a la de la etiqueta
#define PERIODO_ESTADO_MS           5000

// Fotos periódicas en la SD para reentrenar el modelo (0 = desactivado)
#define INTERVALO_CAPTURA_DATASET_S 30

static void iniciar_nvs(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// Botón BOOT: en alerta = responder, bloqueado = rearmar (tras abrir la válvula a mano).
// En los demás estados: corta = fondo, 3 s = referencia, 8 s = borrar calibración, 15 s = clave de fábrica
static void tarea_boton_calibracion(void *arg) {
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << PIN_BOTON_CALIBRACION,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(PIN_BOTON_CALIBRACION) != 0) {
            continue;
        }

        const TickType_t inicio = xTaskGetTickCount();
        while (gpio_get_level(PIN_BOTON_CALIBRACION) == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        const uint32_t duracion_ms = pdTICKS_TO_MS(xTaskGetTickCount() - inicio);

        if (duracion_ms < 50) {
            continue;   // Rebote
        }

        // Mientras no haya app de celular, el botón hace de respuesta del usuario y de rearme
        const estado_sistema_t estado = maquina_estados_obtener_estado();
        if (estado == ESTADO_SISTEMA_ALERTA_USUARIO) {
            maquina_estados_notificar_respuesta_usuario();
            continue;
        }
        if (estado == ESTADO_SISTEMA_BLOQUEADO) {
            maquina_estados_rearmar();
            continue;
        }

        esp_err_t ret;
        if (duracion_ms >= PULSACION_CLAVE_FABRICA_MS) {
            ret = comunicaciones_restaurar_clave_fabrica();
            ESP_LOGW(TAG, "Clave del equipo restaurada a la de fábrica: %s", ret == ESP_OK ? "ok" : esp_err_to_name(ret));
            continue;
        }
        if (duracion_ms >= PULSACION_MUY_LARGA_MS) {
            ret = monitor_temperatura_borrar_calibracion();
        } else if (duracion_ms >= PULSACION_LARGA_MS) {
            ret = monitor_temperatura_calibrar_referencia(TEMPERATURA_REFERENCIA_C);
        } else {
            ret = monitor_temperatura_calibrar_fondo();
        }
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Calibración completada");
        } else {
            ESP_LOGE(TAG, "Calibración fallida (%s), repita el paso", esp_err_to_name(ret));
        }
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Iniciando sistema SATCP...");
    iniciar_nvs();

    monitor_temperatura_config_t config_temp = MONITOR_TEMPERATURA_CONFIG_POR_DEFECTO();
    config_temp.pin_sda = PIN_I2C_SDA;
    config_temp.pin_scl = PIN_I2C_SCL;
    if (monitor_temperatura_iniciar(&config_temp) != ESP_OK) {
        ESP_LOGE(TAG, "Error crítico: no se pudo iniciar el monitor de temperatura.");
        return;
    }

    const ld2410_config_t config_radar = {
        .puerto = UART_RADAR,
        .pin_tx = PIN_RADAR_TX,
        .pin_rx = PIN_RADAR_RX,
        .pin_out = PIN_RADAR_OUT,
        .baudios = LD2410_BAUDIOS_POR_DEFECTO,
    };
    if (ld2410_iniciar(&config_radar) != ESP_OK) {
        // La máquina de estados sigue: sin radar asume ausencia y pasa a la cámara (falla segura)
        ESP_LOGE(TAG, "No se pudo iniciar el radar de presencia.");
    }

    // La cámara va antes que la SD: así lo validó la prueba anterior de la placa
    const camara_config_t config_camara = {
        .pin_xclk = PIN_CAM_XCLK,
        .pin_sccb_sda = PIN_CAM_SIOD,
        .pin_sccb_scl = PIN_CAM_SIOC,
        .pin_d = PINES_CAM_DATOS,
        .pin_vsync = PIN_CAM_VSYNC,
        .pin_href = PIN_CAM_HREF,
        .pin_pclk = PIN_CAM_PCLK,
        .xclk_hz = 10000000,            // Frecuencia ya probada con esta OV3660
        .resolucion = FRAMESIZE_VGA,    // 640x480: suficiente para el modelo y liviano en la SD
        .calidad_jpeg = 12,
    };
    if (camara_iniciar(&config_camara) != ESP_OK) {
        // La fase visual devolverá VISION_ERROR, que la máquina trata como positivo (falla segura)
        ESP_LOGE(TAG, "No se pudo iniciar la cámara.");
    }

    if (clasificador_iniciar() != ESP_OK) {
        // Sin modelo la fase visual devuelve VISION_ERROR, tratado como positivo (falla segura)
        ESP_LOGE(TAG, "No se pudo cargar el modelo de visión.");
    }

    const sd_config_t config_sd = {
        .pin_clk = PIN_SD_CLK,
        .pin_cmd = PIN_SD_CMD,
        .pin_d0 = PIN_SD_D0,
    };
    if (sd_iniciar(&config_sd) != ESP_OK) {
        ESP_LOGW(TAG, "Sin SD: no se guardarán evidencias ni imágenes para el dataset.");
    }

    if (INTERVALO_CAPTURA_DATASET_S > 0 && camara_disponible() && sd_disponible()) {
        captura_dataset_iniciar(INTERVALO_CAPTURA_DATASET_S);
    }

    if (horarios_iniciar() != ESP_OK) {
        // Sin horarios el resto del sistema sigue igual: solo no habrá cierres programados
        ESP_LOGE(TAG, "No se pudieron cargar los horarios programados.");
    }

    const valvula_config_t config_valvula = {
        .pin_pulso = PIN_VALVULA_PULSO,
        .duracion_pulso_ms = DURACION_PULSO_VALVULA_MS,
        .pin_posicion = PIN_VALVULA_POSICION,
        .nivel_cerrada = 0,
    };
    if (valvula_iniciar(&config_valvula) != ESP_OK) {
        // La máquina sigue: el cierre fallará y quedará en el reporte, pero alerta y visión funcionan
        ESP_LOGE(TAG, "No se pudo iniciar la válvula.");
    }

    const maquina_estados_config_t config_maquina = MAQUINA_ESTADOS_CONFIG_POR_DEFECTO();
    if (maquina_estados_iniciar(&config_maquina) != ESP_OK) {
        ESP_LOGE(TAG, "Error crítico: no se pudo iniciar la máquina de estados.");
        return;
    }

    // Sin comunicaciones el sistema sigue protegiendo: alertas y respuesta quedan en el botón BOOT
    comunicaciones_iniciar();

    xTaskCreatePinnedToCore(tarea_boton_calibracion, "boton_cal", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "Botón BOOT: corta = fondo | 3 s = referencia (%.1f °C) | 8 s = borrar calibración | 15 s = clave de fábrica",
             TEMPERATURA_REFERENCIA_C);

    muestra_temperatura_t muestra;
    ld2410_lectura_t radar;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(PERIODO_ESTADO_MS));
        if (ld2410_obtener_lectura(&radar)) {
            ESP_LOGI(TAG, "Radar: estado %d | movimiento %u cm (energía %u) | estático %u cm (energía %u) | OUT=%d",
                     radar.objetivo, radar.distancia_movimiento_cm, radar.energia_movimiento,
                     radar.distancia_estatico_cm, radar.energia_estatico, ld2410_leer_salida());
        } else {
            ld2410_diagnostico_t diagnostico;
            ld2410_obtener_diagnostico(&diagnostico);
            ESP_LOGW(TAG, "Radar: sin tramas por UART | OUT=%d | bytes %" PRIu32 ", cabeceras %" PRIu32
                     ", tramas %" PRIu32 ", desbordes %" PRIu32 ", cortes %" PRIu32 " | últimos: %s",
                     ld2410_leer_salida(), diagnostico.bytes_recibidos, diagnostico.cabeceras,
                     diagnostico.tramas_validas, diagnostico.desbordes, diagnostico.cortes, diagnostico.muestra_hex);
        }
        if (!monitor_temperatura_obtener_ultima_muestra(&muestra)) {
            if (!monitor_temperatura_sensor_ok()) {
                ESP_LOGW(TAG, "[%s] Sensor de temperatura sin respuesta: revisa SDA=%d SCL=%d y 3V3/GND",
                         maquina_estados_nombre(maquina_estados_obtener_estado()), PIN_I2C_SDA, PIN_I2C_SCL);
            }
            continue;
        }
        float linea_base;
        if (monitor_temperatura_obtener_linea_base(&linea_base)) {
            ESP_LOGI(TAG, "[%s] Medida %.2f °C | Estimada %.1f °C | Línea base %.1f °C",
                     maquina_estados_nombre(maquina_estados_obtener_estado()), muestra.temp_objeto_c,
                     monitor_temperatura_estimar(&muestra), linea_base);
        } else {
            ESP_LOGI(TAG, "[%s] Medida %.2f °C | Estimada %.1f °C",
                     maquina_estados_nombre(maquina_estados_obtener_estado()), muestra.temp_objeto_c,
                     monitor_temperatura_estimar(&muestra));
        }
    }
}
