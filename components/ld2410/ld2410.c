#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ld2410.h"

static const char *TAG = "LD2410";

#define TAM_BUFFER_UART         1024
#define TAM_COLA_EVENTOS_UART   32
#define TAM_LECTURA_UART        64
#define TAM_PILA_TAREA          3072
#define MAX_DATOS_TRAMA         64
#define VIGENCIA_LECTURA_US     1000000     // Sin tramas en 1 s, el radar se considera sin datos
#define NUCLEO_TAREA            0           // Núcleo del sistema; el 1 es de las comunicaciones

// Trama de reporte: F4 F3 F2 F1 | longitud (2 B, LE) | datos | F8 F7 F6 F5
static const uint8_t CABECERA[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
static const uint8_t COLA[4] = { 0xF8, 0xF7, 0xF6, 0xF5 };

// Datos: tipo (0x01 ingeniería, 0x02 básico) | 0xAA | objetivo (9 B) | [extra ingeniería] | 0x55 | 0x00
#define DATOS_INICIO_OBJETIVO   0xAA
#define DATOS_FIN_OBJETIVO      0x55
#define LONGITUD_MINIMA_DATOS   13

typedef enum {
    FASE_CABECERA,
    FASE_LONGITUD,
    FASE_DATOS,
    FASE_COLA,
} fase_trama_t;

static struct {
    ld2410_config_t config;
    TaskHandle_t tarea;
    portMUX_TYPE cerrojo;
    ld2410_lectura_t ultima;
    volatile uint32_t bytes_recibidos;
    volatile uint32_t tramas_validas;
    volatile uint32_t cabeceras;
    volatile uint32_t desbordes;
    volatile uint32_t cortes;
    QueueHandle_t cola_uart;
    uint8_t muestra[LD2410_TAM_MUESTRA];
    size_t bytes_muestra;

    fase_trama_t fase;
    size_t indice;
    uint16_t longitud;
    uint8_t datos[MAX_DATOS_TRAMA];
} s_radar = {
    .cerrojo = portMUX_INITIALIZER_UNLOCKED,
};

static void reiniciar_trama(void)
{
    s_radar.fase = FASE_CABECERA;
    s_radar.indice = 0;
}

static void interpretar_datos(const uint8_t *datos, uint16_t longitud)
{
    if (longitud < LONGITUD_MINIMA_DATOS || datos[1] != DATOS_INICIO_OBJETIVO) {
        return;
    }
    // El fin del bloque está al final en ambos modos (básico e ingeniería)
    if (datos[longitud - 2] != DATOS_FIN_OBJETIVO) {
        return;
    }

    const uint8_t *objetivo = &datos[2];
    const ld2410_lectura_t lectura = {
        .objetivo = (ld2410_objetivo_t)(objetivo[0] & 0x03),
        .distancia_movimiento_cm = (uint16_t)(objetivo[1] | (objetivo[2] << 8)),
        .energia_movimiento = objetivo[3],
        .distancia_estatico_cm = (uint16_t)(objetivo[4] | (objetivo[5] << 8)),
        .energia_estatico = objetivo[6],
        .distancia_deteccion_cm = (uint16_t)(objetivo[7] | (objetivo[8] << 8)),
        .marca_us = esp_timer_get_time(),
    };

    taskENTER_CRITICAL(&s_radar.cerrojo);
    s_radar.ultima = lectura;
    taskEXIT_CRITICAL(&s_radar.cerrojo);
    s_radar.tramas_validas++;
}

static void procesar_byte(uint8_t byte)
{
    switch (s_radar.fase) {
    case FASE_CABECERA:
        if (byte == CABECERA[s_radar.indice]) {
            if (++s_radar.indice == sizeof(CABECERA)) {
                s_radar.cabeceras++;
                s_radar.fase = FASE_LONGITUD;
                s_radar.indice = 0;
                s_radar.longitud = 0;
            }
        } else {
            s_radar.indice = (byte == CABECERA[0]) ? 1 : 0;
        }
        break;

    case FASE_LONGITUD:
        s_radar.longitud |= (uint16_t)byte << (8 * s_radar.indice);
        if (++s_radar.indice == 2) {
            if (s_radar.longitud == 0 || s_radar.longitud > MAX_DATOS_TRAMA) {
                reiniciar_trama();
            } else {
                s_radar.fase = FASE_DATOS;
                s_radar.indice = 0;
            }
        }
        break;

    case FASE_DATOS:
        s_radar.datos[s_radar.indice] = byte;
        if (++s_radar.indice == s_radar.longitud) {
            s_radar.fase = FASE_COLA;
            s_radar.indice = 0;
        }
        break;

    case FASE_COLA:
        if (byte != COLA[s_radar.indice]) {
            reiniciar_trama();
        } else if (++s_radar.indice == sizeof(COLA)) {
            interpretar_datos(s_radar.datos, s_radar.longitud);
            reiniciar_trama();
        }
        break;
    }
}

// Solo para el diagnóstico: distingue datos perdidos por no leer a tiempo de una línea en bajo
static void atender_eventos_uart(void)
{
    uart_event_t evento;
    while (xQueueReceive(s_radar.cola_uart, &evento, 0) == pdTRUE) {
        if (evento.type == UART_FIFO_OVF || evento.type == UART_BUFFER_FULL) {
            s_radar.desbordes++;
        } else if (evento.type == UART_BREAK) {
            s_radar.cortes++;
        }
    }
}

static void guardar_muestra(const uint8_t *datos, size_t bytes)
{
    const size_t copiados = bytes < LD2410_TAM_MUESTRA ? bytes : LD2410_TAM_MUESTRA;
    taskENTER_CRITICAL(&s_radar.cerrojo);
    memcpy(s_radar.muestra, datos + bytes - copiados, copiados);
    s_radar.bytes_muestra = copiados;
    taskEXIT_CRITICAL(&s_radar.cerrojo);
}

static void tarea_ld2410(void *arg)
{
    uint8_t buffer[TAM_LECTURA_UART];
    for (;;) {
        const int leidos = uart_read_bytes(s_radar.config.puerto, buffer, sizeof(buffer), pdMS_TO_TICKS(100));
        atender_eventos_uart();
        if (leidos > 0) {
            s_radar.bytes_recibidos += (uint32_t)leidos;
            guardar_muestra(buffer, (size_t)leidos);
        }
        for (int i = 0; i < leidos; i++) {
            procesar_byte(buffer[i]);
        }
    }
}

esp_err_t ld2410_iniciar(const ld2410_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "configuración nula");
    ESP_RETURN_ON_FALSE(!s_radar.tarea, ESP_ERR_INVALID_STATE, TAG, "el radar ya está iniciado");

    s_radar.config = *config;
    reiniciar_trama();

    const uart_config_t config_uart = {
        .baud_rate = (int)config->baudios,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(config->puerto, TAM_BUFFER_UART, 0, TAM_COLA_EVENTOS_UART,
                                            &s_radar.cola_uart, 0), TAG, "no se pudo instalar la UART");
    esp_err_t ret = uart_param_config(config->puerto, &config_uart);
    if (ret == ESP_OK) {
        ret = uart_set_pin(config->puerto, config->pin_tx, config->pin_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (ret != ESP_OK) {
        uart_driver_delete(config->puerto);
        ESP_LOGE(TAG, "no se pudo configurar la UART (%s)", esp_err_to_name(ret));
        return ret;
    }

    if (config->pin_out != GPIO_NUM_NC) {
        const gpio_config_t config_out = {
            .pin_bit_mask = 1ULL << config->pin_out,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Sin radar conectado se lee "sin presencia"
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&config_out));
    }

    if (xTaskCreatePinnedToCore(tarea_ld2410, "ld2410", TAM_PILA_TAREA, NULL, 6, &s_radar.tarea,
                                NUCLEO_TAREA) != pdPASS) {
        uart_driver_delete(config->puerto);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Radar iniciado: UART%d a %" PRIu32 " baudios (TX=%d, RX=%d, OUT=%d)",
             config->puerto, config->baudios, config->pin_tx, config->pin_rx, config->pin_out);
    return ESP_OK;
}

bool ld2410_obtener_lectura(ld2410_lectura_t *lectura)
{
    if (!lectura) {
        return false;
    }
    taskENTER_CRITICAL(&s_radar.cerrojo);
    *lectura = s_radar.ultima;
    taskEXIT_CRITICAL(&s_radar.cerrojo);

    return lectura->marca_us != 0 && esp_timer_get_time() - lectura->marca_us <= VIGENCIA_LECTURA_US;
}

void ld2410_obtener_contadores(uint32_t *bytes_recibidos, uint32_t *tramas_validas)
{
    *bytes_recibidos = s_radar.bytes_recibidos;
    *tramas_validas = s_radar.tramas_validas;
}

void ld2410_obtener_diagnostico(ld2410_diagnostico_t *diagnostico)
{
    uint8_t muestra[LD2410_TAM_MUESTRA];
    taskENTER_CRITICAL(&s_radar.cerrojo);
    const size_t bytes_muestra = s_radar.bytes_muestra;
    memcpy(muestra, s_radar.muestra, bytes_muestra);
    taskEXIT_CRITICAL(&s_radar.cerrojo);

    diagnostico->bytes_recibidos = s_radar.bytes_recibidos;
    diagnostico->cabeceras = s_radar.cabeceras;
    diagnostico->tramas_validas = s_radar.tramas_validas;
    diagnostico->desbordes = s_radar.desbordes;
    diagnostico->cortes = s_radar.cortes;
    diagnostico->muestra_hex[0] = '\0';
    for (size_t i = 0; i < bytes_muestra; i++) {
        snprintf(&diagnostico->muestra_hex[3 * i], 4, "%02X ", muestra[i]);
    }
}

int ld2410_leer_salida(void)
{
    if (!s_radar.tarea || s_radar.config.pin_out == GPIO_NUM_NC) {
        return -1;
    }
    return gpio_get_level(s_radar.config.pin_out);
}
