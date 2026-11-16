#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "monitor_temperatura.h"

static const char *TAG = "MONITOR_TEMP";

#define NVS_ESPACIO_UMBRAL          "satcp"
#define NVS_CLAVE_UMBRAL            "umbral_inc"    // Umbral de incremento medido con la parrilla al mínimo
#define UMBRAL_INCREMENTO_MIN_C     1.0f
#define UMBRAL_INCREMENTO_MAX_C     30.0f

#define MUESTRAS_POR_SEGUNDO        MONITOR_FRECUENCIA_HZ
#define TAM_COLA_EVENTOS            8
#define TAM_PILA_TAREA              4096
#define FALLAS_PARA_ALERTA          10      // 1 s sin lecturas válidas
#define FALLAS_PARA_REINICIAR_BUS   5
#define DURACION_CAL_FONDO_S        MONITOR_VENTANA_S
#define DURACION_CAL_REFERENCIA_S   30
#define SIGMAS_DETECCION_TASA       4.0f    // La tasa debe superar 4 desviaciones del ruido

typedef enum {
    ESTADO_NORMAL = 0,
    ESTADO_INCREMENTO,
    ESTADO_CRITICO,
} estado_monitor_t;

static struct {
    monitor_temperatura_config_t config;
    i2c_master_bus_handle_t bus;
    mlx90614_handle_t sensor;
    buffer_circular_t buffer;
    SemaphoreHandle_t mutex;
    QueueHandle_t cola_eventos;
    TaskHandle_t tarea;

    estado_monitor_t estado;
    uint16_t contador_confirmacion;
    uint16_t contador_normalizacion;
    uint32_t fallas_consecutivas;
    bool sensor_en_falla;

    calibracion_temperatura_t calibracion;
    float tasa_umbral_c_por_min;    // max(configurada, la que exige el ruido medido)

    bool linea_base_valida;
    float linea_base_c;

    // Tras una revisión, la lectura más baja desde entonces: subir umbral_salto_c sobre ella vuelve a
    // avisar aunque la anomalía siga activa, sin esperar a que se normalice
    bool revision_valida;
    float piso_revision_c;
    uint16_t contador_resalto;

    // Umbral de incremento (salto, superación de la línea base y nuevo salto): el de fábrica o el
    // medido encendiendo una parrilla al mínimo
    float umbral_incremento_fabrica_c;
    bool umbral_incremento_calibrado;
} s_monitor;

static void aplicar_umbral_incremento(float umbral_c)
{
    s_monitor.config.umbral_salto_c = umbral_c;
    s_monitor.config.margen_superacion_c = umbral_c;
}

static void cargar_umbral_incremento(void)
{
    nvs_handle_t nvs;
    float umbral_c = 0.0f;
    size_t tam = sizeof(umbral_c);
    if (nvs_open(NVS_ESPACIO_UMBRAL, NVS_READONLY, &nvs) != ESP_OK) {
        return;
    }
    const bool leido = nvs_get_blob(nvs, NVS_CLAVE_UMBRAL, &umbral_c, &tam) == ESP_OK && tam == sizeof(umbral_c);
    nvs_close(nvs);
    if (leido && umbral_c >= UMBRAL_INCREMENTO_MIN_C && umbral_c <= UMBRAL_INCREMENTO_MAX_C) {
        aplicar_umbral_incremento(umbral_c);
        s_monitor.umbral_incremento_calibrado = true;
        ESP_LOGI(TAG, "Umbral de incremento calibrado con la parrilla: %.1f °C", umbral_c);
    }
}

// Debe llamarse con el mutex tomado (o antes de crear la tarea)
static void aplicar_calibracion(const calibracion_temperatura_t *cal)
{
    s_monitor.calibracion = *cal;

    float tasa_por_ruido = 0.0f;
    if (cal->fondo_valido) {
        // La tasa sale de la diferencia de dos promedios de 1 s: su ruido es sqrt(2) veces el de cada uno
        const float ruido_estimado = cal->ruido_1s_c * cal->ganancia_ruido;
        const float minutos = (float)s_monitor.config.ventana_tendencia_s / 60.0f;
        tasa_por_ruido = SIGMAS_DETECCION_TASA * sqrtf(2.0f) * ruido_estimado / minutos;
    }
    s_monitor.tasa_umbral_c_por_min = fmaxf(s_monitor.config.tasa_maxima_c_por_min, tasa_por_ruido);
}

static bool promediar_estimada(size_t antiguedad, float *temperatura_c, int64_t *marca_us)
{
    float objeto_c;
    float ambiente_c;
    if (!buffer_circular_promediar(&s_monitor.buffer, antiguedad, MUESTRAS_POR_SEGUNDO,
                                   &objeto_c, &ambiente_c, marca_us)) {
        return false;
    }
    *temperatura_c = calibracion_estimar_temperatura(&s_monitor.calibracion, objeto_c, ambiente_c);
    return true;
}

static void publicar_evento(tipo_evento_temperatura_t tipo, motivo_anomalia_t motivo, float medida,
                            float temperatura, float referencia, float tasa)
{
    const evento_temperatura_t evento = {
        .tipo = tipo,
        .motivo = motivo,
        .medida_c = medida,
        .temperatura_c = temperatura,
        .referencia_c = referencia,
        .tasa_c_por_min = tasa,
        .marca_us = esp_timer_get_time(),
    };
    if (xQueueSend(s_monitor.cola_eventos, &evento, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Cola de eventos llena, evento %d descartado", tipo);
    }
}

// Debe llamarse con el mutex tomado
static void evaluar_anomalia(void)
{
    const monitor_temperatura_config_t *cfg = &s_monitor.config;

    // Promediar un segundo completo filtra el ruido del sensor sin perder reacción.
    // Con calibración, se trabaja sobre la temperatura estimada del recipiente
    float t_actual;
    int64_t marca_actual;
    if (!promediar_estimada(0, &t_actual, &marca_actual)) {
        return;
    }

    float t_referencia = NAN;
    float tasa = 0.0f;
    int64_t marca_referencia;
    const size_t antiguedad_referencia = (size_t)cfg->ventana_tendencia_s * MUESTRAS_POR_SEGUNDO;
    const bool hay_referencia = promediar_estimada(antiguedad_referencia, &t_referencia, &marca_referencia);
    if (hay_referencia) {
        // Se usa el tiempo real entre promedios, así una lectura perdida no distorsiona la tasa
        const float minutos = (float)(marca_actual - marca_referencia) / 60e6f;
        if (minutos > 0.0f) {
            tasa = (t_actual - t_referencia) / minutos;
        }
    }

    if (s_monitor.linea_base_valida && t_actual < s_monitor.linea_base_c - cfg->margen_descarte_c) {
        ESP_LOGI(TAG, "Línea base de %.1f °C descartada (temperatura bajó a %.1f °C)",
                 s_monitor.linea_base_c, t_actual);
        s_monitor.linea_base_valida = false;
    }

    // Con línea base, la subida rápida solo cuenta por encima de la temperatura ya verificada
    const float minimo_para_tasa = s_monitor.linea_base_valida
                                   ? fmaxf(cfg->umbral_minimo_c, s_monitor.linea_base_c)
                                   : cfg->umbral_minimo_c;

    // El salto se mide sobre la lectura directa del sensor: el umbral salió de pruebas con ella
    float salto = 0.0f;
    float medida_actual;
    float medida_previa;
    float ambiente;
    int64_t marca;
    const size_t antiguedad_salto = (size_t)cfg->ventana_salto_s * MUESTRAS_POR_SEGUNDO;
    const bool hay_medida = buffer_circular_promediar(&s_monitor.buffer, 0, MUESTRAS_POR_SEGUNDO,
                                                      &medida_actual, &ambiente, &marca);
    if (hay_medida && buffer_circular_promediar(&s_monitor.buffer, antiguedad_salto, MUESTRAS_POR_SEGUNDO,
                                                &medida_previa, &ambiente, &marca)) {
        salto = medida_actual - medida_previa;
    }
    if (hay_medida && s_monitor.revision_valida) {
        s_monitor.piso_revision_c = fminf(s_monitor.piso_revision_c, medida_actual);
    }

    const float medida = hay_medida ? medida_actual : NAN;
    estado_monitor_t condicion = ESTADO_NORMAL;
    motivo_anomalia_t motivo = MOTIVO_NINGUNO;
    if (t_actual >= cfg->umbral_critico_c) {
        condicion = ESTADO_CRITICO;
        motivo = MOTIVO_CRITICA;
    } else if (salto >= cfg->umbral_salto_c) {
        condicion = ESTADO_INCREMENTO;
        motivo = MOTIVO_SALTO_BRUSCO;
    } else if (s_monitor.linea_base_valida && t_actual >= s_monitor.linea_base_c + cfg->margen_superacion_c) {
        condicion = ESTADO_INCREMENTO;
        motivo = MOTIVO_SUPERO_LINEA_BASE;
    } else if (hay_referencia && t_actual >= minimo_para_tasa && tasa >= s_monitor.tasa_umbral_c_por_min) {
        condicion = ESTADO_INCREMENTO;
        motivo = MOTIVO_SUBIDA_SOSTENIDA;
    }

    if (condicion > s_monitor.estado) {
        // Escalar solo si la condición se sostiene, para ignorar picos aislados. El salto y la superación
        // de la línea base esperan más: una mano o una olla frente al sensor los provoca un instante
        const uint16_t necesarias = (motivo == MOTIVO_SALTO_BRUSCO || motivo == MOTIVO_SUPERO_LINEA_BASE)
                                    ? cfg->muestras_confirmacion_salto : cfg->muestras_confirmacion;
        s_monitor.contador_normalizacion = 0;
        if (++s_monitor.contador_confirmacion >= necesarias) {
            s_monitor.contador_confirmacion = 0;
            s_monitor.estado = condicion;
            if (condicion == ESTADO_CRITICO) {
                ESP_LOGW(TAG, "TEMPERATURA CRÍTICA: %.1f °C", t_actual);
                publicar_evento(EVENTO_TEMPERATURA_CRITICA, motivo, medida, t_actual, t_referencia, tasa);
            } else {
                ESP_LOGW(TAG, "Incremento anómalo (%s): salto %+.1f °C en %u s, medida %.1f °C, estimada %.1f °C, "
                         "%.1f °C/min", monitor_temperatura_nombre_motivo(motivo), salto, cfg->ventana_salto_s,
                         medida, t_actual, tasa);
                publicar_evento(EVENTO_INCREMENTO_ANOMALO, motivo, medida, t_actual, t_referencia, tasa);
            }
        }
    } else if (condicion == ESTADO_NORMAL && s_monitor.estado != ESTADO_NORMAL) {
        s_monitor.contador_confirmacion = 0;
        if (++s_monitor.contador_normalizacion >= cfg->muestras_normalizacion) {
            s_monitor.contador_normalizacion = 0;
            s_monitor.estado = ESTADO_NORMAL;
            s_monitor.revision_valida = false;
            ESP_LOGI(TAG, "Temperatura normalizada: %.1f °C", t_actual);
            publicar_evento(EVENTO_TEMPERATURA_NORMALIZADA, MOTIVO_NINGUNO, medida, t_actual, t_referencia, tasa);
        }
    } else {
        s_monitor.contador_confirmacion = 0;
        s_monitor.contador_normalizacion = 0;
    }

    // Con la anomalía ya avisada, otro salto es la "misma" condición y no escalaría: se compara contra
    // la lectura más baja desde la última revisión para avisar de nuevo si la temperatura sigue subiendo
    if (s_monitor.estado == ESTADO_INCREMENTO && s_monitor.revision_valida && hay_medida &&
        medida_actual - s_monitor.piso_revision_c >= cfg->umbral_salto_c) {
        if (++s_monitor.contador_resalto >= cfg->muestras_confirmacion_salto) {
            ESP_LOGW(TAG, "Nuevo salto tras la revisión: %+.1f °C sobre %.1f °C, estimada %.1f °C",
                     medida_actual - s_monitor.piso_revision_c, s_monitor.piso_revision_c, t_actual);
            publicar_evento(EVENTO_INCREMENTO_ANOMALO, MOTIVO_SALTO_BRUSCO, medida, t_actual, t_referencia, tasa);
            s_monitor.piso_revision_c = medida_actual;  // Para el siguiente aviso debe subir otro umbral
            s_monitor.contador_resalto = 0;
        }
    } else {
        s_monitor.contador_resalto = 0;
    }
}

static void manejar_falla_lectura(esp_err_t error)
{
    s_monitor.fallas_consecutivas++;
    ESP_LOGD(TAG, "Lectura fallida (%s), %" PRIu32 " seguidas", esp_err_to_name(error), s_monitor.fallas_consecutivas);

    // Un esclavo SMBus puede quedarse reteniendo SDA; reiniciar el bus lo libera
    if (s_monitor.fallas_consecutivas % FALLAS_PARA_REINICIAR_BUS == 0) {
        i2c_master_bus_reset(s_monitor.bus);
    }

    if (!s_monitor.sensor_en_falla && s_monitor.fallas_consecutivas >= FALLAS_PARA_ALERTA) {
        s_monitor.sensor_en_falla = true;
        ESP_LOGE(TAG, "Sensor sin respuesta (%s)", esp_err_to_name(error));
        publicar_evento(EVENTO_FALLA_SENSOR, MOTIVO_FALLA_SENSOR, NAN, NAN, NAN, 0.0f);
    }
}

static void manejar_lectura_correcta(void)
{
    if (s_monitor.sensor_en_falla) {
        s_monitor.sensor_en_falla = false;
        ESP_LOGI(TAG, "Sensor recuperado tras %" PRIu32 " lecturas fallidas", s_monitor.fallas_consecutivas);
        publicar_evento(EVENTO_SENSOR_RECUPERADO, MOTIVO_NINGUNO, NAN, NAN, NAN, 0.0f);
    }
    s_monitor.fallas_consecutivas = 0;
}

static void tarea_monitor_temperatura(void *arg)
{
    const TickType_t periodo = pdMS_TO_TICKS(MONITOR_PERIODO_MS);
    TickType_t ultimo_despertar = xTaskGetTickCount();

    for (;;) {
        // DelayUntil mantiene los 100 ms exactos sin acumular el tiempo de lectura
        if (xTaskDelayUntil(&ultimo_despertar, periodo) == pdFALSE) {
            ESP_LOGW(TAG, "Ciclo de muestreo retrasado");
        }

        muestra_temperatura_t muestra = { .marca_us = esp_timer_get_time() };
        esp_err_t ret = mlx90614_leer_temperatura_objeto(s_monitor.sensor, &muestra.temp_objeto_c);
        if (ret == ESP_OK) {
            ret = mlx90614_leer_temperatura_ambiente(s_monitor.sensor, &muestra.temp_ambiente_c);
        }
        if (ret != ESP_OK) {
            manejar_falla_lectura(ret);
            continue;
        }
        manejar_lectura_correcta();

        xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
        buffer_circular_agregar(&s_monitor.buffer, &muestra);
        evaluar_anomalia();
        xSemaphoreGive(s_monitor.mutex);
    }
}

// Petición SMBus: SCL en bajo > 1.44 ms saca al MLX90614 del modo PWM si estuviera activo
static void solicitar_modo_smbus(gpio_num_t pin_sda, gpio_num_t pin_scl)
{
    // Los pines 39-42 arrancan con función JTAG: gpio_config los pasa a GPIO
    const gpio_config_t config_entrada = {
        .pin_bit_mask = (1ULL << pin_sda) | (1ULL << pin_scl),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&config_entrada);
    gpio_set_level(pin_scl, 0);
    gpio_set_direction(pin_scl, GPIO_MODE_OUTPUT_OD);
    esp_rom_delay_us(2000);
    gpio_set_direction(pin_scl, GPIO_MODE_INPUT);
    gpio_reset_pin(pin_sda);
    gpio_reset_pin(pin_scl);
}

esp_err_t monitor_temperatura_iniciar(const monitor_temperatura_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "configuración nula");
    ESP_RETURN_ON_FALSE(!s_monitor.tarea, ESP_ERR_INVALID_STATE, TAG, "el monitor ya está iniciado");
    ESP_RETURN_ON_FALSE(config->ventana_tendencia_s > 0 &&
                        (config->ventana_tendencia_s + 1) * MUESTRAS_POR_SEGUNDO <= MONITOR_TAM_BUFFER,
                        ESP_ERR_INVALID_ARG, TAG, "ventana_tendencia_s debe estar entre 1 y %d", MONITOR_VENTANA_S - 1);
    ESP_RETURN_ON_FALSE(config->ventana_salto_s > 0 &&
                        (config->ventana_salto_s + 1) * MUESTRAS_POR_SEGUNDO <= MONITOR_TAM_BUFFER,
                        ESP_ERR_INVALID_ARG, TAG, "ventana_salto_s debe estar entre 1 y %d", MONITOR_VENTANA_S - 1);
    ESP_RETURN_ON_FALSE(config->muestras_confirmacion > 0 && config->muestras_confirmacion_salto > 0 &&
                        config->muestras_normalizacion > 0,
                        ESP_ERR_INVALID_ARG, TAG, "los contadores de confirmación deben ser > 0");

    s_monitor.config = *config;
    s_monitor.umbral_incremento_fabrica_c = config->umbral_salto_c;
    cargar_umbral_incremento();

    solicitar_modo_smbus(config->pin_sda, config->pin_scl);

    esp_err_t ret = ESP_OK;
    const i2c_master_bus_config_t config_bus = {
        .i2c_port = config->puerto_i2c,
        .sda_io_num = config->pin_sda,
        .scl_io_num = config->pin_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   // Los módulos GY-906 ya traen pull-ups de 10k
    };
    ESP_GOTO_ON_ERROR(i2c_new_master_bus(&config_bus, &s_monitor.bus), error, TAG, "no se pudo crear el bus I2C");

    // Sin respuesta el monitor arranca igual: la tarea reintenta cada 100 ms, publica EVENTO_FALLA_SENSOR
    // (tratado como alerta) y EVENTO_SENSOR_RECUPERADO si el contacto vuelve
    const mlx90614_config_t config_sensor = {
        .direccion = config->direccion_sensor,
        .velocidad_hz = config->velocidad_i2c_hz,
        .permitir_sin_respuesta = true,
    };
    ret = mlx90614_crear(s_monitor.bus, &config_sensor, &s_monitor.sensor);
    const bool sensor_responde = ret == ESP_OK;
    if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "El MLX90614 no responde (revisa 3V3, GND, SDA=%d y SCL=%d): el sistema arranca sin "
                 "temperatura y seguirá reintentando", config->pin_sda, config->pin_scl);
        ret = ESP_OK;
    }
    ESP_GOTO_ON_ERROR(ret, error, TAG, "no se pudo crear el sensor");

    float emisividad;
    if (sensor_responde && mlx90614_leer_emisividad(s_monitor.sensor, &emisividad) == ESP_OK) {
        ESP_LOGI(TAG, "Emisividad configurada: %.3f", emisividad);
    }

    ESP_GOTO_ON_ERROR(buffer_circular_crear(&s_monitor.buffer, MONITOR_TAM_BUFFER), error, TAG,
                      "sin memoria para el buffer");

    calibracion_temperatura_t calibracion = { 0 };
    if (calibracion_cargar(&calibracion) == ESP_OK) {
        ESP_LOGI(TAG, "Calibración cargada: fondo %.2f °C, factor de llenado %.4f%s",
                 calibracion.fondo_c, calibracion.factor_llenado,
                 calibracion.referencia_valida ? "" : " (falta el paso de referencia)");
    } else {
        ESP_LOGW(TAG, "Sin calibración: se usa la lectura cruda del sensor");
    }
    aplicar_calibracion(&calibracion);
    ESP_LOGI(TAG, "Umbral de tasa efectivo: %.1f °C/min", s_monitor.tasa_umbral_c_por_min);

    s_monitor.mutex = xSemaphoreCreateMutex();
    s_monitor.cola_eventos = xQueueCreate(TAM_COLA_EVENTOS, sizeof(evento_temperatura_t));
    ESP_GOTO_ON_FALSE(s_monitor.mutex && s_monitor.cola_eventos, ESP_ERR_NO_MEM, error, TAG,
                      "sin memoria para mutex/cola");

    if (!sensor_responde) {
        // La falla se avisa desde el arranque, sin esperar el primer segundo de reintentos
        s_monitor.sensor_en_falla = true;
        s_monitor.fallas_consecutivas = FALLAS_PARA_ALERTA;
        publicar_evento(EVENTO_FALLA_SENSOR, MOTIVO_FALLA_SENSOR, NAN, NAN, NAN, 0.0f);
    }

    ESP_GOTO_ON_FALSE(xTaskCreatePinnedToCore(tarea_monitor_temperatura, "monitor_temp", TAM_PILA_TAREA, NULL,
                                              config->prioridad_tarea, &s_monitor.tarea, config->nucleo_tarea) == pdPASS,
                      ESP_ERR_NO_MEM, error, TAG, "no se pudo crear la tarea");

    ESP_LOGI(TAG, "Monitor iniciado: %d Hz, ventana de %d s (%d muestras), SDA=%d SCL=%d, I2C a %" PRIu32 " Hz",
             MONITOR_FRECUENCIA_HZ, MONITOR_VENTANA_S, MONITOR_TAM_BUFFER, config->pin_sda, config->pin_scl,
             config->velocidad_i2c_hz);
    return ESP_OK;

error:
    if (s_monitor.cola_eventos) {
        vQueueDelete(s_monitor.cola_eventos);
    }
    if (s_monitor.mutex) {
        vSemaphoreDelete(s_monitor.mutex);
    }
    if (s_monitor.buffer.datos) {
        buffer_circular_liberar(&s_monitor.buffer);
    }
    if (s_monitor.sensor) {
        mlx90614_eliminar(s_monitor.sensor);
    }
    if (s_monitor.bus) {
        i2c_del_master_bus(s_monitor.bus);
    }
    memset(&s_monitor, 0, sizeof(s_monitor));
    return ret;
}

bool monitor_temperatura_esperar_evento(evento_temperatura_t *evento, TickType_t espera)
{
    if (!s_monitor.cola_eventos || !evento) {
        return false;
    }
    return xQueueReceive(s_monitor.cola_eventos, evento, espera) == pdTRUE;
}

bool monitor_temperatura_obtener_ultima_muestra(muestra_temperatura_t *muestra)
{
    if (!s_monitor.mutex || !muestra) {
        return false;
    }
    // Con el sensor en falla la última muestra es vieja: no se entrega como si fuera actual
    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    const bool hay_muestra = !s_monitor.sensor_en_falla && buffer_circular_obtener(&s_monitor.buffer, 0, muestra);
    xSemaphoreGive(s_monitor.mutex);
    return hay_muestra;
}

void monitor_temperatura_marcar_revision(void)
{
    if (!s_monitor.mutex) {
        return;
    }
    float medida;
    float ambiente;
    int64_t marca;
    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    if (buffer_circular_promediar(&s_monitor.buffer, 0, MUESTRAS_POR_SEGUNDO, &medida, &ambiente, &marca)) {
        s_monitor.piso_revision_c = medida;
        s_monitor.revision_valida = true;
        s_monitor.contador_resalto = 0;
    }
    xSemaphoreGive(s_monitor.mutex);
}

bool monitor_temperatura_sensor_ok(void)
{
    return s_monitor.tarea && !s_monitor.sensor_en_falla;
}

size_t monitor_temperatura_copiar_historial(muestra_temperatura_t *destino, size_t maximo)
{
    if (!s_monitor.mutex || !destino) {
        return 0;
    }
    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    const size_t copiadas = buffer_circular_copiar(&s_monitor.buffer, destino, maximo);
    xSemaphoreGive(s_monitor.mutex);
    return copiadas;
}

float monitor_temperatura_estimar(const muestra_temperatura_t *muestra)
{
    if (!s_monitor.mutex || !muestra) {
        return NAN;
    }
    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    const float estimada = calibracion_estimar_temperatura(&s_monitor.calibracion,
                                                           muestra->temp_objeto_c, muestra->temp_ambiente_c);
    xSemaphoreGive(s_monitor.mutex);
    return estimada;
}

bool monitor_temperatura_obtener_estimada(float *temperatura_c)
{
    if (!s_monitor.mutex || !temperatura_c) {
        return false;
    }
    int64_t marca_us;
    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    const bool hay_datos = !s_monitor.sensor_en_falla && promediar_estimada(0, temperatura_c, &marca_us);
    xSemaphoreGive(s_monitor.mutex);
    return hay_datos;
}

void monitor_temperatura_establecer_linea_base(float temperatura_c)
{
    if (!s_monitor.mutex) {
        return;
    }
    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    s_monitor.linea_base_c = temperatura_c;
    s_monitor.linea_base_valida = true;
    xSemaphoreGive(s_monitor.mutex);
    ESP_LOGI(TAG, "Nueva línea base: %.1f °C (anómalo a partir de %.1f °C)",
             temperatura_c, temperatura_c + s_monitor.config.margen_superacion_c);
}

void monitor_temperatura_borrar_linea_base(void)
{
    if (!s_monitor.mutex) {
        return;
    }
    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    s_monitor.linea_base_valida = false;
    xSemaphoreGive(s_monitor.mutex);
}

bool monitor_temperatura_obtener_linea_base(float *temperatura_c)
{
    if (!s_monitor.mutex || !temperatura_c) {
        return false;
    }
    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    const bool valida = s_monitor.linea_base_valida;
    *temperatura_c = s_monitor.linea_base_c;
    xSemaphoreGive(s_monitor.mutex);
    return valida;
}

bool monitor_temperatura_obtener_calibracion(calibracion_temperatura_t *cal)
{
    if (!s_monitor.mutex || !cal) {
        return false;
    }
    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    *cal = s_monitor.calibracion;
    xSemaphoreGive(s_monitor.mutex);
    return true;
}

typedef esp_err_t (*calculo_calibracion_t)(const muestra_temperatura_t *muestras, size_t cantidad,
                                           calibracion_temperatura_t *cal, void *contexto);

// Deja correr el muestreo normal durante `segundos`, toma esa ventana del buffer y aplica el cálculo
static esp_err_t ejecutar_calibracion(uint32_t segundos, calculo_calibracion_t calculo, void *contexto)
{
    ESP_RETURN_ON_FALSE(s_monitor.tarea, ESP_ERR_INVALID_STATE, TAG, "el monitor no está iniciado");

    const size_t maximo = (size_t)segundos * MUESTRAS_POR_SEGUNDO;
    muestra_temperatura_t *muestras = malloc(maximo * sizeof(*muestras));
    ESP_RETURN_ON_FALSE(muestras, ESP_ERR_NO_MEM, TAG, "sin memoria para la calibración");

    for (uint32_t restante = segundos; restante > 0; restante -= (restante >= 10 ? 10 : restante)) {
        ESP_LOGI(TAG, "  calibrando... %" PRIu32 " s restantes", restante);
        vTaskDelay(pdMS_TO_TICKS((restante >= 10 ? 10 : restante) * 1000));
    }

    const size_t cantidad = monitor_temperatura_copiar_historial(muestras, maximo);

    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    calibracion_temperatura_t cal = s_monitor.calibracion;
    xSemaphoreGive(s_monitor.mutex);

    esp_err_t ret = calculo(muestras, cantidad, &cal, contexto);
    free(muestras);
    if (ret != ESP_OK) {
        return ret;
    }

    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    aplicar_calibracion(&cal);
    const float tasa_umbral = s_monitor.tasa_umbral_c_por_min;
    xSemaphoreGive(s_monitor.mutex);
    ESP_LOGI(TAG, "Umbral de tasa efectivo: %.1f °C/min", tasa_umbral);

    ret = calibracion_guardar(&cal);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Calibración aplicada pero no guardada en NVS (%s)", esp_err_to_name(ret));
    }
    return ESP_OK;
}

static esp_err_t calculo_fondo(const muestra_temperatura_t *muestras, size_t cantidad,
                               calibracion_temperatura_t *cal, void *contexto)
{
    return calibracion_calcular_fondo(muestras, cantidad, MUESTRAS_POR_SEGUNDO, cal);
}

static esp_err_t calculo_referencia(const muestra_temperatura_t *muestras, size_t cantidad,
                                    calibracion_temperatura_t *cal, void *contexto)
{
    const float temperatura_referencia_c = *(const float *)contexto;
    return calibracion_calcular_referencia(muestras, cantidad, MUESTRAS_POR_SEGUNDO, temperatura_referencia_c, cal);
}

esp_err_t monitor_temperatura_calibrar_fondo(void)
{
    ESP_LOGI(TAG, "CALIBRACIÓN DE FONDO (%d s): estufa APAGADA y nadie frente al sensor", DURACION_CAL_FONDO_S);
    return ejecutar_calibracion(DURACION_CAL_FONDO_S, calculo_fondo, NULL);
}

esp_err_t monitor_temperatura_calibrar_referencia(float temperatura_referencia_c)
{
    ESP_LOGI(TAG, "CALIBRACIÓN DE REFERENCIA (%d s): recipiente estable a %.1f °C, nadie frente al sensor",
             DURACION_CAL_REFERENCIA_S, temperatura_referencia_c);
    return ejecutar_calibracion(DURACION_CAL_REFERENCIA_S, calculo_referencia, &temperatura_referencia_c);
}

esp_err_t monitor_temperatura_borrar_calibracion(void)
{
    ESP_RETURN_ON_FALSE(s_monitor.mutex, ESP_ERR_INVALID_STATE, TAG, "el monitor no está iniciado");

    const calibracion_temperatura_t vacia = { 0 };
    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    aplicar_calibracion(&vacia);
    aplicar_umbral_incremento(s_monitor.umbral_incremento_fabrica_c);
    s_monitor.umbral_incremento_calibrado = false;
    xSemaphoreGive(s_monitor.mutex);

    nvs_handle_t nvs;
    if (nvs_open(NVS_ESPACIO_UMBRAL, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_key(nvs, NVS_CLAVE_UMBRAL);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "Calibración borrada: lectura cruda del sensor y umbral de incremento de fábrica (%.1f °C)",
             s_monitor.umbral_incremento_fabrica_c);
    return calibracion_borrar();
}

bool monitor_temperatura_obtener_medida(float *temperatura_c)
{
    if (!s_monitor.mutex || !temperatura_c) {
        return false;
    }
    float ambiente;
    int64_t marca;
    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    const bool hay_datos = !s_monitor.sensor_en_falla &&
                           buffer_circular_promediar(&s_monitor.buffer, 0, MUESTRAS_POR_SEGUNDO,
                                                     temperatura_c, &ambiente, &marca);
    xSemaphoreGive(s_monitor.mutex);
    return hay_datos;
}

esp_err_t monitor_temperatura_establecer_umbral_incremento(float umbral_c)
{
    ESP_RETURN_ON_FALSE(s_monitor.mutex, ESP_ERR_INVALID_STATE, TAG, "el monitor no está iniciado");
    ESP_RETURN_ON_FALSE(umbral_c >= UMBRAL_INCREMENTO_MIN_C && umbral_c <= UMBRAL_INCREMENTO_MAX_C,
                        ESP_ERR_INVALID_ARG, TAG, "umbral fuera de rango");

    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_ESPACIO_UMBRAL, NVS_READWRITE, &nvs), TAG, "NVS");
    esp_err_t ret = nvs_set_blob(nvs, NVS_CLAVE_UMBRAL, &umbral_c, sizeof(umbral_c));
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs);
    }
    nvs_close(nvs);
    ESP_RETURN_ON_ERROR(ret, TAG, "no se pudo guardar el umbral");

    xSemaphoreTake(s_monitor.mutex, portMAX_DELAY);
    aplicar_umbral_incremento(umbral_c);
    s_monitor.umbral_incremento_calibrado = true;
    xSemaphoreGive(s_monitor.mutex);
    ESP_LOGI(TAG, "Umbral de incremento: %.1f °C (salto en %u s y superación de la línea base)",
             umbral_c, s_monitor.config.ventana_salto_s);
    return ESP_OK;
}

float monitor_temperatura_obtener_umbral_incremento(bool *calibrado)
{
    if (calibrado) {
        *calibrado = s_monitor.umbral_incremento_calibrado;
    }
    return s_monitor.config.umbral_salto_c;
}

const char *monitor_temperatura_nombre_motivo(motivo_anomalia_t motivo)
{
    switch (motivo) {
    case MOTIVO_SALTO_BRUSCO:       return "salto_brusco";
    case MOTIVO_SUPERO_LINEA_BASE:  return "supero_linea_base";
    case MOTIVO_SUBIDA_SOSTENIDA:   return "subida_sostenida";
    case MOTIVO_CRITICA:            return "critica";
    case MOTIVO_FALLA_SENSOR:       return "falla_sensor";
    case MOTIVO_NINGUNO:            break;
    }
    return "ninguno";
}
