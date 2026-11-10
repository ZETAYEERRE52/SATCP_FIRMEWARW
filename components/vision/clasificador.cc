#include <cmath>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "jpeg_decoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "clasificador.h"
#include "modelo_fuego.h"

static const char *TAG = "CLASIFICADOR";

namespace {

constexpr int TAM_ENTRADA = 96;                     // parametros.json: tam_imagen
constexpr int CANALES = 3;
constexpr size_t TAM_ARENA = 512 * 1024;            // MobileNetV2 0.35 usa bastante menos; se informa al iniciar
constexpr size_t TAM_MAX_RGB = 640 * 480 * CANALES; // Cuadro VGA completo, por si falla la decodificación reducida

tflite::MicroInterpreter *s_interprete;
uint8_t *s_rgb;
SemaphoreHandle_t s_mutex;      // La fase visual y los pedidos del teléfono comparten el modelo

// Redimensiona con interpolación bilineal (centros de píxel, igual que tf.image.resize del entrenamiento)
// y cuantiza directamente al tensor de entrada int8
void preparar_entrada(const uint8_t *rgb, int ancho, int alto, TfLiteTensor *entrada)
{
    const float escala = entrada->params.scale;
    const int cero = entrada->params.zero_point;
    const float paso_x = (float)ancho / TAM_ENTRADA;
    const float paso_y = (float)alto / TAM_ENTRADA;
    int8_t *destino = entrada->data.int8;

    for (int y = 0; y < TAM_ENTRADA; y++) {
        const float fy = fmaxf((y + 0.5f) * paso_y - 0.5f, 0.0f);
        const int y0 = (int)fy;
        const int y1 = y0 + 1 < alto ? y0 + 1 : alto - 1;
        const float dy = fy - y0;
        for (int x = 0; x < TAM_ENTRADA; x++) {
            const float fx = fmaxf((x + 0.5f) * paso_x - 0.5f, 0.0f);
            const int x0 = (int)fx;
            const int x1 = x0 + 1 < ancho ? x0 + 1 : ancho - 1;
            const float dx = fx - x0;
            for (int c = 0; c < CANALES; c++) {
                const float p00 = rgb[(y0 * ancho + x0) * CANALES + c];
                const float p01 = rgb[(y0 * ancho + x1) * CANALES + c];
                const float p10 = rgb[(y1 * ancho + x0) * CANALES + c];
                const float p11 = rgb[(y1 * ancho + x1) * CANALES + c];
                const float valor = (p00 * (1 - dx) + p01 * dx) * (1 - dy) + (p10 * (1 - dx) + p11 * dx) * dy;
                const int q = (int)lroundf(valor / escala) + cero;
                *destino++ = (int8_t)(q < -128 ? -128 : (q > 127 ? 127 : q));
            }
        }
    }
}

esp_err_t decodificar_jpeg(const uint8_t *jpeg, size_t longitud, esp_jpeg_image_scale_t reduccion,
                           esp_jpeg_image_output_t *imagen)
{
    esp_jpeg_image_cfg_t config = {};
    config.indata = const_cast<uint8_t *>(jpeg);
    config.indata_size = longitud;
    config.outbuf = s_rgb;
    config.outbuf_size = TAM_MAX_RGB;
    config.out_format = JPEG_IMAGE_FORMAT_RGB888;
    config.out_scale = reduccion;
    return esp_jpeg_decode(&config, imagen);
}

}  // namespace

extern "C" esp_err_t clasificador_iniciar(void)
{
    ESP_RETURN_ON_FALSE(!s_interprete, ESP_ERR_INVALID_STATE, TAG, "el clasificador ya está iniciado");

    const tflite::Model *modelo = tflite::GetModel(modelo_fuego);
    ESP_RETURN_ON_FALSE(modelo->version() == TFLITE_SCHEMA_VERSION, ESP_FAIL, TAG,
                        "versión de esquema del modelo incompatible (%lu)", (unsigned long)modelo->version());

    uint8_t *arena = static_cast<uint8_t *>(heap_caps_aligned_alloc(16, TAM_ARENA, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    s_rgb = static_cast<uint8_t *>(heap_caps_malloc(TAM_MAX_RGB, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!arena || !s_rgb) {
        heap_caps_free(arena);
        heap_caps_free(s_rgb);
        s_rgb = nullptr;
        ESP_LOGE(TAG, "Sin PSRAM para el modelo (¿PSRAM activada en sdkconfig?)");
        return ESP_ERR_NO_MEM;
    }

    // Operaciones de MobileNetV2 int8; si el modelo pide otra, AllocateTensors lo indica
    static tflite::MicroMutableOpResolver<14> resolutor;
    resolutor.AddConv2D();
    resolutor.AddDepthwiseConv2D();
    resolutor.AddAdd();
    resolutor.AddMul();
    resolutor.AddSub();
    resolutor.AddPad();
    resolutor.AddMean();
    resolutor.AddAveragePool2D();
    resolutor.AddFullyConnected();
    resolutor.AddSoftmax();
    resolutor.AddReshape();
    resolutor.AddQuantize();
    resolutor.AddDequantize();
    resolutor.AddRelu6();

    static tflite::MicroInterpreter interprete(modelo, resolutor, arena, TAM_ARENA);
    ESP_RETURN_ON_FALSE(interprete.AllocateTensors() == kTfLiteOk, ESP_FAIL, TAG, "no se pudieron asignar los tensores");

    const TfLiteTensor *entrada = interprete.input(0);
    const TfLiteTensor *salida = interprete.output(0);
    ESP_RETURN_ON_FALSE(entrada->type == kTfLiteInt8 && entrada->dims->size == 4 &&
                        entrada->dims->data[1] == TAM_ENTRADA && entrada->dims->data[2] == TAM_ENTRADA &&
                        entrada->dims->data[3] == CANALES,
                        ESP_FAIL, TAG, "la entrada del modelo no es int8 %dx%dx%d", TAM_ENTRADA, TAM_ENTRADA, CANALES);
    ESP_RETURN_ON_FALSE(salida->type == kTfLiteInt8 && salida->dims->data[salida->dims->size - 1] == NUM_CLASES,
                        ESP_FAIL, TAG, "la salida del modelo no tiene %d clases int8", NUM_CLASES);

    s_mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_mutex, ESP_ERR_NO_MEM, TAG, "sin memoria para el mutex");
    s_interprete = &interprete;
    ESP_LOGI(TAG, "Modelo cargado: %u KB, memoria de trabajo usada %u KB de %u KB",
             modelo_fuego_len / 1024, (unsigned)(interprete.arena_used_bytes() / 1024), (unsigned)(TAM_ARENA / 1024));
    return ESP_OK;
}

extern "C" bool clasificador_disponible(void)
{
    return s_interprete != nullptr;
}

extern "C" esp_err_t clasificador_clasificar_jpeg(const uint8_t *jpeg, size_t longitud,
                                                  resultado_clasificacion_t *resultado)
{
    ESP_RETURN_ON_FALSE(s_interprete, ESP_ERR_INVALID_STATE, TAG, "clasificador no iniciado");
    ESP_RETURN_ON_FALSE(jpeg && longitud > 0 && resultado, ESP_ERR_INVALID_ARG, TAG, "argumentos inválidos");

    const int64_t inicio = esp_timer_get_time();
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    // Se decodifica reducido (160x120 tanto desde VGA como desde QVGA de la vista en vivo): es mucho
    // más rápido y sobra para una entrada de 96x96, sin quedar por debajo de ella
    esp_jpeg_image_cfg_t config_info = {};
    config_info.indata = const_cast<uint8_t *>(jpeg);
    config_info.indata_size = longitud;
    esp_jpeg_image_output_t imagen = {};
    esp_jpeg_image_scale_t reduccion = JPEG_IMAGE_SCALE_1_4;
    if (esp_jpeg_get_image_info(&config_info, &imagen) == ESP_OK) {
        if (imagen.width < TAM_ENTRADA * 4 || imagen.height < TAM_ENTRADA * 4) {
            reduccion = imagen.width >= TAM_ENTRADA * 2 && imagen.height >= TAM_ENTRADA * 2
                        ? JPEG_IMAGE_SCALE_1_2 : JPEG_IMAGE_SCALE_0;
        }
    }
    esp_err_t ret = decodificar_jpeg(jpeg, longitud, reduccion, &imagen);
    if (ret != ESP_OK) {
        ret = decodificar_jpeg(jpeg, longitud, JPEG_IMAGE_SCALE_0, &imagen);
    }
    if (ret != ESP_OK) {
        xSemaphoreGive(s_mutex);
        ESP_LOGE(TAG, "no se pudo decodificar el JPEG (%s)", esp_err_to_name(ret));
        return ret;
    }

    TfLiteTensor *entrada = s_interprete->input(0);
    preparar_entrada(s_rgb, imagen.width, imagen.height, entrada);
    const bool inferencia_ok = s_interprete->Invoke() == kTfLiteOk;
    if (!inferencia_ok) {
        xSemaphoreGive(s_mutex);
        ESP_LOGE(TAG, "falló la inferencia");
        return ESP_FAIL;
    }

    const TfLiteTensor *salida = s_interprete->output(0);
    int mejor = 0;
    for (int i = 0; i < NUM_CLASES; i++) {
        resultado->probabilidad[i] = (salida->data.int8[i] - salida->params.zero_point) * salida->params.scale;
        if (resultado->probabilidad[i] > resultado->probabilidad[mejor]) {
            mejor = i;
        }
    }
    xSemaphoreGive(s_mutex);
    // La llama ocupa una parte pequeña de la imagen de la estufa: basta un poco de "fuego" para contarlo
    if (resultado->probabilidad[CLASE_FUEGO] >= CLASIFICADOR_UMBRAL_FUEGO) {
        mejor = CLASE_FUEGO;
    }
    resultado->clase = static_cast<clase_vision_t>(mejor);
    resultado->tiempo_ms = (uint32_t)((esp_timer_get_time() - inicio) / 1000);
    return ESP_OK;
}

extern "C" const char *clasificador_nombre_clase(clase_vision_t clase)
{
    switch (clase) {
    case CLASE_FUEGO:  return "fuego";
    case CLASE_HUMO:   return "humo";
    case CLASE_NORMAL: return "normal";
    default:           return "desconocida";
    }
}
