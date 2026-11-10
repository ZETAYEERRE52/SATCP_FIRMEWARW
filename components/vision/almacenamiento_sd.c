#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "nvs.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "almacenamiento_sd.h"

static const char *TAG = "SD";

#define NVS_ESPACIO         "satcp"
#define NVS_CLAVE_ARRANQUE  "arranques"
#define MAX_RUTA            128

static struct {
    sdmmc_card_t *tarjeta;
    SemaphoreHandle_t mutex;
    uint32_t sesion;            // Número de arranque, para no sobrescribir entre reinicios
    uint32_t contador;
} s_sd;

static uint32_t incrementar_contador_arranques(void)
{
    nvs_handle_t nvs;
    uint32_t arranques = 0;
    if (nvs_open(NVS_ESPACIO, NVS_READWRITE, &nvs) != ESP_OK) {
        return 0;
    }
    nvs_get_u32(nvs, NVS_CLAVE_ARRANQUE, &arranques);
    arranques++;
    nvs_set_u32(nvs, NVS_CLAVE_ARRANQUE, arranques);
    nvs_commit(nvs);
    nvs_close(nvs);
    return arranques;
}

static esp_err_t crear_carpeta(const char *ruta)
{
    if (mkdir(ruta, 0775) != 0 && errno != EEXIST) {
        ESP_LOGE(TAG, "No se pudo crear %s (errno %d)", ruta, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t sd_iniciar(const sd_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "configuración nula");
    ESP_RETURN_ON_FALSE(!s_sd.tarjeta, ESP_ERR_INVALID_STATE, TAG, "la SD ya está montada");

    const esp_vfs_fat_mount_config_t config_montaje = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t ranura = SDMMC_SLOT_CONFIG_DEFAULT();
    ranura.width = 1;
    ranura.clk = config->pin_clk;
    ranura.cmd = config->pin_cmd;
    ranura.d0 = config->pin_d0;
    ranura.d1 = GPIO_NUM_NC;
    ranura.d2 = GPIO_NUM_NC;
    ranura.d3 = GPIO_NUM_NC;
    ranura.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_PUNTO_MONTAJE, &host, &ranura, &config_montaje, &s_sd.tarjeta);
    if (ret != ESP_OK) {
        s_sd.tarjeta = NULL;
        ESP_LOGE(TAG, "No se pudo montar la SD (%s): ¿tarjeta insertada y en FAT32?", esp_err_to_name(ret));
        return ret;
    }
    sdmmc_card_print_info(stdout, s_sd.tarjeta);

    s_sd.mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_sd.mutex, ESP_ERR_NO_MEM, TAG, "sin memoria");
    s_sd.sesion = incrementar_contador_arranques();
    s_sd.contador = 0;

    ESP_LOGI(TAG, "SD montada en %s, sesión s%04" PRIu32, SD_PUNTO_MONTAJE, s_sd.sesion);
    return ESP_OK;
}

bool sd_disponible(void)
{
    return s_sd.tarjeta != NULL;
}

esp_err_t sd_guardar_imagen(const char *carpeta, const char *etiqueta, const uint8_t *datos, size_t longitud,
                            char *ruta, size_t tam_ruta)
{
    ESP_RETURN_ON_FALSE(s_sd.tarjeta, ESP_ERR_INVALID_STATE, TAG, "SD no montada");
    ESP_RETURN_ON_FALSE(carpeta && etiqueta && datos && longitud > 0, ESP_ERR_INVALID_ARG, TAG, "argumentos inválidos");

    char directorio[MAX_RUTA];
    char archivo[MAX_RUTA];
    esp_err_t ret = ESP_OK;

    xSemaphoreTake(s_sd.mutex, portMAX_DELAY);

    int largo = snprintf(directorio, sizeof(directorio), "%s/%s", SD_PUNTO_MONTAJE, carpeta);
    ESP_GOTO_ON_FALSE(largo > 0 && largo < (int)sizeof(directorio), ESP_ERR_INVALID_SIZE, fin, TAG, "ruta muy larga");
    ESP_GOTO_ON_ERROR(crear_carpeta(directorio), fin, TAG, "carpeta base");
    largo = snprintf(directorio, sizeof(directorio), "%s/%s/s%04" PRIu32, SD_PUNTO_MONTAJE, carpeta, s_sd.sesion);
    ESP_GOTO_ON_FALSE(largo > 0 && largo < (int)sizeof(directorio), ESP_ERR_INVALID_SIZE, fin, TAG, "ruta muy larga");
    ESP_GOTO_ON_ERROR(crear_carpeta(directorio), fin, TAG, "carpeta de sesión");

    largo = snprintf(archivo, sizeof(archivo), "%s/%s_%06" PRIu32 ".jpg", directorio, etiqueta, ++s_sd.contador);
    ESP_GOTO_ON_FALSE(largo > 0 && largo < (int)sizeof(archivo), ESP_ERR_INVALID_SIZE, fin, TAG, "ruta muy larga");
    FILE *f = fopen(archivo, "wb");
    ESP_GOTO_ON_FALSE(f, ESP_FAIL, fin, TAG, "no se pudo abrir %s", archivo);
    const size_t escritos = fwrite(datos, 1, longitud, f);
    fclose(f);
    ESP_GOTO_ON_FALSE(escritos == longitud, ESP_FAIL, fin, TAG, "escritura incompleta en %s", archivo);

    if (ruta) {
        strlcpy(ruta, archivo, tam_ruta);
    }

fin:
    xSemaphoreGive(s_sd.mutex);
    return ret;
}
