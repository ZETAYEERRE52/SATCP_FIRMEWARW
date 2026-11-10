#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_satcp.h"
#include "comunicaciones_interno.h"

static const char *TAG = "BLE_APP";

// Si se agotan los búferes de NimBLE se espera a que se vacíen: hasta 5 s por fragmento. La espera es de
// un tick (10 ms con CONFIG_FREERTOS_HZ=100): con 5 ms, pdMS_TO_TICKS daba 0 y no se esperaba nada
#define REINTENTOS_SIN_MEMORIA  500
#define ESPERA_SIN_MEMORIA_TICKS 1
// Durante la foto se pide un intervalo corto para enviarla en pocos segundos; luego se vuelve al normal
#define FOTO_ITVL_MIN_MS        15
#define FOTO_ITVL_MAX_MS        30
// La radio se reparte entre Wi-Fi y Bluetooth: anunciar sin pausa le quita tiempo al Wi-Fi.
// Con 200-300 ms el teléfono sigue encontrando el equipo en uno o dos segundos
#define ANUNCIO_MIN_MS          200
#define ANUNCIO_MAX_MS          300
// Cada evento de conexión ocupa la radio que comparte con el Wi-Fi: con la app conectada por Bluetooth
// el ping al Wi-Fi pasó de 87 ms y 0 % de pérdida a 444 ms y 34 %. La latencia de periférico deja que el
// ESP32 se salte hasta 4 eventos seguidos cuando no tiene nada que enviar (unos 0,6 s), sin frenar lo
// que sí envía. La conexión aguanta 6 s sin paquetes antes de cortarse
#define CONEXION_ITVL_MIN_MS    100
#define CONEXION_ITVL_MAX_MS    150
#define CONEXION_LATENCIA       4
#define CONEXION_SUPERVISION_MS 6000

// Servicio UART de Nordic: 6E40000x-B5A3-F393-E0A9-E50E24DCCA9E (bytes en orden inverso)
#define UUID_NUS(x) BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                                     0x93, 0xF3, 0xA3, 0xB5, (x), 0x00, 0x40, 0x6E)
// Servicio de imagen de SATCP: 8D6F000x-3C4E-4B1A-9F5E-5A7C0A7E5A01
#define UUID_IMAGEN(x) BLE_UUID128_INIT(0x01, 0x5A, 0x7E, 0x0A, 0x7C, 0x5A, 0x5E, 0x9F, \
                                        0x1A, 0x4B, 0x4E, 0x3C, (x), 0x00, 0x6F, 0x8D)

static const ble_uuid128_t UUID_SERVICIO_UART = UUID_NUS(0x01);
static const ble_uuid128_t UUID_RX = UUID_NUS(0x02);              // La app escribe aquí
static const ble_uuid128_t UUID_TX = UUID_NUS(0x03);              // El ESP32 notifica aquí
static const ble_uuid128_t UUID_SERVICIO_IMAGEN = UUID_IMAGEN(0x01);
static const ble_uuid128_t UUID_DATOS_IMAGEN = UUID_IMAGEN(0x02);

static struct {
    volatile uint16_t conexion;
    uint16_t handle_tx;
    uint16_t handle_imagen;
    volatile bool suscrito_tx;
    volatile bool suscrito_imagen;
    uint8_t tipo_direccion;
    char rx[TAM_MAX_MENSAJE];       // Mensaje en reensamblado (puede llegar en varias escrituras)
    size_t largo_rx;
    volatile bool sincronizado;     // La pila ya puede anunciar
    volatile bool anuncio_pausado;  // Hay una app por Wi-Fi: no se anuncia para no quitarle radio
} s_ble = {
    .conexion = BLE_HS_CONN_HANDLE_NONE,
};

static void anunciar(void);

bool ble_satcp_conectado(void)
{
    return s_ble.conexion != BLE_HS_CONN_HANDLE_NONE;
}

// Notifica un búfer partido según el MTU. Devuelve false si se cortó la conexión
static bool notificar(uint16_t handle, const uint8_t *datos, size_t largo)
{
    const uint16_t conexion = s_ble.conexion;
    if (conexion == BLE_HS_CONN_HANDLE_NONE) {
        return false;
    }
    const uint16_t mtu = ble_att_mtu(conexion);
    const size_t fragmento = mtu > 3 ? mtu - 3 : 20;

    for (size_t enviado = 0; enviado < largo;) {
        const size_t n = largo - enviado < fragmento ? largo - enviado : fragmento;
        int rc = BLE_HS_ENOMEM;
        for (int intento = 0; intento < REINTENTOS_SIN_MEMORIA && rc == BLE_HS_ENOMEM; intento++) {
            struct os_mbuf *om = ble_hs_mbuf_from_flat(datos + enviado, n);
            rc = om ? ble_gatts_notify_custom(conexion, handle, om) : BLE_HS_ENOMEM;
            if (rc == BLE_HS_ENOMEM) {
                vTaskDelay(ESPERA_SIN_MEMORIA_TICKS);
            }
        }
        if (rc != 0) {
            ESP_LOGW(TAG, "Notificación interrumpida (%d)", rc);
            return false;
        }
        enviado += n;
    }
    return true;
}

// Normal: intervalo largo con latencia, para dejarle la radio al Wi-Fi. Rápido: solo mientras va la foto
static void pedir_parametros(bool rapido)
{
    if (s_ble.conexion == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    const struct ble_gap_upd_params parametros = {
        .itvl_min = BLE_GAP_CONN_ITVL_MS(rapido ? FOTO_ITVL_MIN_MS : CONEXION_ITVL_MIN_MS),
        .itvl_max = BLE_GAP_CONN_ITVL_MS(rapido ? FOTO_ITVL_MAX_MS : CONEXION_ITVL_MAX_MS),
        .latency = rapido ? 0 : CONEXION_LATENCIA,
        .supervision_timeout = BLE_GAP_SUPERVISION_TIMEOUT_MS(CONEXION_SUPERVISION_MS),
    };
    const int rc = ble_gap_update_params(s_ble.conexion, &parametros);
    if (rc != 0) {
        ESP_LOGW(TAG, "No se pudieron pedir los parámetros de conexión (%d)", rc);
    }
}

void ble_satcp_enviar_json(const char *json)
{
    if (!json || !s_ble.suscrito_tx) {
        return;
    }
    if (notificar(s_ble.handle_tx, (const uint8_t *)json, strlen(json))) {
        notificar(s_ble.handle_tx, (const uint8_t *)"\n", 1);
    }
}

void ble_satcp_enviar_imagen(int32_t id, const uint8_t *datos, size_t bytes, void (*enviar_evento)(const char *json))
{
    char evento[160];
    if (!s_ble.suscrito_imagen) {
        snprintf(evento, sizeof(evento),
                 "{\"tipo\":\"evento\",\"evento\":\"imagen_error\",\"id\":%" PRId32 ",\"error\":\"sin_suscripcion\"}", id);
        enviar_evento(evento);
        return;
    }

    const uint16_t mtu = s_ble.conexion != BLE_HS_CONN_HANDLE_NONE ? ble_att_mtu(s_ble.conexion) : 23;
    snprintf(evento, sizeof(evento),
             "{\"tipo\":\"evento\",\"evento\":\"imagen_inicio\",\"id\":%" PRId32 ",\"bytes\":%u,\"crc32\":%" PRIu32
             ",\"fragmento\":%u}",
             id, (unsigned)bytes, esp_rom_crc32_le(0, datos, bytes), (unsigned)(mtu - 3));
    enviar_evento(evento);

    const int64_t inicio = esp_timer_get_time();
    pedir_parametros(true);
    const bool completo = notificar(s_ble.handle_imagen, datos, bytes);
    pedir_parametros(false);
    const uint32_t ms = (uint32_t)((esp_timer_get_time() - inicio) / 1000);

    snprintf(evento, sizeof(evento),
             "{\"tipo\":\"evento\",\"evento\":\"%s\",\"id\":%" PRId32 "}", completo ? "imagen_fin" : "imagen_error", id);
    enviar_evento(evento);
    ESP_LOGI(TAG, "Imagen de %u bytes enviada en %" PRIu32 " ms%s", (unsigned)bytes, ms, completo ? "" : " (incompleta)");
}

// ¿El búfer ya tiene un objeto JSON completo? Cuenta llaves fuera de las cadenas
static bool json_completo(const char *texto, size_t largo)
{
    int profundidad = 0;
    bool en_cadena = false;
    bool escapado = false;
    bool empezo = false;
    for (size_t i = 0; i < largo; i++) {
        const char c = texto[i];
        if (en_cadena) {
            if (escapado) {
                escapado = false;
            } else if (c == '\\') {
                escapado = true;
            } else if (c == '"') {
                en_cadena = false;
            }
        } else if (c == '"') {
            en_cadena = true;
        } else if (c == '{') {
            profundidad++;
            empezo = true;
        } else if (c == '}') {
            profundidad--;
        }
    }
    return empezo && profundidad == 0;
}

static void entregar_rx(void)
{
    size_t largo = s_ble.largo_rx;
    while (largo > 0 && (s_ble.rx[largo - 1] == '\n' || s_ble.rx[largo - 1] == '\r')) {
        largo--;
    }
    if (largo > 0) {
        s_ble.rx[largo] = '\0';
        comunicaciones_recibir(s_ble.rx, largo, TRANSPORTE_BLE, 0);
    }
    s_ble.largo_rx = 0;
}

// Un mensaje termina con '\n' o cuando el objeto JSON queda completo, aunque llegue partido
static void recibir_fragmento(const uint8_t *datos, size_t largo)
{
    for (size_t i = 0; i < largo; i++) {
        if (s_ble.largo_rx >= sizeof(s_ble.rx) - 1) {
            ESP_LOGW(TAG, "Mensaje demasiado largo, descartado");
            s_ble.largo_rx = 0;
        }
        s_ble.rx[s_ble.largo_rx++] = (char)datos[i];
        if (datos[i] == '\n') {
            entregar_rx();
        }
    }
    if (s_ble.largo_rx > 0 && json_completo(s_ble.rx, s_ble.largo_rx)) {
        entregar_rx();
    }
}

static int acceso_gatt(uint16_t conexion, uint16_t atributo, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    uint8_t datos[BLE_ATT_ATTR_MAX_LEN];    // Una escritura trae como mucho un atributo (512 bytes)
    uint16_t largo = 0;
    if (OS_MBUF_PKTLEN(ctxt->om) > sizeof(datos) ||
        ble_hs_mbuf_to_flat(ctxt->om, datos, sizeof(datos), &largo) != 0) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    recibir_fragmento(datos, largo);
    return 0;
}

static int sin_lectura(uint16_t conexion, uint16_t atributo, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

static const struct ble_gatt_svc_def SERVICIOS[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_SERVICIO_UART.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &UUID_RX.u,
                .access_cb = acceso_gatt,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &UUID_TX.u,
                .access_cb = sin_lectura,
                .val_handle = &s_ble.handle_tx,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_SERVICIO_IMAGEN.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &UUID_DATOS_IMAGEN.u,
                .access_cb = sin_lectura,
                .val_handle = &s_ble.handle_imagen,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    { 0 },
};

static int evento_gap(struct ble_gap_event *evento, void *arg)
{
    switch (evento->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (evento->connect.status == 0) {
            s_ble.conexion = evento->connect.conn_handle;
            s_ble.largo_rx = 0;
            comunicaciones_cliente_desconectado(TRANSPORTE_BLE, 0);     // Ninguna sesión de una conexión anterior
            ESP_LOGI(TAG, "App conectada por Bluetooth");
            pedir_parametros(false);
        } else {
            anunciar();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        // 0x208 = tiempo de supervisión agotado (se perdió la señal); 0x213 = la app cerró la conexión
        ESP_LOGW(TAG, "App desconectada del Bluetooth (motivo 0x%03X)", evento->disconnect.reason);
        comunicaciones_cliente_desconectado(TRANSPORTE_BLE, 0);
        s_ble.conexion = BLE_HS_CONN_HANDLE_NONE;
        s_ble.suscrito_tx = false;
        s_ble.suscrito_imagen = false;
        anunciar();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        anunciar();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (evento->subscribe.attr_handle == s_ble.handle_tx) {
            s_ble.suscrito_tx = evento->subscribe.cur_notify;
            if (s_ble.suscrito_tx) {
                comunicaciones_forzar_estado();
            }
        } else if (evento->subscribe.attr_handle == s_ble.handle_imagen) {
            s_ble.suscrito_imagen = evento->subscribe.cur_notify;
        }
        break;
    case BLE_GAP_EVENT_CONN_UPDATE:
        if (evento->conn_update.status == 0) {
            struct ble_gap_conn_desc descripcion;
            if (ble_gap_conn_find(evento->conn_update.conn_handle, &descripcion) == 0) {
                ESP_LOGI(TAG, "Conexión: intervalo %.2f ms, latencia %d, supervisión %d ms",
                         descripcion.conn_itvl * 1.25f, descripcion.conn_latency, descripcion.supervision_timeout * 10);
            }
        }
        break;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU negociado: %d", evento->mtu.value);
        break;
    default:
        break;
    }
    return 0;
}

static void anunciar(void)
{
    if (s_ble.anuncio_pausado) {
        return;
    }
    // El UUID de 128 bits del servicio UART permite a la app filtrar el escaneo por servicio
    struct ble_hs_adv_fields campos = { 0 };
    campos.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    campos.name = (const uint8_t *)BLE_NOMBRE_DISPOSITIVO;
    campos.name_len = sizeof(BLE_NOMBRE_DISPOSITIVO) - 1;
    campos.name_is_complete = 1;
    campos.uuids128 = &UUID_SERVICIO_UART;
    campos.num_uuids128 = 1;
    campos.uuids128_is_complete = 0;    // Hay un segundo servicio (imagen) que no cabe en el anuncio
    int rc = ble_gap_adv_set_fields(&campos);
    if (rc != 0) {
        ESP_LOGE(TAG, "No se pudo configurar el anuncio (%d)", rc);
        return;
    }

    struct ble_gap_adv_params parametros = { 0 };
    parametros.conn_mode = BLE_GAP_CONN_MODE_UND;
    parametros.disc_mode = BLE_GAP_DISC_MODE_GEN;
    parametros.itvl_min = BLE_GAP_ADV_ITVL_MS(ANUNCIO_MIN_MS);
    parametros.itvl_max = BLE_GAP_ADV_ITVL_MS(ANUNCIO_MAX_MS);
    rc = ble_gap_adv_start(s_ble.tipo_direccion, NULL, BLE_HS_FOREVER, &parametros, evento_gap, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "No se pudo anunciar (%d)", rc);
    }
}

void ble_satcp_pausar_anuncio(bool pausar)
{
    if (pausar == s_ble.anuncio_pausado) {
        return;
    }
    s_ble.anuncio_pausado = pausar;
    if (!s_ble.sincronizado || ble_satcp_conectado()) {
        return;     // Al sincronizar o al desconectarse se anuncia (o no) según la marca
    }
    if (pausar) {
        ble_gap_adv_stop();
        ESP_LOGI(TAG, "Anuncio Bluetooth en pausa: hay una app conectada por Wi-Fi");
    } else {
        anunciar();
        ESP_LOGI(TAG, "Anuncio Bluetooth reanudado");
    }
}

static void al_sincronizar(void)
{
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_ble.tipo_direccion);
    s_ble.sincronizado = true;
    anunciar();
    ESP_LOGI(TAG, "Bluetooth listo como \"%s\"", BLE_NOMBRE_DISPOSITIVO);
}

static void al_reiniciar(int motivo)
{
    ESP_LOGW(TAG, "Pila Bluetooth reiniciada (motivo %d)", motivo);
}

static void tarea_host(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_satcp_iniciar(void)
{
    ESP_RETURN_ON_ERROR(nimble_port_init(), TAG, "no se pudo iniciar NimBLE");
    // NimBLE registra cada notificación ("GATT procedure initiated: notify"): solo avisos y errores
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    ble_hs_cfg.sync_cb = al_sincronizar;
    ble_hs_cfg.reset_cb = al_reiniciar;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_RETURN_ON_FALSE(ble_gatts_count_cfg(SERVICIOS) == 0, ESP_FAIL, TAG, "error en la tabla GATT");
    ESP_RETURN_ON_FALSE(ble_gatts_add_svcs(SERVICIOS) == 0, ESP_FAIL, TAG, "no se pudieron registrar los servicios");
    ble_svc_gap_device_name_set(BLE_NOMBRE_DISPOSITIVO);

    nimble_port_freertos_init(tarea_host);
    return ESP_OK;
}
