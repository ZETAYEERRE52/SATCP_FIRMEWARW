#include <string.h>
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "horarios.h"
#include "mdns.h"
#include "nvs.h"
#include "seguridad.h"
#include "wifi_satcp.h"

static const char *TAG = "WIFI";

#define NVS_ESPACIO             "satcp"
#define NVS_CLAVE_SSID          "wifi_ssid"
#define NVS_CLAVE_CLAVE         "wifi_clave"
#define CANAL_AP                1
#define MAX_CLIENTES_AP         4
#define REINTENTOS_SEGUIDOS     5
// Mientras la estación busca la red, el punto de acceso cambia de canal y los teléfonos conectados
// a "SATCP" pierden la conexión; por eso tras varios fallos se espera antes de volver a buscar
#define ESPERA_REINTENTO_US     (60 * 1000000LL)
#define ESPERA_DESCONEXION_MS   300     // Margen para que llegue el evento de la desconexión pedida
// Asociado a la red pero sin IP en este tiempo: el DHCP no respondió y se reintenta la conexión
#define ESPERA_DHCP_US          (15 * 1000000LL)
// 0: la estación no duerme (menos retraso; el equipo va enchufado). Si el Bluetooth se vuelve
// inestable por compartir la radio, poner 1 para volver al modo de ahorro por defecto
#define AHORRO_ENERGIA_WIFI     0

static struct {
    bool tiene_red;
    volatile bool cambiando_red;    // Mientras se guarda una red nueva, la desconexión no reconecta a la vieja
    int reintentos;
    char ssid[33];                  // Red de la casa configurada (para los mensajes del log)
    char ip[24];
    esp_timer_handle_t temporizador;
    esp_timer_handle_t vigilante_dhcp;
    portMUX_TYPE cerrojo;
} s_wifi = {
    .cerrojo = portMUX_INITIALIZER_UNLOCKED,
};

// Texto del motivo de desconexión, para no tener que buscar el número
static const char *describir_motivo(uint8_t motivo)
{
    switch (motivo) {
    case WIFI_REASON_AUTH_EXPIRE:               return "la red no respondió a la autenticación";
    case WIFI_REASON_AUTH_LEAVE:                return "la red cerró la sesión";
    case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY: return "desconectado por inactividad";
    case WIFI_REASON_ASSOC_TOOMANY:             return "la red tiene demasiados equipos conectados";
    case WIFI_REASON_ASSOC_LEAVE:               return "desconexión normal";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:    return "saludo de 4 pasos sin respuesta: casi siempre clave incorrecta";
    case WIFI_REASON_802_1X_AUTH_FAILED:        return "autenticación rechazada";
    case WIFI_REASON_STA_LEAVING:               return "desconexión pedida por el propio ESP32";
    case WIFI_REASON_TIMEOUT:                   return "tiempo de espera agotado";
    case WIFI_REASON_BEACON_TIMEOUT:            return "se perdió la señal de la red";
    case WIFI_REASON_NO_AP_FOUND:               return "red no encontrada: apagada, en 5 GHz o con otro nombre";
    case WIFI_REASON_AUTH_FAIL:                 return "autenticación fallida: clave incorrecta o la red rechaza al equipo";
    case WIFI_REASON_ASSOC_FAIL:                return "la red rechazó la asociación: límite de equipos, filtro MAC o modo no compatible";
    case WIFI_REASON_HANDSHAKE_TIMEOUT:         return "saludo sin respuesta: clave incorrecta o radio saturada";
    case WIFI_REASON_CONNECTION_FAIL:           return "falló la conexión";
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY: return "la red usa una seguridad no compatible (¿solo WPA3?)";
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD: return "la seguridad de la red es más débil que la exigida";
    default:                                    return "ver la tabla de motivos de ESP-IDF";
    }
}

static const char *describir_seguridad(wifi_auth_mode_t modo)
{
    switch (modo) {
    case WIFI_AUTH_OPEN:            return "abierta";
    case WIFI_AUTH_WPA_PSK:         return "WPA";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    default:                        return "otra";
    }
}

static void conectar(void)
{
    ESP_LOGI(TAG, "Conectando a \"%s\" (intento %d de %d)...", s_wifi.ssid, s_wifi.reintentos + 1,
             REINTENTOS_SEGUIDOS + 1);
    esp_wifi_connect();
}

// La desconexión dispara el reintento normal (y cuenta como un intento fallido)
static void dhcp_sin_respuesta(void *arg)
{
    ESP_LOGW(TAG, "Asociado a \"%s\" pero sin IP tras %lld s: el DHCP de la red no respondió; reconectando",
             s_wifi.ssid, ESPERA_DHCP_US / 1000000);
    esp_wifi_disconnect();
}

static void reintentar(void *arg)
{
    s_wifi.reintentos = 0;
    conectar();
}

static void manejar_evento(void *arg, esp_event_base_t base, int32_t id, void *datos)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            if (s_wifi.tiene_red) {
                conectar();
            }
            break;
        case WIFI_EVENT_STA_CONNECTED: {
            // Asociado: falta que el DHCP de la red entregue la IP. Si se queda aquí, el problema es el DHCP
            const wifi_event_sta_connected_t *conexion = (const wifi_event_sta_connected_t *)datos;
            ESP_LOGI(TAG, "Asociado a \"%.*s\" (canal %d, seguridad %s); esperando IP del DHCP...",
                     conexion->ssid_len, (const char *)conexion->ssid, conexion->channel,
                     describir_seguridad(conexion->authmode));
            esp_timer_stop(s_wifi.vigilante_dhcp);
            esp_timer_start_once(s_wifi.vigilante_dhcp, ESPERA_DHCP_US);
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED: {
            const wifi_event_sta_disconnected_t *desconexion = (const wifi_event_sta_disconnected_t *)datos;
            esp_timer_stop(s_wifi.vigilante_dhcp);
            if (s_wifi.tiene_red && !s_wifi.cambiando_red) {
                ESP_LOGW(TAG, "Sin conexión con \"%s\": motivo %d (%s), señal %d dBm", s_wifi.ssid,
                         desconexion->reason, describir_motivo(desconexion->reason), desconexion->rssi);
            }
            taskENTER_CRITICAL(&s_wifi.cerrojo);
            s_wifi.ip[0] = '\0';
            taskEXIT_CRITICAL(&s_wifi.cerrojo);
            if (!s_wifi.tiene_red || s_wifi.cambiando_red) {
                break;
            }
            if (++s_wifi.reintentos <= REINTENTOS_SEGUIDOS) {
                conectar();
            } else if (!esp_timer_is_active(s_wifi.temporizador)) {
                ESP_LOGW(TAG, "No se pudo conectar a la red de la casa; reintento en 60 s");
                esp_timer_start_once(s_wifi.temporizador, ESPERA_REINTENTO_US);
            }
            break;
        }
        case WIFI_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "Teléfono conectado a la red %s", WIFI_AP_SSID);
            break;
        case WIFI_EVENT_AP_STADISCONNECTED: {
            // 4/209 = el teléfono dejó de responder (pantalla apagada, cambió de red); 8 = se fue por su cuenta
            const wifi_event_ap_stadisconnected_t *salida = (const wifi_event_ap_stadisconnected_t *)datos;
            ESP_LOGW(TAG, "Teléfono desconectado de la red %s (motivo %d)", WIFI_AP_SSID, salida->reason);
            break;
        }
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *evento = (const ip_event_got_ip_t *)datos;
        esp_timer_stop(s_wifi.vigilante_dhcp);
        taskENTER_CRITICAL(&s_wifi.cerrojo);
        snprintf(s_wifi.ip, sizeof(s_wifi.ip), IPSTR, IP2STR(&evento->ip_info.ip));
        taskEXIT_CRITICAL(&s_wifi.cerrojo);
        s_wifi.reintentos = 0;
        // Bajo -75 dBm la red de la casa sirve para comandos, pero el video en vivo se traba
        wifi_ap_record_t router;
        const int rssi = esp_wifi_sta_get_ap_info(&router) == ESP_OK ? router.rssi : 0;
        ESP_LOGI(TAG, "IP obtenida en \"%s\": " IPSTR " (puerta de enlace " IPSTR ", %s.local), señal %d dBm%s",
                 s_wifi.ssid, IP2STR(&evento->ip_info.ip), IP2STR(&evento->ip_info.gw), WIFI_MDNS_NOMBRE, rssi,
                 rssi < -75 ? " (débil: acerca el router o el equipo)" : "");
        // Solo la red de la casa tiene internet: ahí se obtiene la hora para los horarios
        horarios_sincronizar_hora();
    }
}

static bool cargar_red(wifi_config_t *config)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_ESPACIO, NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }
    size_t tam_ssid = sizeof(config->sta.ssid);
    size_t tam_clave = sizeof(config->sta.password);
    const bool ok = nvs_get_str(nvs, NVS_CLAVE_SSID, (char *)config->sta.ssid, &tam_ssid) == ESP_OK &&
                    nvs_get_str(nvs, NVS_CLAVE_CLAVE, (char *)config->sta.password, &tam_clave) == ESP_OK;
    nvs_close(nvs);
    return ok && config->sta.ssid[0] != '\0';
}

// La contraseña de la red SATCP es la clave del equipo (la de la etiqueta, o la que puso el usuario)
static void configurar_ap(wifi_config_t *config)
{
    *config = (wifi_config_t){
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = sizeof(WIFI_AP_SSID) - 1,
            .channel = CANAL_AP,
            .max_connection = MAX_CLIENTES_AP,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strlcpy((char *)config->ap.password, seguridad_clave_equipo(), sizeof(config->ap.password));
}

esp_err_t wifi_satcp_actualizar_clave_ap(void)
{
    wifi_config_t config_ap;
    configurar_ap(&config_ap);
    // Los teléfonos conectados a SATCP se desconectan y deben volver a entrar con la clave nueva
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &config_ap), TAG, "config AP");
    ESP_LOGW(TAG, "Contraseña de la red %s actualizada", WIFI_AP_SSID);
    return ESP_OK;
}

esp_err_t wifi_satcp_iniciar(void)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init");
    esp_err_t ret = esp_event_loop_create_default();
    ESP_RETURN_ON_FALSE(ret == ESP_OK || ret == ESP_ERR_INVALID_STATE, ret, TAG, "bucle de eventos");
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    const wifi_init_config_t config_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&config_init), TAG, "esp_wifi_init");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, manejar_evento, NULL), TAG, "eventos");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, manejar_evento, NULL), TAG, "eventos");

    const esp_timer_create_args_t config_temporizador = { .callback = reintentar, .name = "wifi_reintento" };
    ESP_RETURN_ON_ERROR(esp_timer_create(&config_temporizador, &s_wifi.temporizador), TAG, "temporizador");
    const esp_timer_create_args_t config_vigilante = { .callback = dhcp_sin_respuesta, .name = "wifi_dhcp" };
    ESP_RETURN_ON_ERROR(esp_timer_create(&config_vigilante, &s_wifi.vigilante_dhcp), TAG, "vigilante DHCP");

    wifi_config_t config_ap;
    configurar_ap(&config_ap);
    wifi_config_t config_sta = { 0 };
    s_wifi.tiene_red = cargar_red(&config_sta);
    strlcpy(s_wifi.ssid, (const char *)config_sta.sta.ssid, sizeof(s_wifi.ssid));

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "modo");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &config_ap), TAG, "config AP");
    if (s_wifi.tiene_red) {
        ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &config_sta), TAG, "config estación");
    }
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start");
#if !AHORRO_ENERGIA_WIFI
    // Sin ahorro la estación no duerme entre balizas: las respuestas no esperan ~100 ms en el router
    const esp_err_t ret_ps = esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "Ahorro de energía del Wi-Fi desactivado: %s", esp_err_to_name(ret_ps));
#endif

    // Sin mDNS la app aún puede conectarse por IP: no se detiene el arranque
    mdns_txt_item_t txt[] = {
        { "ws", "/ws" },
        { "protocolo", "2" },
        { "vivo", "81" },
    };
    if (mdns_init() != ESP_OK || mdns_hostname_set(WIFI_MDNS_NOMBRE) != ESP_OK ||
        mdns_instance_name_set("SATCP") != ESP_OK ||
        mdns_service_add("SATCP", WIFI_MDNS_SERVICIO, "_tcp", WIFI_PUERTO, txt, sizeof(txt) / sizeof(txt[0])) != ESP_OK) {
        ESP_LOGW(TAG, "mDNS no disponible: la app deberá usar la IP");
    }

    ESP_LOGI(TAG, "Red propia \"%s\" (contraseña: la clave del equipo), IP %s", WIFI_AP_SSID, WIFI_AP_IP);
    if (s_wifi.tiene_red) {
        ESP_LOGI(TAG, "Conectando también a la red de la casa \"%s\"...", (const char *)config_sta.sta.ssid);
    }
    return ESP_OK;
}

esp_err_t wifi_satcp_configurar_red(const char *ssid, const char *clave)
{
    ESP_RETURN_ON_FALSE(ssid && clave && ssid[0], ESP_ERR_INVALID_ARG, TAG, "red inválida");
    wifi_config_t config_sta = { 0 };
    ESP_RETURN_ON_FALSE(strlen(ssid) < sizeof(config_sta.sta.ssid) && strlen(clave) < sizeof(config_sta.sta.password),
                        ESP_ERR_INVALID_SIZE, TAG, "nombre o clave demasiado largos");
    strlcpy((char *)config_sta.sta.ssid, ssid, sizeof(config_sta.sta.ssid));
    strlcpy((char *)config_sta.sta.password, clave, sizeof(config_sta.sta.password));

    nvs_handle_t nvs;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_ESPACIO, NVS_READWRITE, &nvs), TAG, "NVS");
    esp_err_t ret = nvs_set_str(nvs, NVS_CLAVE_SSID, ssid);
    if (ret == ESP_OK) {
        ret = nvs_set_str(nvs, NVS_CLAVE_CLAVE, clave);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs);
    }
    nvs_close(nvs);
    ESP_RETURN_ON_ERROR(ret, TAG, "no se pudo guardar la red");
    ESP_LOGI(TAG, "Red \"%s\" guardada en memoria", ssid);

    // Durante el cambio, el evento de desconexión no debe reconectar: lo haría con la red vieja
    esp_timer_stop(s_wifi.temporizador);
    s_wifi.cambiando_red = true;
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(ESPERA_DESCONEXION_MS));
    ret = esp_wifi_set_config(WIFI_IF_STA, &config_sta);
    strlcpy(s_wifi.ssid, ssid, sizeof(s_wifi.ssid));
    s_wifi.tiene_red = true;
    s_wifi.reintentos = 0;
    s_wifi.cambiando_red = false;
    ESP_RETURN_ON_ERROR(ret, TAG, "config estación");
    conectar();
    return ESP_OK;
}

bool wifi_satcp_ip(char *ip, size_t tam)
{
    taskENTER_CRITICAL(&s_wifi.cerrojo);
    strlcpy(ip, s_wifi.ip, tam);
    taskEXIT_CRITICAL(&s_wifi.cerrojo);
    return ip[0] != '\0';
}
