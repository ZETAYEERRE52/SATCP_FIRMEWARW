idf_component_register(SRCS "comunicaciones.c" "protocolo.c" "seguridad.c" "wifi_satcp.c" "servidor_web.c"
                            "servidor_vivo.c" "ble_satcp.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_wifi esp_netif esp_event esp_http_server esp_timer esp_hw_support esp_rom
                                     nvs_flash bt espressif__cjson espressif__mdns mbedtls bootloader_support lwip
                                     vision sistema_seguridad monitor_temperatura ld2410 horarios)
