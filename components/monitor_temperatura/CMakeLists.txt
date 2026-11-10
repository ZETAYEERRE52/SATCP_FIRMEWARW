idf_component_register(SRCS "monitor_temperatura.c" "buffer_circular.c" "calibracion_temperatura.c"
                       INCLUDE_DIRS "include"
                       REQUIRES mlx90614 esp_driver_i2c esp_driver_gpio
                       PRIV_REQUIRES esp_timer nvs_flash)
