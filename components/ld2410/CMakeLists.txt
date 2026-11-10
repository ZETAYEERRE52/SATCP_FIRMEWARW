idf_component_register(SRCS "ld2410.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_uart esp_driver_gpio
                       PRIV_REQUIRES esp_timer)
