idf_component_register(SRCS "camara.c" "almacenamiento_sd.c" "captura_dataset.c" "clasificador.cc" "foto_clasificada.c"
                            "modelo/modelo_fuego.cc"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "modelo"
                       REQUIRES espressif__esp32-camera esp_driver_gpio
                       PRIV_REQUIRES fatfs sdmmc esp_driver_sdmmc nvs_flash esp_timer
                                     espressif__esp-tflite-micro espressif__esp_jpeg)
