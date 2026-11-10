idf_component_register(SRCS "maquina_estados.c" "acciones_presencia.c" "acciones_vision.c" "acciones_simuladas.c"
                            "valvula.c"
                       INCLUDE_DIRS "include"
                       REQUIRES monitor_temperatura esp_driver_gpio
                       PRIV_REQUIRES esp_timer ld2410 vision horarios nvs_flash)
