idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES monitor_temperatura sistema_seguridad ld2410 vision comunicaciones horarios nvs_flash)
