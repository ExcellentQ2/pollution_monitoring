idf_component_register(SRCS "main.c"
                            "c02.c"
                            "dust.c"
                            "display.c"
                       INCLUDE_DIRS "."
                       REQUIRES scd30 esp_timer driver esp_lcd lvgl) # <-- Добавлено esp_lcd