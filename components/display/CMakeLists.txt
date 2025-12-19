idf_component_register(SRCS "display.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_timer lvgl esp_lcd)