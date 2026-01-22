idf_component_register(
    SRCS
        "main.c"
        "wifi.c"
        "mqtt.c"
        "scd30_task.c"
        "sps30_task.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        cjson
        esp_wifi
        esp_event
        nvs_flash
        mqtt
)
