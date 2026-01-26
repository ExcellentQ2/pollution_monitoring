#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_rom_gpio.h"

#include "cJSON.h"

#include "app_data.h"
#include "wifi.h"
#include "mqtt.h"
#include "oled_task.h"

static const char *TAG = "MAIN";

/* -------- I2C -------- */

#define SDA_PIN   21
#define SCL_PIN   22
#define I2C_PORT  I2C_NUM_0
#define I2C_FREQ  100000

void oled_task_start(void);


/* -------- Global Data -------- */

sensor_data_t global_data = {0};


/* -------- I2C Bus Recovery -------- */

static void i2c_bus_recovery(void)
{
    gpio_reset_pin(SDA_PIN);
    gpio_reset_pin(SCL_PIN);

    gpio_set_direction(SDA_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(SCL_PIN, GPIO_MODE_OUTPUT_OD);

    gpio_set_level(SDA_PIN, 1);

    for (int i = 0; i < 9; i++) {

        gpio_set_level(SCL_PIN, 0);
        esp_rom_delay_us(20);

        gpio_set_level(SCL_PIN, 1);
        esp_rom_delay_us(20);
    }

    gpio_set_level(SDA_PIN, 0);
    esp_rom_delay_us(20);

    gpio_set_level(SCL_PIN, 1);
    esp_rom_delay_us(20);

    gpio_set_level(SDA_PIN, 1);
}


/* -------- JSON Creator -------- */

static char* create_sensor_json(sensor_data_t *data)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    cJSON_AddStringToObject(root, "device_id", "esp32_city_smog");

    if (data->scd30_valid) {

        cJSON_AddNumberToObject(root, "co2_ppm", data->co2);
        cJSON_AddNumberToObject(root, "temperature_c", data->temperature);
        cJSON_AddNumberToObject(root, "humidity_rel", data->humidity);
    }

    if (data->sps30_valid) {

        cJSON_AddNumberToObject(root, "pm1_0", data->pm1_0);
        cJSON_AddNumberToObject(root, "pm2_5", data->pm2_5);
        cJSON_AddNumberToObject(root, "pm4_0", data->pm4_0);
        cJSON_AddNumberToObject(root, "pm10",  data->pm10);
    }

    char *json = cJSON_PrintUnformatted(root);

    cJSON_Delete(root);

    return json;
}


/* -------- Main -------- */

void app_main(void)
{
    esp_err_t err;

    /* -------- WiFi + MQTT -------- */

    wifi_init_sta();
    mqtt_init();

    /* Give network time to settle */
    vTaskDelay(pdMS_TO_TICKS(3000));


    /* -------- I2C Init -------- */

    i2c_bus_recovery();
    vTaskDelay(pdMS_TO_TICKS(200));

    i2c_config_t conf = {

        .mode = I2C_MODE_MASTER,

        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,

        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,

        .master.clk_speed = I2C_FREQ,
        .clk_flags = 0,
    };

    err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
    }

    err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "I2C init done");


    /* -------- Wait before sensors -------- */

    ESP_LOGI(TAG, "Waiting for sensors to boot...");
    vTaskDelay(pdMS_TO_TICKS(3000));


    /* -------- Start Sensors -------- */

    if (xTaskCreate(scd30_task, "scd30_task", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SCD30 task");
    }

    if (xTaskCreate(sps30_task, "sps30_task", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create SPS30 task");
    }


    /* -------- Start OLED -------- */

    oled_task_start();


    /* -------- Main Loop -------- */

    while (1) {

        vTaskDelay(pdMS_TO_TICKS(5000));


        /* Log sensor status */

        ESP_LOGI(TAG,
            "Status: SCD30=%d SPS30=%d",
            global_data.scd30_valid,
            global_data.sps30_valid
        );


        if (!global_data.scd30_valid &&
            !global_data.sps30_valid) {

            ESP_LOGW(TAG, "No sensor data yet");
            continue;
        }


        char *payload = create_sensor_json(&global_data);

        if (!payload) {

            ESP_LOGE(TAG, "JSON creation failed");
            continue;
        }


        ESP_LOGI(TAG, "Publishing MQTT: %s", payload);

        mqtt_publish(payload);

        free(payload);
    }
}
