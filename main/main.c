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

// --- I2C settings ---
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_PORT 0
#define I2C_FREQ 100000

// ===== MQTT TEST MODE =====
// 1 = fake data
// 0 = real sensor
#define MQTT_TEST_MODE 0

sensor_data_t global_data = {0};

// --- I2C Bus recovery ---
void i2c_bus_recovery() {
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

// --- JSON creator (EXTENDED, not redesigned) ---
char* create_sensor_json(sensor_data_t *data) {
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    cJSON_AddStringToObject(root, "device_id", "esp32_city_smog");

    // SCD30 data
    if (data->scd30_valid) {
        cJSON_AddNumberToObject(root, "co2_ppm", data->co2);
        cJSON_AddNumberToObject(root, "temperature_c", data->temperature);
        cJSON_AddNumberToObject(root, "humidity_rel", data->humidity);
    }

    // SPS30 data
    if (data->sps30_valid) {
        cJSON_AddNumberToObject(root, "pm1_0", data->pm1_0);
        cJSON_AddNumberToObject(root, "pm2_5", data->pm2_5);
        cJSON_AddNumberToObject(root, "pm4_0", data->pm4_0);
        cJSON_AddNumberToObject(root, "pm10",  data->pm10);
    }

    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_string;
}

void app_main(void)
{
    // ===== WiFi + MQTT =====
    wifi_init_sta();
    mqtt_init();
    // ======================

#if !MQTT_TEST_MODE
    // Only run I2C + sensor when NOT in test mode

    i2c_bus_recovery();
    vTaskDelay(pdMS_TO_TICKS(100));

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C Initialized.");

    xTaskCreate(scd30_task, "scd30_task", 4096, NULL, 5, NULL);
    xTaskCreate(sps30_task, "sps30_task", 4096, NULL, 5, NULL);
    oled_task_start();
#endif

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

#if MQTT_TEST_MODE
    // ===== FAKE DATA (MQTT TEST MODE) =====
    sensor_data_t test = {
        .co2 = 500.0,
        .temperature = 22.5,
        .humidity = 45.0,

        .pm1_0 = 3.2,
        .pm2_5 = 7.8,
        .pm4_0 = 12.4,
        .pm10  = 18.9,

        .scd30_valid = true,
        .sps30_valid = true
    };

    char *payload = create_sensor_json(&test);

    if (payload != NULL) {
        ESP_LOGI(TAG, "JSON Payload (TEST MODE): %s", payload);
        mqtt_publish(payload);
        free(payload);
    }

#else
        if (global_data.scd30_valid || global_data.sps30_valid) {
            char *payload = create_sensor_json(&global_data);

            if (payload != NULL) {
                ESP_LOGI(TAG, "JSON Payload: %s", payload);
                mqtt_publish(payload);
                free(payload);
            }
        } else {
            ESP_LOGW(TAG, "Waiting for sensor data...");
        }
#endif
    }
}
 
