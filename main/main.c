#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c.h"

#include "esp_log.h"

#include "cJSON.h"

#include "app_data.h"
#include "wifi.h"
#include "mqtt.h"
#include "oled_task.h"

static const char *TAG = "MAIN";

/* I2C */

#define SDA_PIN   21
#define SCL_PIN   22
#define I2C_PORT  I2C_NUM_0
#define I2C_FREQ  100000


/* Global */

sensor_data_t global_data = {0};
SemaphoreHandle_t i2c_mutex;


/* JSON */

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


/* Main */

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "BOOT");


    /* Mutex */

    i2c_mutex = xSemaphoreCreateMutex();

    if (!i2c_mutex) {
        ESP_LOGE(TAG, "I2C mutex failed");
        return;
    }


    /* WiFi + MQTT */

    wifi_init_sta();
    mqtt_init();

    vTaskDelay(pdMS_TO_TICKS(3000));


    /* I2C Init */

    i2c_config_t conf = {

        .mode = I2C_MODE_MASTER,

        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,

        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,

        .master.clk_speed = I2C_FREQ,
    };

    err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed");
        return;
    }

    err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C install failed");
        return;
    }

    ESP_LOGI(TAG, "I2C OK");


    vTaskDelay(pdMS_TO_TICKS(3000));


    /* Start Sensors */

    xTaskCreate(scd30_task, "scd30", 8192, NULL, 5, NULL);
    xTaskCreate(sps30_task, "sps30", 4096, NULL, 5, NULL);


    /* OLED */

    oled_task_start();


    /* Loop */

    while (1) {

        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG,
            "Status: SCD30=%d SPS30=%d",
            global_data.scd30_valid,
            global_data.sps30_valid
        );


        if (!global_data.scd30_valid &&
            !global_data.sps30_valid) {
            continue;
        }


        char *payload = create_sensor_json(&global_data);

        if (!payload) continue;


        mqtt_publish(payload);

        free(payload);
    }
}
