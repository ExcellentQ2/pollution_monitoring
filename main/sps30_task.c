#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dust.h"
#include "app_data.h"

static const char *TAG = "SPS30_TASK";

void sps30_task(void *pvParameters)
{
    ESP_LOGI(TAG, "SPS30 task starting");

    /* Try to start measurement, do NOT crash if it fails */
    esp_err_t err = sps30_start_measurement();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPS30 start failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "SPS30 measurement started");
    }

    while (1) {

        bool ready = false;

        err = sps30_data_ready(&ready);

        if (err != ESP_OK) {

            ESP_LOGE(TAG, "Data ready failed: %s", esp_err_to_name(err));

        } 
        else if (ready) {

            float pm1, pm25, pm4, pm10;

            err = sps30_read_pm(&pm1, &pm25, &pm4, &pm10);

            if (err == ESP_OK) {

                global_data.pm1_0 = pm1;
                global_data.pm2_5 = pm25;
                global_data.pm4_0 = pm4;
                global_data.pm10  = pm10;

                global_data.sps30_valid = true;

                ESP_LOGI(TAG,
                    "PM: %.1f %.1f %.1f %.1f",
                    pm1, pm25, pm4, pm10
                );

            } 
            else {
                ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
