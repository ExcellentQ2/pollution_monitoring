#include "c02.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "scd30_i2c.h"
#include "sensirion_i2c_hal.h"

static const char *TAG = "C02_WORKER";

void c02_task_loop(void *pvParameters) {
    sensirion_i2c_hal_init();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    int16_t err = scd30_start_periodic_measurement(0);
    if (err != 0) {
        ESP_LOGE(TAG, "Failed to start SCD30: %d", err);
        vTaskDelete(NULL);
    }
    
    ESP_LOGI(TAG, "SCD30 started");

    while (1) {
        uint16_t data_ready = 0;
        scd30_get_data_ready(&data_ready);
        
        if (data_ready) {
            float co2, temp, hum;
            err = scd30_read_measurement_data(&co2, &temp, &hum);
            
            if (err == 0) {
                ESP_LOGI(TAG, "CO2: %.2f ppm | T: %.2f C | H: %.2f %%", co2, temp, hum);
            } else {
                ESP_LOGE(TAG, "Read error: %d", err);
            }
        }
        
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void c02_task_start(void) {
    xTaskCreate(c02_task_loop, "c02_task", 4096, NULL, 5, NULL);
}