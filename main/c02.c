#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "scd30.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

static const char *TAG = "SCD30_EXAMPLE";

void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void scd30_task(void *pvParameters) {
    scd30_t dev;
    
    i2c_master_init();

    if (scd30_init_desc(&dev, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO) != ESP_OK) {
        ESP_LOGE(TAG, "SCD30 descriptor init failed");
        vTaskDelete(NULL);
    }

    if (scd30_start_continuous_measurement(&dev, 0) != ESP_OK) {
        ESP_LOGE(TAG, "SCD30 start measurement failed");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "SCD30 measurement started");

    while (1) {
        if (scd30_get_data_ready(&dev)) {
            float co2, temp, hum;

            if (scd30_read_measurement(&dev, &co2, &temp, &hum) == ESP_OK) {
                ESP_LOGI(TAG, "CO2: %.2f ppm | Temp: %.2f C | RH: %.2f %%", co2, temp, hum);
            } else {
                ESP_LOGE(TAG, "Failed to read SCD30 measurement");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    xTaskCreate(scd30_task, "scd30_task", 4096, NULL, 5, NULL);
}