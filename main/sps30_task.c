#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "app_data.h"

static const char *TAG = "SPS30";
#define SPS30_ADDR 0x69
#define TIMEOUT pdMS_TO_TICKS(1000)

static esp_err_t sps30_write_start() {
    // Cmd 0x0010, Type 0x0300, CRC 0xAC
    uint8_t buf[5] = {0x00, 0x10, 0x03, 0x00, 0xAC};
    return i2c_master_write_to_device(0, SPS30_ADDR, buf, 5, TIMEOUT);
}

static esp_err_t sps30_read_val(float *pm25) {
    uint8_t cmd[2] = {0x03, 0x00};
    esp_err_t err = i2c_master_write_to_device(0, SPS30_ADDR, cmd, 2, TIMEOUT);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t raw[60];
    err = i2c_master_read_from_device(0, SPS30_ADDR, raw, 60, TIMEOUT);
    if (err != ESP_OK) return err;

    uint32_t pm25_i = ((uint32_t)raw[6]<<24)|((uint32_t)raw[7]<<16)|((uint32_t)raw[9]<<8)|raw[10];
    memcpy(pm25, &pm25_i, 4);
    return ESP_OK;
}

void sps30_task(void *pvParameters) {
    // ВАЖНО: Ждем 5 секунд, пока SCD30 проснется и успокоится
    ESP_LOGI(TAG, "Waiting 5s for SCD30 startup...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Запускаем SPS30
    if (sps30_write_start() == ESP_OK) {
        ESP_LOGI(TAG, "Started OK");
    } else {
        ESP_LOGE(TAG, "Start Failed (Check wiring!)");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        float p25;
        if (sps30_read_val(&p25) == ESP_OK) {
            global_data.pm2_5 = p25;
            global_data.sps30_valid = true;
        }
    }
}