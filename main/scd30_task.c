#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "app_data.h"

static const char *TAG = "SCD30";
#define SCD30_ADDR 0x61
#define TIMEOUT pdMS_TO_TICKS(1000)

static uint8_t scd30_calc_crc(const uint8_t *data, int len) {
    uint8_t crc = 0xFF;
    for (int j = 0; j < len; j++) {
        crc ^= data[j];
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static esp_err_t scd30_write_val(uint16_t cmd, uint16_t val) {
    uint8_t buf[5];
    buf[0] = cmd >> 8; buf[1] = cmd & 0xFF;
    buf[2] = val >> 8; buf[3] = val & 0xFF;
    buf[4] = scd30_calc_crc(&buf[2], 2);
    return i2c_master_write_to_device(0, SCD30_ADDR, buf, 5, TIMEOUT);
}

static esp_err_t scd30_write_cmd(uint16_t cmd) {
    uint8_t buf[2] = {cmd >> 8, cmd & 0xFF};
    return i2c_master_write_to_device(0, SCD30_ADDR, buf, 2, TIMEOUT);
}

static esp_err_t scd30_read(uint16_t cmd, uint8_t *data, size_t len) {
    uint8_t buf[2] = {cmd >> 8, cmd & 0xFF};
    esp_err_t err = i2c_master_write_to_device(0, SCD30_ADDR, buf, 2, TIMEOUT);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(50));
    return i2c_master_read_from_device(0, SCD30_ADDR, data, len, TIMEOUT);
}

void scd30_task(void *pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "Init SCD30...");
        scd30_write_cmd(0xD304); // Reset
        vTaskDelay(pdMS_TO_TICKS(2000));
        scd30_write_val(0x4600, 2); // Interval 2s
        
        if (scd30_write_val(0x0010, 0x0000) == ESP_OK) {
            ESP_LOGI(TAG, "Started!");
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(2000));
                uint8_t rdy[3] = {0};
                
                // Проверяем готовность данных
                if (scd30_read(0x0202, rdy, 3) == ESP_OK && rdy[1] == 1) {
                    uint8_t raw[18];
                    // Читаем 18 байт (CO2, Temp, Hum)
                    if (scd30_read(0x0300, raw, 18) == ESP_OK) {
                        
                        // Парсим CO2
                        uint32_t co2_i = ((uint32_t)raw[0]<<24)|((uint32_t)raw[1]<<16)|((uint32_t)raw[3]<<8)|raw[4];
                        float co2; memcpy(&co2, &co2_i, 4);

                        // Парсим Температуру
                        uint32_t temp_i = ((uint32_t)raw[6]<<24)|((uint32_t)raw[7]<<16)|((uint32_t)raw[9]<<8)|raw[10];
                        float temp; memcpy(&temp, &temp_i, 4);

                        // Парсим Влажность (ЭТО МЫ ДОБАВИЛИ)
                        uint32_t hum_i = ((uint32_t)raw[12]<<24)|((uint32_t)raw[13]<<16)|((uint32_t)raw[15]<<8)|raw[16];
                        float hum; memcpy(&hum, &hum_i, 4);

                        // Сохраняем ВСЁ в глобальную структуру
                        global_data.co2 = co2;
                        global_data.temperature = temp;
                        global_data.humidity = hum;       // <--- Сохраняем влажность
                        global_data.scd30_valid = true;
                        
                        // Лог для проверки
                        ESP_LOGI(TAG, "Read: CO2=%.0f, T=%.2f, Hum=%.1f%%", co2, temp, hum);
                    }
                }
            }
        } else {
            ESP_LOGE(TAG, "Start fail, retry in 3s...");
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}