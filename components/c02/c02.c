#include "c02.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/i2c.h> // Нужно для сканера

// Подключаем драйвер SCD30
#include "scd30_i2c.h" 
#include "sensirion_i2c_hal.h"

static const char *TAG = "C02_WORKER";

// --- ВАЖНО: Убедитесь, что эти настройки совпадают с sensirion_i2c_hal.c ---
// Если вы используете HAL, порт там уже выбран, но для сканера нам нужно знать какой.
// Обычно это I2C_NUM_0, если вы не меняли на 1.
#define I2C_PORT_NUM I2C_NUM_1

/**
 * @brief Функция для проверки, есть ли живые устройства на шине
 */
void i2c_scanner(void) {
    ESP_LOGI(TAG, ">> Starting I2C Scan...");
    int devices_found = 0;
    for (int i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        // Таймаут 50мс
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "FOUND DEVICE AT: 0x%02x", i);
            devices_found++;
        }
    }
    if (devices_found == 0) {
        ESP_LOGE(TAG, "!!! NO I2C DEVICES FOUND !!! Check wiring and pins.");
    } else {
        ESP_LOGI(TAG, "I2C Scan complete. Found %d devices.", devices_found);
    }
}

void c02_task_loop(void *pvParameters) {
    // 1. Инициализация шины I2C (через ваш HAL)
    sensirion_i2c_hal_init();

    // Даем датчику время "проснуться" (SCD30 медленный)
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // 2. ЗАПУСКАЕМ СКАНЕР
    // Если в логах не будет адреса 0x61, значит проблема в проводах/пинах!
    i2c_scanner();

    // 3. Пытаемся запустить датчик (цикл попыток)
    int retry_count = 0;
    while (1) {
        // 0 - атмосферное давление (0 = деактивировать компенсацию)
        int16_t err = scd30_start_periodic_measurement(0);
        
        if (err == 0) {
            ESP_LOGI(TAG, "SCD30 started successfully!");
            break; // Выходим из цикла инициализации
        }
        
        ESP_LOGE(TAG, "Failed to start SCD30 (Error: %d). Retrying... (%d)", err, ++retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        // Если 5 раз не вышло - возможно, нужно передернуть HAL
        if (retry_count % 5 == 0) {
            ESP_LOGW(TAG, "Re-initializing I2C Bus...");
            sensirion_i2c_hal_free();
            vTaskDelay(100 / portTICK_PERIOD_MS);
            sensirion_i2c_hal_init();
        }
    }

    // 4. Основной цикл опроса
    while (1) {
        uint16_t data_ready = 0;
        int16_t err = scd30_get_data_ready(&data_ready);
        
        if (err != 0) {
            ESP_LOGW(TAG, "Error checking data ready: %d", err);
        } else if (data_ready) {
            float co2, temp, hum;
            err = scd30_read_measurement_data(&co2, &temp, &hum);
            
            if (err == 0) {
                ESP_LOGI(TAG, "CO2: %.2f ppm | T: %.2f C | H: %.2f %%", co2, temp, hum);
            } else {
                ESP_LOGE(TAG, "Read measurement error: %d", err);
            }
        } else {
            // Данные еще не готовы, это нормально для SCD30
            // ESP_LOGD(TAG, "Data not ready yet...");
        }

        // ВАЖНО: Никогда не делайте return из задачи FreeRTOS!
        // Если ошибка - просто ждем и пробуем снова.
        
        // SCD30 обновляет данные раз в 2 секунды
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void c02_task_start(void) {
    xTaskCreate(c02_task_loop, "c02_task", 4096, NULL, 5, NULL);
}