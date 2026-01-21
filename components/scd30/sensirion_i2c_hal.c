#include "sensirion_i2c_hal.h"
#include "sensirion_common.h"
#include "sensirion_config.h"
#include "sensirion_i2c_esp32_config.h" // Наш конфиг

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <i2cdev.h>        // Библиотека esp-idf-lib
#include <esp_rom_sys.h>

static const char *TAG = "SCD30_HAL";

// Локальное хранилище настроек и дескриптора устройства
static struct esp32_i2c_config i2c_config = {0};
static i2c_dev_t dev = {0};

// Функция приема настроек из main.c
void sensirion_i2c_config_esp32(struct esp32_i2c_config* cfg) {
    if (cfg) {
        i2c_config = *cfg;
    }
}

// Инициализация HAL
void sensirion_i2c_hal_init(void) {
    // 1. Инициализация глобальной библиотеки i2cdev (Критично!)
    i2cdev_init();

    // 2. Очистка и применение настроек
    memset(&dev, 0, sizeof(i2c_dev_t));
    
    dev.port = i2c_config.port;
    dev.cfg.sda_io_num = i2c_config.sda;
    dev.cfg.scl_io_num = i2c_config.scl;
    dev.cfg.master.clk_speed = i2c_config.freq;
    dev.cfg.sda_pullup_en = i2c_config.sda_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    dev.cfg.scl_pullup_en = i2c_config.scl_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;

    // 3. Создание мьютекса
    esp_err_t res = i2c_dev_create_mutex(&dev);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create mutex: %d", res);
    } else {
        ESP_LOGI(TAG, "Init OK. Port: %d, SDA: %d, SCL: %d", 
                 dev.port, dev.cfg.sda_io_num, dev.cfg.scl_io_num);
    }
}

void sensirion_i2c_hal_free(void) {
    i2c_dev_delete_mutex(&dev);
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
    dev.addr = address; // Обновляем адрес перед чтением
    esp_err_t res = i2c_dev_read(&dev, NULL, 0, data, (size_t)count);
    return (res == ESP_OK) ? 0 : -1;
}

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint8_t count) {
    dev.addr = address; // Обновляем адрес перед записью
    esp_err_t res = i2c_dev_write(&dev, NULL, 0, data, (size_t)count);
    return (res == ESP_OK) ? 0 : -1;
}

void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    if (useconds < 10000) {
        esp_rom_delay_us(useconds);
    } else {
        vTaskDelay(pdMS_TO_TICKS(useconds / 1000));
    }
}