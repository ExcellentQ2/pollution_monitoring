#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "cJSON.h"        // Библиотека JSON
#include "app_data.h"

static const char *TAG = "MAIN";

// --- Настройки I2C ---
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_PORT 0
#define I2C_FREQ 20000 

sensor_data_t global_data = {0};

// --- Функция "Лечения" шины (Bus Recovery) ---
void i2c_bus_recovery() {
    gpio_reset_pin(SDA_PIN); gpio_reset_pin(SCL_PIN);
    gpio_set_direction(SDA_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(SCL_PIN, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(SDA_PIN, 1);
    for (int i = 0; i < 9; i++) {
        gpio_set_level(SCL_PIN, 0); esp_rom_delay_us(20);
        gpio_set_level(SCL_PIN, 1); esp_rom_delay_us(20);
    }
    gpio_set_level(SDA_PIN, 0); esp_rom_delay_us(20);
    gpio_set_level(SCL_PIN, 1); esp_rom_delay_us(20);
    gpio_set_level(SDA_PIN, 1);
}

// --- ФУНКЦИЯ СОЗДАНИЯ JSON (С ВЛАЖНОСТЬЮ) ---
char* create_sensor_json(float co2, float temp, float hum) {
    // 1. Создаем пустой JSON объект: {}
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) return NULL;

    // 2. Добавляем данные
    cJSON_AddStringToObject(root, "device_id", "esp32_scd30");
    cJSON_AddNumberToObject(root, "co2_ppm", co2);
    
    // Округляем до 2 знаков для красоты (опционально)
    cJSON_AddNumberToObject(root, "temperature_c", temp);
    
    // --> ДОБАВЛЕНА ВЛАЖНОСТЬ <--
    cJSON_AddNumberToObject(root, "humidity_rel", hum);

    // 3. Превращаем объект в строку
    // cJSON_PrintUnformatted делает строку без пробелов (экономит байты)
    char *json_string = cJSON_PrintUnformatted(root);

    // 4. Очищаем память объекта cJSON (Важно!)
    cJSON_Delete(root);

    return json_string;
}

void app_main(void)
{
    // 1. Лечение шины
    i2c_bus_recovery();
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. Инициализация драйвера
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

    // 3. Запуск задач
    xTaskCreate(scd30_task, "scd30_task", 4096, NULL, 5, NULL);
    
    // SPS30 временно отключен
    // xTaskCreate(sps30_task, "sps30_task", 4096, NULL, 5, NULL);

    // 4. Главный цикл: отправка данных
    while (1) {
        // Пауза перед отправкой (например, каждые 5 секунд)
        vTaskDelay(pdMS_TO_TICKS(5000));

        if (global_data.scd30_valid) {
            
            // Создаем JSON строку
            char *payload = create_sensor_json(
                global_data.co2, 
                global_data.temperature, 
                global_data.humidity // Передаем влажность
            );

            if (payload != NULL) {
                // Выводим результат (тут потом будет отправка в Cloud)
                ESP_LOGI(TAG, "JSON Payload: %s", payload);

                // Обязательно освобождаем память строки!
                free(payload);
            }
        } else {
            ESP_LOGW(TAG, "Waiting for sensor data...");
        }
    }
}