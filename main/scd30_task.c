#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "app_data.h"


static const char *TAG = "SCD30";

#define SCD30_ADDR 0x61
#define TIMEOUT pdMS_TO_TICKS(1000)


/* CRC */

static uint8_t scd30_crc(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;

    for (int j = 0; j < len; j++) {

        crc ^= data[j];

        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x80) ?
                (uint8_t)((crc << 1) ^ 0x31) :
                (uint8_t)(crc << 1);
        }
    }

    return crc;
}


/* I2C Helpers */

static esp_err_t scd30_write(uint8_t *buf, int len)
{
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    esp_err_t err =
        i2c_master_write_to_device(
            0, SCD30_ADDR, buf, len, TIMEOUT
        );

    xSemaphoreGive(i2c_mutex);

    return err;
}


static esp_err_t scd30_read(uint16_t cmd, uint8_t *data, int len)
{
    uint8_t buf[2] = {cmd >> 8, cmd & 0xFF};

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    esp_err_t err =
        i2c_master_write_to_device(
            0, SCD30_ADDR, buf, 2, TIMEOUT
        );

    if (err == ESP_OK) {

        vTaskDelay(pdMS_TO_TICKS(50));

        err = i2c_master_read_from_device(
            0, SCD30_ADDR, data, len, TIMEOUT
        );
    }

    xSemaphoreGive(i2c_mutex);

    return err;
}


/* Task */

void scd30_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Init");


    /* Reset */

    uint8_t rst[2] = {0xD3, 0x04};

    if (scd30_write(rst, 2) != ESP_OK) {
        ESP_LOGE(TAG, "Reset failed");
        vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(3000));


    /* Interval = 2s */

    uint8_t interval[5];

    interval[0] = 0x46;
    interval[1] = 0x00;
    interval[2] = 0x00;
    interval[3] = 0x02;
    interval[4] = scd30_crc(&interval[2], 2);

    scd30_write(interval, 5);


    /* Start */

    uint8_t start[5];

    start[0] = 0x00;
    start[1] = 0x10;
    start[2] = 0x00;
    start[3] = 0x00;
    start[4] = scd30_crc(&start[2], 2);

    if (scd30_write(start, 5) != ESP_OK) {
        ESP_LOGE(TAG, "Start failed");
        vTaskDelete(NULL);
    }


    ESP_LOGI(TAG, "Running");


    /* Loop */

    while (1) {

        vTaskDelay(pdMS_TO_TICKS(2000));

        uint8_t ready[3];

        if (scd30_read(0x0202, ready, 3) != ESP_OK)
            continue;

        if (ready[1] != 1)
            continue;


        uint8_t raw[18];

        if (scd30_read(0x0300, raw, 18) != ESP_OK)
            continue;


        uint32_t co2_i =
            ((uint32_t)raw[0]<<24) |
            ((uint32_t)raw[1]<<16) |
            ((uint32_t)raw[3]<<8)  |
             raw[4];

        float co2;
        memcpy(&co2, &co2_i, 4);


        uint32_t temp_i =
            ((uint32_t)raw[6]<<24) |
            ((uint32_t)raw[7]<<16) |
            ((uint32_t)raw[9]<<8)  |
             raw[10];

        float temp;
        memcpy(&temp, &temp_i, 4);


        uint32_t hum_i =
            ((uint32_t)raw[12]<<24) |
            ((uint32_t)raw[13]<<16) |
            ((uint32_t)raw[15]<<8)  |
             raw[16];

        float hum;
        memcpy(&hum, &hum_i, 4);


        global_data.co2 = co2;
        global_data.temperature = temp;
        global_data.humidity = hum;

        global_data.scd30_valid = true;


        ESP_LOGI(TAG,
            "CO2=%.0f T=%.2f H=%.1f%%",
            co2, temp, hum
        );
    }
}
