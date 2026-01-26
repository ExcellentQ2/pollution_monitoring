#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"

#include "app_data.h"

/* ================= CONFIG ================= */

#define TAG "OLED"

#define OLED_ADDR      0x3C

#define I2C_PORT       I2C_NUM_0
#define SDA_PIN        21
#define SCL_PIN        22
#define I2C_FREQ       400000

/* ========================================= */

extern sensor_data_t global_data;


/* ---------- Low-level I2C ---------- */

static esp_err_t oled_write_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};

    i2c_cmd_handle_t handle = i2c_cmd_link_create();

    i2c_master_start(handle);
    i2c_master_write_byte(handle, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(handle, buf, 2, true);
    i2c_master_stop(handle);

    esp_err_t ret = i2c_master_cmd_begin(
        I2C_PORT, handle, pdMS_TO_TICKS(100));

    i2c_cmd_link_delete(handle);

    return ret;
}


static esp_err_t oled_write_data(uint8_t *data, size_t len)
{
    uint8_t ctrl = 0x40;

    i2c_cmd_handle_t handle = i2c_cmd_link_create();

    i2c_master_start(handle);
    i2c_master_write_byte(handle, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(handle, ctrl, true);
    i2c_master_write(handle, data, len, true);
    i2c_master_stop(handle);

    esp_err_t ret = i2c_master_cmd_begin(
        I2C_PORT, handle, pdMS_TO_TICKS(100));

    i2c_cmd_link_delete(handle);

    return ret;
}


/* ---------- SSD1306 Init ---------- */

static void oled_init(void)
{
    oled_write_cmd(0xAE);
    oled_write_cmd(0x20);
    oled_write_cmd(0x00);
    oled_write_cmd(0xB0);
    oled_write_cmd(0xC8);
    oled_write_cmd(0x00);
    oled_write_cmd(0x10);
    oled_write_cmd(0x40);
    oled_write_cmd(0x81);
    oled_write_cmd(0xFF);
    oled_write_cmd(0xA1);
    oled_write_cmd(0xA6);
    oled_write_cmd(0xA8);
    oled_write_cmd(0x3F);
    oled_write_cmd(0xA4);
    oled_write_cmd(0xD3);
    oled_write_cmd(0x00);
    oled_write_cmd(0xD5);
    oled_write_cmd(0xF0);
    oled_write_cmd(0xD9);
    oled_write_cmd(0x22);
    oled_write_cmd(0xDA);
    oled_write_cmd(0x12);
    oled_write_cmd(0xDB);
    oled_write_cmd(0x20);
    oled_write_cmd(0x8D);
    oled_write_cmd(0x14);
    oled_write_cmd(0xAF);

    ESP_LOGI(TAG, "SSD1306 initialized");
}


/* ---------- Clear ---------- */

static void oled_clear(void)
{
    uint8_t buf[128];
    memset(buf, 0, sizeof(buf));

    for (int page = 0; page < 8; page++) {

        oled_write_cmd(0xB0 + page);
        oled_write_cmd(0x00);
        oled_write_cmd(0x10);

        oled_write_data(buf, 128);
    }
}


/* ---------- Task ---------- */

static void oled_task(void *arg)
{
    oled_clear();

    while (1) {

        ESP_LOGI(TAG,
            "T=%.1f H=%.1f CO2=%.0f PM2.5=%.1f",
            global_data.temperature,
            global_data.humidity,
            global_data.co2,
            global_data.pm2_5
        );

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


/* ---------- Start ---------- */

void oled_task_start(void)
{
    oled_init();

    xTaskCreate(
        oled_task,
        "oled_task",
        4096,
        NULL,
        4,
        NULL
    );
}
