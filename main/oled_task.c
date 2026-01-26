#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "string.h"

#include "app_data.h"

#define OLED_ADDR  0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define I2C_PORT I2C_NUM_0

static const char *TAG = "OLED";

/* ================= FONT 5x7 ================= */

static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // space 32
    {0x00,0x00,0x5F,0x00,0x00}, // !
    {0x00,0x07,0x00,0x07,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14}, // #
    {0x24,0x2A,0x7F,0x2A,0x12}, // $
    {0x23,0x13,0x08,0x64,0x62}, // %
    {0x36,0x49,0x55,0x22,0x50}, // &
    {0x00,0x05,0x03,0x00,0x00}, // '
};

/* Only printable ASCII 32–127 needed */
#define FONT_OFFSET 32

/* ================= LOW LEVEL ================= */

static void oled_cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};

    i2c_master_write_to_device(
        I2C_PORT, OLED_ADDR,
        data, 2,
        100 / portTICK_PERIOD_MS
    );
}

static void oled_data(uint8_t *data, size_t len)
{
    uint8_t buf[len + 1];
    buf[0] = 0x40;

    memcpy(&buf[1], data, len);

    i2c_master_write_to_device(
        I2C_PORT, OLED_ADDR,
        buf, len + 1,
        100 / portTICK_PERIOD_MS
    );
}

/* ================= CORE ================= */

static void oled_set_cursor(uint8_t page, uint8_t col)
{
    oled_cmd(0xB0 | page);
    oled_cmd(0x00 | (col & 0x0F));
    oled_cmd(0x10 | (col >> 4));
}

static void oled_clear(void)
{
    uint8_t zero[128];
    memset(zero, 0, sizeof(zero));

    for (int i = 0; i < 8; i++) {

        oled_set_cursor(i, 0);
        oled_data(zero, 128);
    }
}

static void oled_char(char c)
{
    if (c < 32 || c > 127) c = '?';

    uint8_t buf[6];

    memcpy(buf, font5x7[c - FONT_OFFSET], 5);
    buf[5] = 0x00;

    oled_data(buf, 6);
}

static void oled_string(const char *s)
{
    while (*s) {
        oled_char(*s++);
    }
}

/* ================= INIT ================= */

static void oled_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    oled_cmd(0xAE);
    oled_cmd(0x20); oled_cmd(0x00);
    oled_cmd(0xB0);
    oled_cmd(0xC8);
    oled_cmd(0x00);
    oled_cmd(0x10);
    oled_cmd(0x40);
    oled_cmd(0x81); oled_cmd(0xFF);
    oled_cmd(0xA1);
    oled_cmd(0xA6);
    oled_cmd(0xA8); oled_cmd(0x3F);
    oled_cmd(0xA4);
    oled_cmd(0xD3); oled_cmd(0x00);
    oled_cmd(0xD5); oled_cmd(0xF0);
    oled_cmd(0xD9); oled_cmd(0x22);
    oled_cmd(0xDA); oled_cmd(0x12);
    oled_cmd(0xDB); oled_cmd(0x20);
    oled_cmd(0x8D); oled_cmd(0x14);
    oled_cmd(0xAF);

    ESP_LOGI(TAG, "OLED initialized");
}

/* ================= TASK ================= */

static void oled_task(void *arg)
{
    char buf[64];

    oled_init();
    oled_clear();

    while (1) {

        oled_clear();

        /* Line 1 */
        oled_set_cursor(0, 0);

        if (global_data.scd30_valid) {

            snprintf(buf, sizeof(buf),
                     "T:%.1f H:%.1f",
                     global_data.temperature,
                     global_data.humidity);

        } else {
            strcpy(buf, "SCD30: ---");
        }

        oled_string(buf);


        /* Line 2 */
        oled_set_cursor(2, 0);

        if (global_data.scd30_valid) {

            snprintf(buf, sizeof(buf),
                     "CO2: %.0f ppm",
                     global_data.co2);

        } else {
            strcpy(buf, "CO2: ---");
        }

        oled_string(buf);


        /* Line 3 */
        oled_set_cursor(4, 0);

        if (global_data.sps30_valid) {

            snprintf(buf, sizeof(buf),
                     "PM2.5: %.1f",
                     global_data.pm2_5);

        } else {
            strcpy(buf, "PM: ---");
        }

        oled_string(buf);


        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void oled_task_start(void)
{
    xTaskCreate(oled_task, "oled", 4096, NULL, 4, NULL);
}
