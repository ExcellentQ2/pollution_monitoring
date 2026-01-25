#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"

#include "app_data.h"   // where your shared sensor values live

static const char *TAG = "OLED";

/* ----------- CONFIG ----------- */
#define OLED_I2C_ADDR   0x3C
#define OLED_WIDTH      128
#define OLED_HEIGHT     64

#define I2C_PORT        I2C_NUM_0   // must match your existing bus
#define I2C_SDA_PIN     21          // must match your existing pins
#define I2C_SCL_PIN     22

static esp_lcd_panel_handle_t panel_handle = NULL;

static void oled_task(void *arg)
{
    char buf[256];

    while (1) {
        snprintf(buf, sizeof(buf),
            "Temp: %.1f C\n"
            "Hum: %.1f %%\n"
            "CO2: %.1f ppm\n"
            "PM1: %.1f\n"
            "PM2.5: %.1f\n"
            "PM4: %.1f\n"
            "PM10: %.1f",
            temp_val, hum_val, co2_val,
            pm1_val, pm25_val, pm4_val, pm10_val
        );

        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, OLED_WIDTH, OLED_HEIGHT, buf);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void oled_task_start(void)
{
    /* I2C bus (already used by sensors, so DO NOT re-init if already inited) */
    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = OLED_I2C_ADDR,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_PORT, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "SSD1306 OLED initialized");

    xTaskCreate(oled_task, "oled_task", 4096, NULL, 5, NULL);
}
