#include "display.h"
#include "sensors.h"
#include "lvgl.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define TAG "DISPLAY"
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8];
static _lock_t lvgl_api_lock;

// Stub sensor data (replace with real sensor variables)
extern float temp_val;
extern float hum_val;
extern float co2_val;
extern float pm1_val;
extern float pm25_val;
extern float pm4_val;
extern float pm10_val;

// --- LVGL tick increment callback ---
static void lv_tick_cb(void *arg) {
    lv_tick_inc(5); // 5 ms tick
}

// --- Flush callback ---
static void lv_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    esp_lcd_panel_handle_t panel = lv_display_get_user_data(disp);

    for (int y = area->y1; y <= area->y2; y++) {
        for (int x = area->x1; x <= area->x2; x++) {
            bool pixel = (px_map[(OLED_WIDTH >> 3) * y + (x >> 3)] & (1 << (7 - x % 8)));
            uint8_t *buf = oled_buffer + OLED_WIDTH * (y >> 3) + x;
            if (pixel) {
                (*buf) &= ~(1 << (y % 8));
            } else {
                (*buf) |= (1 << (y % 8));
            }
        }
    }

    esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, oled_buffer);
}

// --- Display task ---
void display_task(void *arg) {
    ESP_LOGI(TAG, "Initializing SSD1306...");

    // --- Initialize I2C panel ---
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr = 0x3C,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(I2C_NUM_0, &io_cfg, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_cfg = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1
    };
    esp_lcd_panel_ssd1306_config_t ssd_cfg = {
        .height = OLED_HEIGHT
    };
    panel_cfg.vendor_config = &ssd_cfg;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_cfg, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // --- Initialize LVGL ---
    lv_init();
    lv_display_t *disp = lv_display_create(OLED_WIDTH, OLED_HEIGHT);
    lv_display_set_user_data(disp, panel_handle);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_I1);
    lv_display_set_buffers(disp, oled_buffer, NULL, sizeof(oled_buffer), LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(disp, lv_flush_cb);

    // --- Create label ---
    _lock_acquire(&lvgl_api_lock);
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, OLED_WIDTH);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    _lock_release(&lvgl_api_lock);

    // --- Start LVGL tick timer ---
    const esp_timer_create_args_t tick_args = {
        .callback = lv_tick_cb,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 5000)); 

    ESP_LOGI(TAG, "Display task started");
    
    char buf[128];
    while (1) {
        _lock_acquire(&lvgl_api_lock);

        snprintf(buf, sizeof(buf),
            "Temp: %.1f C\nHum: %.1f %%\nCO2: %.1f ppm\nPM1: %.1f\nPM2.5: %.1f\nPM4: %.1f\nPM10: %.1f",
            temp_val, hum_val, co2_val, pm1_val, pm25_val, pm4_val, pm10_val);

        lv_label_set_text(label, buf);

        _lock_release(&lvgl_api_lock);
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}
