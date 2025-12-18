#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_log.h>

// Ваши пины
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 50000
#define I2C_MASTER_NUM I2C_NUM_0

static const char *TAG = "I2C_SCAN";

void app_main(void) {
    // 1. Настройка I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    printf("I2C Scanner started...\n");

    // 2. Сканирование адресов от 1 до 127
    int devices_found = 0;
    for (int i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        // Отправляем команду и ждем ACK
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("Found device at address: 0x%02x\n", i);
            devices_found++;
        }
    }

    if (devices_found == 0) {
        printf("NO devices found! Check wiring (SDA/SCL swapped?) and Power.\n");
    }
    
    // Удаляем драйвер, чтобы не мешать, если захотите вернуть старый код
    i2c_driver_delete(I2C_MASTER_NUM);
}