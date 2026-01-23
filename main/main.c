#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "display.h"
#include "sensors.h"
#include "scd30_task.h"
#include "sps30_task.h"
#include "esp_log.h"

void app_main(void)
{
    // Initialize I2C bus 
    i2c_config_t conf={
        .mode=I2C_MODE_MASTER,
        .sda_io_num=21,
        .scl_io_num=22,
        .sda_pullup_en=GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,
        .master.clk_speed=100000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0,&conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER,0,0,0));

    ESP_LOGI("APP_MAIN","Starting tasks...");
    xTaskCreate(display_task,"display_task",4096,NULL,5,NULL);
    //xTaskCreate(scd30_task,"scd30_task",4096,NULL,5,NULL);
    //xTaskCreate(sps30_task,"sps30_task",4096,NULL,5,NULL);
}
