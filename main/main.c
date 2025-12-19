#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "c02.h"

// --- ИСПРАВЛЕНИЕ: Пишите только имена файлов, без папок ---

// #include "dust.h"   // Если будете подключать пыль, тоже просто "dust.h"

static const char *TAG = "MAIN_APP";

void app_main(void) {
    ESP_LOGI(TAG, "--- Project Starting ---");
    c02_task_start();

    // Запускаем дисплей
   // display_task_start();

    // Запускаем датчик CO2
    
    
    // Пыль пока отключена, так как мы не исправляли конфликты I2C
    // dust_task_start(); 

    ESP_LOGI(TAG, "--- All systems go ---");
}