#ifndef APP_DATA_H
#define APP_DATA_H

#include <stdbool.h>
#include "freertos/semphr.h"

extern SemaphoreHandle_t i2c_mutex;

typedef struct {
    // Данные SPS30
    float pm1_0;
    float pm2_5;
    float pm4_0;
    float pm10;
    bool sps30_valid;

    // Данные SCD30
    float co2;
    float temperature;
    float humidity;
    bool scd30_valid;
} sensor_data_t;

// Глобальная переменная, доступная всем
extern sensor_data_t global_data;

// Прототипы функций задач
void sps30_task(void *pvParameters);
void scd30_task(void *pvParameters);

#endif