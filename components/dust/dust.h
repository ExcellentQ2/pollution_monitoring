/* components/dust/dust.h */
#ifndef DUST_H
#define DUST_H

#include <stdbool.h>
#include <esp_err.h>

// Функции для управления сенсором

// Запустить измерения
esp_err_t sps30_start_measurement(void);

// Проверить, готовы ли новые данные
esp_err_t sps30_data_ready(bool *ready);

// Прочитать данные (PM1.0, PM2.5, PM4.0, PM10.0)
esp_err_t sps30_read_pm(float *pm1, float *pm25, float *pm4, float *pm10);

#endif // DUST_H