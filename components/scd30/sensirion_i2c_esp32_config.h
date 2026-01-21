#ifndef SENSIRION_I2C_ESP32_CONFIG_H
#define SENSIRION_I2C_ESP32_CONFIG_H

#include <driver/i2c.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

// Структура для хранения настроек I2C
struct esp32_i2c_config {
    i2c_port_t port;        // Номер порта (I2C_NUM_0)
    uint32_t freq;          // Частота (100000)
    gpio_num_t sda;         // Пин SDA
    gpio_num_t scl;         // Пин SCL
    bool sda_pullup;        // Включить подтяжку SDA?
    bool scl_pullup;        // Включить подтяжку SCL?
};

// Функция для передачи настроек в драйвер
void sensirion_i2c_config_esp32(struct esp32_i2c_config* cfg);

#ifdef __cplusplus
}
#endif

#endif /* SENSIRION_I2C_ESP32_CONFIG_H */