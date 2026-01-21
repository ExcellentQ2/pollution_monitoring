/* components/dust/dust.c */
#include "dust.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "SPS30_DRIVER"

// Константы
#define SPS30_ADDR          0x69
#define I2C_PORT            I2C_NUM_0 // Используем порт, открытый в main

// Команды
#define CMD_START_MEAS      0x0010
#define CMD_STOP_MEAS       0x0104
#define CMD_DATA_READY      0x0202
#define CMD_READ_MEAS       0x0300
#define START_ARG_MASS      0x0300

// --- Внутренние функции (Hidden) ---

static uint8_t sensirion_crc8(const uint8_t *data, int len) {
    uint8_t crc = 0xFF;
    for (int j = 0; j < len; j++) {
        crc ^= data[j];
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static esp_err_t i2c_write(const uint8_t *buf, size_t len) {
    return i2c_master_write_to_device(I2C_PORT, SPS30_ADDR, buf, len, pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read(uint8_t *buf, size_t len) {
    return i2c_master_read_from_device(I2C_PORT, SPS30_ADDR, buf, len, pdMS_TO_TICKS(100));
}

static esp_err_t sps30_send_cmd(uint16_t cmd) {
    uint8_t pkt[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    return i2c_write(pkt, sizeof(pkt));
}

static bool extract_float_6b(const uint8_t *six, float *out) {
    if (sensirion_crc8(&six[0], 2) != six[2]) return false;
    if (sensirion_crc8(&six[3], 2) != six[5]) return false;

    uint8_t be[4] = { six[0], six[1], six[3], six[4] };
    uint32_t u = ((uint32_t)be[0] << 24) | ((uint32_t)be[1] << 16) | ((uint32_t)be[2] << 8) | be[3];

    float f;
    memcpy(&f, &u, sizeof(f));
    *out = f;
    return true;
}

// --- Публичные функции (API) ---

esp_err_t sps30_start_measurement(void) {
    uint8_t pkt[5];
    pkt[0] = (uint8_t)(CMD_START_MEAS >> 8);
    pkt[1] = (uint8_t)(CMD_START_MEAS & 0xFF);
    pkt[2] = (uint8_t)(START_ARG_MASS >> 8);
    pkt[3] = (uint8_t)(START_ARG_MASS & 0xFF);
    pkt[4] = sensirion_crc8(&pkt[2], 2);
    return i2c_write(pkt, sizeof(pkt));
}

esp_err_t sps30_data_ready(bool *ready) {
    esp_err_t err = sps30_send_cmd(CMD_DATA_READY);
    if (err != ESP_OK) return err;

    uint8_t r[3];
    err = i2c_read(r, sizeof(r));
    if (err != ESP_OK) return err;

    if (sensirion_crc8(&r[0], 2) != r[2]) return ESP_ERR_INVALID_CRC;
    *ready = (r[1] == 0x01);
    return ESP_OK;
}

esp_err_t sps30_read_pm(float *pm1, float *pm25, float *pm4, float *pm10) {
    esp_err_t err = sps30_send_cmd(CMD_READ_MEAS);
    if (err != ESP_OK) return err;

    uint8_t buf[60];
    err = i2c_read(buf, sizeof(buf));
    if (err != ESP_OK) return err;

    if (!extract_float_6b(&buf[0],  pm1))  return ESP_ERR_INVALID_CRC;
    if (!extract_float_6b(&buf[6],  pm25)) return ESP_ERR_INVALID_CRC;
    if (!extract_float_6b(&buf[12], pm4))  return ESP_ERR_INVALID_CRC;
    if (!extract_float_6b(&buf[18], pm10)) return ESP_ERR_INVALID_CRC;

    return ESP_OK;
}