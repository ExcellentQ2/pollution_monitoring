#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c.h"

#define TAG "SPS30"

// ---------------- I2C config ----------------
#define I2C_PORT            I2C_NUM_0
#define I2C_SDA             21
#define I2C_SCL             22
#define I2C_FREQ_HZ         100000

// ---------------- SPS30 ----------------
#define SPS30_ADDR          0x69

// SPS30 commands (16-bit)
#define CMD_START_MEAS      0x0010
#define CMD_STOP_MEAS       0x0104
#define CMD_DATA_READY      0x0202
#define CMD_READ_MEAS       0x0300

// Start measurement argument: 0x0300 => float mass concentration format
#define START_ARG_MASS      0x0300

// Sensirion CRC-8 (poly 0x31, init 0xFF)
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
    return i2c_master_write_to_device(I2C_PORT, SPS30_ADDR, buf, len, pdMS_TO_TICKS(1000));
}

static esp_err_t i2c_read(uint8_t *buf, size_t len) {
    return i2c_master_read_from_device(I2C_PORT, SPS30_ADDR, buf, len, pdMS_TO_TICKS(1000));
}

static void i2c_scan(void) {
    printf("\nI2C scan:\n");
    for (int addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(200));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("  Found device at 0x%02X\n", addr);
        }
    }
    printf("I2C scan done.\n\n");
}

static esp_err_t sps30_send_cmd(uint16_t cmd) {
    uint8_t pkt[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    return i2c_write(pkt, sizeof(pkt));
}

static esp_err_t sps30_start_measurement(void) {
    // [CMD_MSB CMD_LSB ARG_MSB ARG_LSB CRC(ARG)]
    uint8_t pkt[5];
    pkt[0] = (uint8_t)(CMD_START_MEAS >> 8);
    pkt[1] = (uint8_t)(CMD_START_MEAS & 0xFF);
    pkt[2] = (uint8_t)(START_ARG_MASS >> 8);
    pkt[3] = (uint8_t)(START_ARG_MASS & 0xFF);
    pkt[4] = sensirion_crc8(&pkt[2], 2);
    return i2c_write(pkt, sizeof(pkt));
}

static esp_err_t sps30_data_ready(bool *ready) {
    esp_err_t err = sps30_send_cmd(CMD_DATA_READY);
    if (err != ESP_OK) return err;

    // read 2 bytes + crc
    uint8_t r[3];
    err = i2c_read(r, sizeof(r));
    if (err != ESP_OK) return err;

    if (sensirion_crc8(&r[0], 2) != r[2]) return ESP_ERR_INVALID_CRC;

    // data-ready flag is second byte (0x00 / 0x01)
    *ready = (r[1] == 0x01);
    return ESP_OK;
}

static bool extract_float_6b(const uint8_t *six, float *out) {
    // six bytes: b0 b1 crc  b2 b3 crc
    if (sensirion_crc8(&six[0], 2) != six[2]) return false;
    if (sensirion_crc8(&six[3], 2) != six[5]) return false;

    // big-endian float bytes are: b0 b1 b2 b3
    uint8_t be[4] = { six[0], six[1], six[3], six[4] };
    uint32_t u = ((uint32_t)be[0] << 24) | ((uint32_t)be[1] << 16) | ((uint32_t)be[2] << 8) | be[3];

    float f;
    memcpy(&f, &u, sizeof(f));
    *out = f;
    return true;
}

static esp_err_t sps30_read_pm(float *pm1, float *pm25, float *pm4, float *pm10) {
    esp_err_t err = sps30_send_cmd(CMD_READ_MEAS);
    if (err != ESP_OK) return err;

    // 10 floats * 6 bytes each = 60 bytes
    uint8_t buf[60];
    err = i2c_read(buf, sizeof(buf));
    if (err != ESP_OK) return err;

    // first 4 floats are mass concentration PM1.0, PM2.5, PM4.0, PM10