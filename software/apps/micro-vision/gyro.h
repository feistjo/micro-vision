#pragma once

#include <stdint.h>

#include "nrf_twi_mngr.h"

uint8_t GYRO_ADDR;

typedef enum {
    GYRO_CTRL2_G = 0x11,
    GYRO_WHO_AM_I = 0x0F,
    GYRO_CTRL7_G = 0x16, // Defaults of 0 should be fine?
    GYRO_STATUS_REG = 0x1E,
    GYRO_OUTX_L_G = 0x22,
    GYRO_OUTX_H_G = 0x23,
    GYRO_OUTY_L_G = 0x24,
    GYRO_OUTY_H_G = 0x25,
    GYRO_OUTZ_L_G = 0x26,
    GYRO_OUTZ_H_G = 0x27
} gyro_reg_t;

typedef enum {
    GYRO_OFF = 0,
    GYRO_12_5_HZ = 1 << 4,
    GYRO_26_HZ = 2 << 4,
    GYRO_52_HZ = 3 << 4,
    GYRO_104_HZ = 4 << 4,
    GYRO_208_HZ = 5 << 4,
    GYRO_416_HZ = 6 << 4,
    GYRO_833_HZ = 7 << 4,
    GYRO_1660_HZ = 8 << 4,
    GYRO_3330_HZ = 9 << 4,
    GYRO_6660_HZ = 10 << 4
} gyro_odr_t;

uint8_t GYRO_DATA_READY_MASK;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} gyro_data_t;

float GYRO_TICKS_PER_DEGREE;

/**
 * Initialize gyro interface.
 *
 * @param i2c I2C interface object
 */
void gyro_init(const nrf_twi_mngr_t* i2c);

gyro_data_t gyro_read();

bool gyro_ready();
