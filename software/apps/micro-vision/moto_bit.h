#pragma once

#include "nrf_twi_mngr.h"

static const uint8_t MOTO_BIT_ADDR = 0x59;

//register defs
typedef enum {
  LEFT_MOTOR = 0x21;
  RIGHT_MOTOR = 0x20;
  INVERT_LEFT = 0x13;
  INVERT_RIGHT = 0x12;
  ENABLE_MOTORS = 0x70;
} moto_bit_reg_t;

void moto_bit_init(const nrf_twi_mngr_t* i2c);

void turn(float angle, uint8_t speed);

void set_speed(uint8_t speed);

void invert_motor(uint8_t motor);

void disable_motors();

void enable_motors();
