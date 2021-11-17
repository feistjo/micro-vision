#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "nrf_twi_mngr.h"

static const uint8_t MOTO_BIT_ADDR = 0x59;

//register defs
typedef enum {
  LEFT_MOTOR = 0x21,
  RIGHT_MOTOR = 0x20,
  INVERT_LEFT = 0x13,
  INVERT_RIGHT = 0x12,
  ENABLE_MOTORS = 0x70
} moto_bit_reg_t;

/**
 * Initialize moto_bit interface.
 *
 * @param i2c I2C interface object
 */
void moto_bit_init(const nrf_twi_mngr_t* i2c);

/**
 * Turn the moto_bit.
 *
 * @param angle The angle in degrees to turn. Positive values turn counter-clockwise
 *              and negative values turn clockwise.
 * @param speed The speed from 0.0f to 1.0f to turn at.
 */
void moto_bit_turn(float angle, float speed);

/**
 * Set the speed of the moto_bit.
 *
 * @param speed The speed, in the range -1.0f to 1.0f. Positive values indicate forward
 *              movement, negative values indicate backward movement, and 0.0f indicates
 *              stopping movement altogether.
 */
void moto_bit_set_speed(float speed);

/**
 * Inverts/uninverts a motor. An inverted motor will drive backward if given a
 * forward speed and vice-versa.
 *
 * @param motor Which motor to invert.
 * @param inverted Whether the motor should be inverted.
 */
void moto_bit_set_motor_inverted(moto_bit_reg_t motor, bool inverted);

/**
 * Disables the motors.
 */
void moto_bit_disable_motors();

/**
 * Enables the motors.
 */
void moto_bit_enable_motors();

/**
 * Sets whether the motors should be enabled. Disabled motors will not respond to
 * commands.
 */
void moto_bit_set_motors_enabled(bool enabled);

/**
 * Stops the moto_bit. This is equivalent to moto_bit_set_speed(0.0f).
 */
void moto_bit_stop();
