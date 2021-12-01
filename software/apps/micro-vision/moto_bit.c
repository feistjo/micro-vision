#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "moto_bit.h"
#include "nrf_delay.h"
#include "nrf_twi_mngr.h"

#include "microbit_v2.h"
#include "nrf_assert.h"

static const nrf_twi_mngr_t* i2c_interface = NULL;

static float DEG_PER_UNIT_DIST = 0.4f; // TODO

// Helper function to perform a 1-byte I2C read of a given register
//
// i2c_addr - address of the device to read from
// reg_addr - address of the register within the device to read
//
// returns 8-bit read value
static uint8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr) {
  uint8_t rx_buf = 0;
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ(i2c_addr, &rx_buf, 1, 0)
  };
  ret_code_t ret = nrf_twi_mngr_perform(i2c_interface, NULL, read_transfer, 2, NULL);
  ASSERT(ret == NRF_SUCCESS);

  return rx_buf;
}

// Helper function to perform a 1-byte I2C write of a given register
//
// i2c_addr - address of the device to write to
// reg_addr - address of the register within the device to write
static void i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
  //Note: there should only be a single two-byte transfer to be performed
  uint8_t datas[] = {reg_addr, data};
  nrf_twi_mngr_transfer_t const write_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, datas, 2, 0),
  };
  ret_code_t ret = nrf_twi_mngr_perform(i2c_interface, NULL, write_transfer, 1, NULL);
  //ASSERT(ret == NRF_SUCCESS);
  //printf("ret %d, success %d\n", ret, NRF_SUCCESS);
}

// See moto_bit.h
void moto_bit_init(const nrf_twi_mngr_t* i2c) {
    ASSERT(i2c != NULL);
    i2c_interface = i2c;
    moto_bit_enable_motors();
    //i2c_reg_write(MOTO_BIT_ADDR, LEFT_MOTOR, 255);
    //i2c_reg_write(MOTO_BIT_ADDR, RIGHT_MOTOR, 255);
}

// Helper function that sets motor inversion such that
// a forward movement will turn to the left (ccw)
static void set_left_turn() {
    moto_bit_set_motor_inverted(INVERT_LEFT, true);
    moto_bit_set_motor_inverted(INVERT_RIGHT, false);
}

// Helper function that sets motor inversion such that
// a forward movement will turn to the right (cw)
static void set_right_turn() {
    moto_bit_set_motor_inverted(INVERT_LEFT, false);
    moto_bit_set_motor_inverted(INVERT_RIGHT, true);
}

// Helper function which sets the motors to move forward
static void set_straight() {
    moto_bit_set_motor_inverted(INVERT_LEFT, false);
    moto_bit_set_motor_inverted(INVERT_RIGHT, false);
}

// See moto_bit.h
void moto_bit_turn(float angle, float speed) {
    ASSERT(speed >= 0);

    float millis = (angle >= 0 ? angle : -1*angle) / DEG_PER_UNIT_DIST / speed;
    if (angle > 0) {
        set_left_turn();
    } else {
        set_right_turn();
    }

    moto_bit_set_speed(speed);
    nrf_delay_ms(millis);
    moto_bit_set_speed(0);
    set_straight();
}

// See moto_bit.h
void moto_bit_set_speed(float speed) {
    ASSERT(speed <= 1.0f && speed >= -1.0f);

    uint8_t speed_val = 0;
    if (speed >= 0) {
        // Forward direction: MSB is 1, lower bits are 0-127, the larger the faster
        speed_val = speed * 127;
        speed_val += 128;
    } else {
        // Reverse direction: MSB is 0, lower bits are 0-127, the larger the slower
        speed_val = 128 + (speed * 127);
    }

    i2c_reg_write(MOTO_BIT_ADDR, LEFT_MOTOR, speed_val);
    i2c_reg_write(MOTO_BIT_ADDR, RIGHT_MOTOR, speed_val);
}

// See moto_bit.h
void moto_bit_set_motor_inverted(moto_bit_reg_t motor, bool inverted) {

    // Fix register addresses in case the wrong ones are used
    if (motor == LEFT_MOTOR) {
        motor = INVERT_LEFT;
    } else if (motor == RIGHT_MOTOR) {
        motor = INVERT_RIGHT;
    }

    ASSERT(motor == INVERT_LEFT || motor == INVERT_RIGHT);

    i2c_reg_write(MOTO_BIT_ADDR, motor, inverted ? 1 : 0);
}

// See moto_bit.h
void moto_bit_disable_motors() {
    //i2c_reg_write(MOTO_BIT_ADDR, ENABLE_MOTORS, 0);
    moto_bit_set_motors_enabled(false);
}

// See moto_bit.h
void moto_bit_enable_motors() {
    //i2c_reg_write(MOTO_BIT_ADDR, ENABLE_MOTORS, 1);
    moto_bit_set_motors_enabled(true);
}

// See moto_bit.h
void moto_bit_set_motors_enabled(bool enabled) {
    i2c_reg_write(MOTO_BIT_ADDR, ENABLE_MOTORS, enabled ? 1 : 0);
}

// See moto_bit.h
void moto_bit_stop() {
    moto_bit_set_speed(0.0f);
}
