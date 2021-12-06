#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "moto_bit.h"
#include "nrf_delay.h"
#include "nrf_twi_mngr.h"
#include "app_timer.h"
#include "gyro.h"

#include "microbit_v2.h"
#include "nrf_assert.h"

APP_TIMER_DEF(gyro_sample_timer);

static const nrf_twi_mngr_t* i2c_interface = NULL;

static float THRESHOLD = 100.0f; // TODO
static float curr_angle = 0;

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
    ASSERT(ret == NRF_SUCCESS);
}

// Samples the gyro and adds its value to the current angle
static void gyro_sample(void* _unused) {
    gyro_data_t gyro_data = gyro_read();

    // filter noise so we don't accumulate error too much
    if (fabs(gyro_data.z) < THRESHOLD) {
      gyro_data.z = 0;
    }

    curr_angle += gyro_data.z / GYRO_TICKS_PER_DEGREE;
}

// See moto_bit.h
void moto_bit_init(const nrf_twi_mngr_t* i2c) {
    ASSERT(i2c != NULL);
    i2c_interface = i2c;
    moto_bit_enable_motors();
    app_timer_create(&gyro_sample_timer, APP_TIMER_MODE_REPEATED, gyro_sample);
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

// Returns 1 * the sign of the number, or 0 if the number is 0
static int8_t sign(float num) {
    if (num > 0) {
        return 1;
    } else if (num < 0) {
        return -1;
    } else {
        return 0;
    }
}

// See moto_bit.h
void moto_bit_turn(float angle, float speed) {
    ASSERT(speed >= 0);

    curr_angle = 0;
    int8_t initial_sign = sign(angle); // sign(angle - curr_angle) == sign(angle - 0) == sign(angle)
    app_timer_start(gyro_sample_timer, 32786 / 100, NULL); // gyro runs at 104 hz, so if we sample at 100 hz there _should_ always be data relatively quickly

    if (angle > 0) {
        set_left_turn();
    } else {
        set_right_turn();
    }

    moto_bit_set_speed(speed);

    // Wait for turn to complete
    // We know the turn has completed when we go past the goal angle, but the goal angle
    // could be positive or negative, so instead we wait for the sign of the difference
    // between the current and goal angles to change
    while (sign(angle - curr_angle) == initial_sign) {}

    app_timer_stop(gyro_sample_timer);
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
