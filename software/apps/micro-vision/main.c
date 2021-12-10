#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "nrf_delay.h"
#include "nrf_twi_mngr.h"
#include "app_timer.h"

#include "microbit_v2.h"
#include "moto_bit.h"
#include "gyro.h"

#define THRESHOLD 800

// Global variables
NRF_TWI_MNGR_DEF(twi_mngr_instance, 1, 0);

int main(void) {
  // Delay for 2 seconds to give moto:bit time to initialize
  // If we don't do this, our initialization code is ignored by
  // its microprocessor and then none of the rest of the code
  // works
  nrf_delay_ms(2000);

  app_timer_init();

  // Initialize I2C peripheral and driver
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = EDGE_P19;
  i2c_config.sda = EDGE_P20;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  i2c_config.interrupt_priority = 0;
  nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);

  gyro_init(&twi_mngr_instance, GYRO_833_HZ);

  moto_bit_init(&twi_mngr_instance);

  char inbuf[1];
  float speed = 0.375f;
  float turn_speed = 0.75f;

  // We won't move initially so that the pi has time to boot
  // We will move forward after our first turn
  while (1) {
    int read_status = _read(0, inbuf, 1);
    if (read_status == -1) {
      //printf("Read error\n");
    } else {
      int8_t angle = *(int8_t*)inbuf;
      angle = -angle; // the angles sent by the pi are backwards from our perspective

      printf("Angle to turn: %d\n", angle);
      moto_bit_turn(angle, turn_speed);
      printf("done turn\n"); // send something back to indicate we're done

      moto_bit_set_speed(speed);
    }
  }
}

