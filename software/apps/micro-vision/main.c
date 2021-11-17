#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "nrf_delay.h"
#include "nrf_twi_mngr.h"

#include "microbit_v2.h"
#include "moto_bit.h"

// Global variables
NRF_TWI_MNGR_DEF(twi_mngr_instance, 1, 0);

int main(void) {
  printf("Board started!\n");

  // Initialize I2C peripheral and driver
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = I2C_SCL;
  i2c_config.sda = I2C_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  i2c_config.interrupt_priority = 0;
  nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);

  moto_bit_init(&twi_mngr_instance);

  moto_bit_set_speed(1.0f);
  nrf_delay_ms(500);

  moto_bit_turn(90, 0.25f);

  moto_bit_set_speed(0.5f);
  nrf_delay_ms(500);

  moto_bit_stop();
  nrf_delay_ms(250);

  moto_bit_set_speed(-0.5f);
  nrf_delay_ms(500);

  moto_bit_turn(-90, 0.25f);

  moto_bit_set_speed(-1.0f);
  nrf_delay_ms(500);

  moto_bit_stop();

  while (1) {
    nrf_delay_ms(1000);
  }
}

