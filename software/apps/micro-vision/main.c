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
  printf("\n==============\nBoard started!\n==============\nSleeping for 2 seconds to give the moto:bit time to initialize...");
  nrf_delay_ms(2000);
  printf("done!\n");

  printf("Initializing app_timer library...");
  app_timer_init();
  printf("done!\n");

  // Initialize I2C peripheral and driver
  printf("Initializing I2C...");
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = EDGE_P19;
  i2c_config.sda = EDGE_P20;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  i2c_config.interrupt_priority = 0;
  nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  printf("done!\n");

  printf("Initializing gyro...");
  gyro_init(&twi_mngr_instance, GYRO_833_HZ);
  printf("done!\n");

  printf("Initializing moto:bit...");
  moto_bit_init(&twi_mngr_instance);
  printf("done!\n");

  while (1) {
    printf("Turning left 90\n");
    moto_bit_turn(90, 0.5f);
    printf("Finished turn!\n");
    nrf_delay_ms(1000);
    printf("Turning right 90\n");
    moto_bit_turn(-90, 0.5f);
    printf("Finished turn!\n");
    nrf_delay_ms(1000);
  }
}

