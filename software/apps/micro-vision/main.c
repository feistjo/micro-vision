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
  nrf_twi_mngr_perform(&twi_mngr_instance, NULL, write_transfer, 1, NULL);
}

int main(void) {
  printf("Board started!\n");
  nrf_delay_ms(2000);

  // Initialize I2C peripheral and driver
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = EDGE_P19;
  i2c_config.sda = EDGE_P20;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  i2c_config.interrupt_priority = 0;
  nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);

  printf("I2C initialized!\n");

  moto_bit_init(&twi_mngr_instance);

  printf("moto:bit initialized\n");

  printf("Moving forward\n");
  moto_bit_set_speed(0.5f);
  nrf_delay_ms(2000);

  printf("Turning left\n");
  moto_bit_turn(90, 0.5f);

  printf("Forward\n");
  moto_bit_set_speed(0.5f);
  nrf_delay_ms(2000);

  moto_bit_stop();
  nrf_delay_ms(2000);

  moto_bit_set_speed(-0.5f);
  nrf_delay_ms(2000);

  moto_bit_turn(-90, 0.5f);
  //nrf_delay_ms(1000);

  moto_bit_set_speed(-0.5f);
  nrf_delay_ms(2000);

  moto_bit_stop();

  while (1) {
    nrf_delay_ms(1000);
  }
}

