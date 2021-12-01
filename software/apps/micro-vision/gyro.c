#include "gyro.h"
#include "nrf_delay.h"
#include "nrf_twi_mngr.h"

static const nrf_twi_mngr_t* i2c_interface = NULL;

uint8_t GYRO_ADDR = 0b1101011;
uint8_t GYRO_DATA_READY_MASK = 0b10;

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
  ASSERT(ret == NRF_SUCCESS);
}

// See gyro.h
void gyro_init(const nrf_twi_mngr_t* i2c) {
    ASSERT(i2c != NULL);
    i2c_interface = i2c;

    uint8_t whoamiValue = i2c_reg_read(GYRO_ADDR, GYRO_WHO_AM_I);
    ASSERT(whoamiValue == 0x6C);

    i2c_reg_write(GYRO_ADDR, GYRO_CTRL2_G, GYRO_104_HZ);
    //i2c_reg_write(GYRO_ADDR, GYRO_CTRL7_G, 0);
}

gyro_data_t gyro_read() {
  while (!gyro_ready()) {}

  uint8_t xlow = i2c_reg_read(GYRO_ADDR, GYRO_OUTX_L_G);
  uint8_t xhigh = i2c_reg_read(GYRO_ADDR, GYRO_OUTX_H_G);
  uint8_t ylow = i2c_reg_read(GYRO_ADDR, GYRO_OUTY_L_G);
  uint8_t yhigh = i2c_reg_read(GYRO_ADDR, GYRO_OUTY_H_G);
  uint8_t zlow = i2c_reg_read(GYRO_ADDR, GYRO_OUTZ_L_G);
  uint8_t zhigh = i2c_reg_read(GYRO_ADDR, GYRO_OUTZ_H_G);

  uint16_t xUnsigned = (xhigh << 8) | xlow;
  uint16_t yUnsigned = (yhigh << 8) | ylow;
  uint16_t zUnsigned = (zhigh << 8) | zlow;

  gyro_data_t gyro_data;
  gyro_data.x = *((int16_t*)&xUnsigned);
  gyro_data.y = *((int16_t*)&yUnsigned);
  gyro_data.z = *((int16_t*)&zUnsigned);

  return gyro_data;
}

bool gyro_ready() {
  return (i2c_reg_read(GYRO_ADDR, GYRO_STATUS_REG) & GYRO_DATA_READY_MASK) != 0;
}

