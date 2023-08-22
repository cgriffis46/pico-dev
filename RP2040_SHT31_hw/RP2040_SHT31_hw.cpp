
#include "include/RP2040_SHT31.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "include/RP2040_SHT31.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

RP2040_SHT31_hw::RP2040_SHT31_hw(i2c_inst_t *i2c){
  this->i2c = *i2c;
}

bool RP2040_SHT31_hw::begin(uint8_t devAddress){
  this->devAddress = devAddress;
  this->reset();
  return this->readStatus() != 0xFFFF;
}

/**
 * Gets the current status register contents.
 *
 * @return The 16-bit status register.
 */
uint16_t RP2040_SHT31_hw::readStatus(void) {

  this->writeCommand(SHT31_READSTATUS);
  int bytesRead;

  uint8_t data[3];
 
  i2c_read_blocking(i2c1, SHT31_DEFAULT_ADDR, data, 3,false);

  uint16_t stat = data[0];
  stat <<= 8;
  stat |= data[1];
  return stat;
}

bool RP2040_SHT31_hw::writeCommand(uint16_t command){
  uint8_t cmd[2];
  int b;
  cmd[0] = command >> 8;
  cmd[1] = command & 0xFF;
  b = i2c_write_blocking(i2c1, SHT31_DEFAULT_ADDR, cmd, 2, false);
  if(b>0) return true;
  return false;
}

bool RP2040_SHT31_hw::readTempHum(float* t, float* h){
  uint8_t readbuffer[6];
  int bytesRead = 0;

  uint8_t cmd[2];

  cmd[0] = SHT31_MEAS_HIGHREP_MSB;
  cmd[1] = SHT31_MEAS_HIGHREP_LSB; 
  
  i2c_write_blocking(i2c1, SHT31_DEFAULT_ADDR, cmd, 2, false);
  sleep_ms(20);
  i2c_read_blocking(i2c1,SHT31_DEFAULT_ADDR,readbuffer, 6,false);

   if (readbuffer[2] != this->crc8(readbuffer, 2) ||
      readbuffer[5] != this->crc8(readbuffer + 3, 2))
    return false;

  int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) + readbuffer[1]);
  // simplified (65536 instead of 65535) integer version of:
  // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
  stemp = ((4375 * stemp) >> 14) - 4500;
  this->temp = (float)stemp / 100.0f;
  *t = temp;
  uint32_t shum = ((uint32_t)readbuffer[3] << 8) + readbuffer[4];
  // simplified (65536 instead of 65535) integer version of:
  // humidity = (shum * 100.0f) / 65535.0f;
  shum = (625 * shum) >> 12;
  this->humidity = (float)shum / 100.0f;
  *h = this->humidity;

  return true;
}


/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
uint8_t RP2040_SHT31_hw::crc8(const uint8_t *data, int len) {
  /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

void RP2040_SHT31_hw::reset(){
  uint8_t cmd[2];
  cmd[0] = SHT31_SOFTRESET_MSB;
  cmd[1] = SHT31_SOFTRESET_LSB;
  i2c_write_blocking(i2c1, SHT31_DEFAULT_ADDR, cmd, 2, false);
  sleep_us(2000);
}

/**
 * Enables or disabled the heating element.
 *
 * @param h True to enable the heater, False to disable it.
 */
void RP2040_SHT31_hw::heater(bool h) {
  if (h)
    writeCommand(SHT31_HEATEREN);
  else
    writeCommand(SHT31_HEATERDIS);
  sleep_us(1000);
}

/*!
 *  @brief  Return sensor heater state
 *  @return heater state (TRUE = enabled, FALSE = disabled)
 */
bool RP2040_SHT31_hw::isHeaterEnabled() {
  uint16_t regValue = readStatus();
  return regValue&SHT31_REG_HEATER_BIT>0;
}

