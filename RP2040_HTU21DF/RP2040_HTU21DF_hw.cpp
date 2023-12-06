/*!
 * @file Adafruit_HTU21DF.cpp
 *
 * @mainpage Adafruit HTU21DF Sensor
 *
 * @section intro_sec Introduction
 *
 * This is a library for the HTU21DF Humidity & Temp Sensor
 *
 * Designed specifically to work with the HTU21DF sensor from Adafruit
 * ----> https://www.adafruit.com/products/1899
 *
 * These displays use I2C to communicate, 2 pins are required to
 * interface
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#include "include/RP2040_HTU21DF_hw.h"
#include "hardware/i2c.h"
/**
 * Constructor for the HTU21DF driver.
 */
Adafruit_HTU21DF::Adafruit_HTU21DF(i2c_inst_t *i2c) {
    this->i2c = *i2c;
  /* Assign default values to internal tracking variables. */
  _last_humidity = 0.0f;
  _last_temp = 0.0f;

}

/**
 * Initialises the I2C transport, and configures the IC for normal operation.
 * @param theWire Pointer to TwoWire I2C object, uses &Wire by default
 * @return true (1) if the device was successfully initialised, otherwise
 *         false (0).
 */
bool Adafruit_HTU21DF::begin(uint8_t devAddress = HTU21DF_I2CADDR) {
  uint8_t readbuffer;
  uint8_t cmd = HTU21DF_READREG;
  this->devAddress = devAddress;

  reset();

//  Adafruit_BusIO_Register reg =
//      Adafruit_BusIO_Register(i2c_dev, HTU21DF_READREG);

//  return (reg.read() == 0x2); // after reset should be 0x2
  i2c_write_blocking(&this->i2c, this->devAddress,&cmd, 1, false);
  i2c_read_blocking(&this->i2c,this->devAddress,&readbuffer, 1,false);

  return (readbuffer == 0x2);
}

/**
 * Sends a 'reset' request to the HTU21DF, followed by a 15ms delay.
 */
void Adafruit_HTU21DF::reset(void) {
  uint8_t cmd = HTU21DF_RESET;
  //i2c_dev->write(&cmd, 1);

  i2c_write_blocking(i2c1, this->devAddress, &cmd, 1, false);

  sleep_ms(15);
  //delay(15);
}

/**
 * Performs a single temperature conversion in degrees Celsius.
 *
 * @return a single-precision (32-bit) float value indicating the measured
 *         temperature in degrees Celsius or NAN on failure.
 */
float Adafruit_HTU21DF::readTemperature(void) {
  // OK lets ready!
  uint8_t cmd = HTU21DF_READTEMP;
    i2c_write_blocking(&this->i2c, this->devAddress, &cmd, 1, false);
  //if (!i2c_dev->write(&cmd, 1)) {
  //  return NAN;
  //}

  sleep_ms(50); // add delay between request and actual read!

  uint8_t buf[3];
  //if (!i2c_dev->read(buf, 3)) {
  //  return NAN;
  //}
  i2c_read_blocking(&this->i2c,this->devAddress,buf, 3,false);


  /* Read 16 bits of data, dropping the last two status bits. */
  uint16_t t = buf[0];
  t <<= 8;
  t |= buf[1] & 0b11111100;

  // 3rd byte is the CRC

  float temp = t;
  temp *= 175.72f;
  temp /= 65536.0f;
  temp -= 46.85f;

  /* Track the value internally in case we need to access it later. */
  _last_temp = temp;

  return temp;
}

/**
 * Performs a single relative humidity conversion.
 *
 * @return A single-precision (32-bit) float value indicating the relative
 *         humidity in percent (0..100.0%).
 */
float Adafruit_HTU21DF::readHumidity(void) {
  /* Prepare the I2C request. */
  uint8_t cmd = HTU21DF_READHUM;
//  if (!i2c_dev->write(&cmd, 1)) {
//    return NAN;
//  }

  i2c_write_blocking(&this->i2c, this->devAddress, &cmd, 1, false);

  /* Wait a bit for the conversion to complete. */
  sleep_us(50000);


  uint8_t buf[3];
//  if (!i2c_dev->read(buf, 3)) {
//    return NAN;
//  }
  i2c_read_blocking(&this->i2c,this->devAddress,buf,3,false);

  /* Read 16 bits of data, dropping the last two status bits. */
  uint16_t h = buf[0];
  h <<= 8;
  h |= buf[1] & 0b11111100;

  // 3rd byte is the CRC

  float hum = h;
  hum *= 125.0f;
  hum /= 65536.0f;
  hum -= 6.0f;

  /* Track the value internally in case we need to access it later. */
  _last_humidity = hum;

  return hum;
}
