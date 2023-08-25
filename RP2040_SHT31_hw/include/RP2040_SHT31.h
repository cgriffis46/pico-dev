#ifndef _RP2040_SHT31_hw
#define _RP2040_SHT31_hw

#include <stdint.h>
#include "hardware/i2c.h"

#define SHT31_DEFAULT_ADDR 0x44 /**< SHT31 Default Address */
#define SHT31_MEAS_HIGHREP_STRETCH                                             \
  0x2C06 /**< Measurement High Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_HIGHREP_STRETCH_MSB 0x2C
#define SHT31_MEAS_HIGHREP_STRETCH_LSB 0x06
#define SHT31_MEAS_MEDREP_STRETCH                                              \
  0x2C0D /**< Measurement Medium Repeatability with Clock Stretch Enabled */
#define SHT31_MEAS_LOWREP_STRETCH                                              \
  0x2C10 /**< Measurement Low Repeatability with Clock Stretch Enabled*/
#define SHT31_MEAS_HIGHREP                                                     \
  0x2400 /**< Measurement High Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_MEDREP                                                      \
  0x240B /**< Measurement Medium Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_LOWREP                                                      \
  0x2416 /**< Measurement Low Repeatability with Clock Stretch Disabled */
#define SHT31_MEAS_HIGHREP_MSB 0x24
#define SHT31_MEAS_HIGHREP_LSB 0x00
#define SHT31_READSTATUS 0xF32D   /**< Read Out of Status Register */
#define SHT31_READSTATUS_MSB 0xF3   /**< Read Out of Status Register */
#define SHT31_READSTATUS_LSB 0x2D   /**< Read Out of Status Register */
#define SHT31_CLEARSTATUS 0x3041  /**< Clear Status */
#define SHT31_SOFTRESET 0x30A2    /**< Soft Reset */
#define SHT31_HEATEREN 0x306D     /**< Heater Enable */
#define SHT31_HEATERDIS 0x3066    /**< Heater Disable */
#define SHT31_REG_HEATER_BIT 0x0d /**< Status Register Heater Bit */
#define SHT31_SOFTRESET_MSB 0x30
#define SHT31_SOFTRESET_LSB 0xA2

#define SHT31_FETCH_DATA_MSB 0xE0
#define SHT31_FETCH_DATA_LSB 0x00

#define SHT31_PERIODIC_05mps_HIGHREP_MSB 0x20
#define SHT31_PERIODIC_05mps_HIGHREP_LSB 0x32

#define SHT31_PERIODIC_05mps_MEDREP_MSB 0x20
#define SHT31_PERIODIC_05mps_MEDREP_LSB 0x24

#define SHT31_PERIODIC_05mps_LOWREP_MSB 0x20
#define SHT31_PERIODIC_05mps_LOWREP_LSB 0x2F

#define SHT31_PERIODIC_1mps_HIGHREP_MSB 0x21
#define SHT31_PERIODIC_1mps_HIGHREP_LSB 0x30

#define SHT31_PERIODIC_1mps_MEDREP_MSB 0x21
#define SHT31_PERIODIC_1mps_MEDREP_LSB 0x26

#define SHT31_PERIODIC_1mps_LOWREP_MSB 0x21
#define SHT31_PERIODIC_1mps_LOWREP_LSB 0x2D

#define SHT31_PERIODIC_2mps_HIGHREP_MSB 0x22
#define SHT31_PERIODIC_2mps_HIGHREP_LSB 0x36

#define SHT31_PERIODIC_2mps_MEDREP_MSB 0x22
#define SHT31_PERIODIC_2mps_MEDREP_LSB 0x20

#define SHT31_PERIODIC_2mps_LOWREP_MSB 0x22
#define SHT31_PERIODIC_2mps_LOWREP_LSB 0x2B

#define SHT31_PERIODIC_4mps_HIGHREP_MSB 0x23
#define SHT31_PERIODIC_4mps_HIGHREP_LSB 0x34

#define SHT31_PERIODIC_4mps_MEDREP_MSB 0x23
#define SHT31_PERIODIC_4mps_MEDREP_LSB 0x22

#define SHT31_PERIODIC_4mps_LOWREP_MSB 0x23
#define SHT31_PERIODIC_4mps_LOWREP_LSB 0x29

#define SHT31_PERIODIC_10mps_HIGHREP_MSB 0x27
#define SHT31_PERIODIC_10mps_HIGHREP_LSB 0x37

#define SHT31_PERIODIC_10mps_MEDREP_MSB 0x27
#define SHT31_PERIODIC_10mps_MEDREP_LSB 0x21

#define SHT31_PERIODIC_10mps_LOWREP_MSB 0x27
#define SHT31_PERIODIC_10mps_LOWREP_LSB 0x2A

class RP2040_SHT31_hw {
public:
    RP2040_SHT31_hw(i2c_inst_t *i2c);
    bool begin(uint8_t devAddress);
    bool readTempHum(float* t, float* h);
    void heater(bool h);
    void reset();
    uint16_t readStatus(void);
    bool isHeaterEnabled();
    void PeriodicMode();
    bool callback(float *t, float *h);
private:
    float temp = 0;
    float humidity = 0;
    bool writeCommand(uint16_t command);
    static uint8_t crc8(const uint8_t *data, int len);
protected:
    i2c_inst_t i2c;
    uint8_t devAddress = SHT31_DEFAULT_ADDR;
    uint SDA_PIN = 2;
    uint SCL_PIN = 3;
};
#endif