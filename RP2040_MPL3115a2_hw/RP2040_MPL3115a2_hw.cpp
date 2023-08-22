#include "include/RP2040_MPL3115a2_hw.h"
#include <stdint.h>
#include "hardware/i2c.h"
#include "hardware/gpio.h"

RP2040_MPL3115a2_hw::RP2040_MPL3115a2_hw(i2c_inst_t *i2c){
    this->i2c = *i2c;
}
bool RP2040_MPL3115a2_hw::begin(uint8_t devAddress){
    uint8_t cmd[3]; 
    uint8_t whoami;
    this->devAddress = MPL3115A2_ADDRESS;
    //sanity check
    cmd[0] = MPL3115A2_WHOAMI;
    i2c_write_blocking(&this->i2c,this->devAddress,cmd,1,true);
    i2c_read_blocking(&this->i2c,this->devAddress,&whoami,1,false);
    if (whoami != 0xC4) {
        return false;
    }
    // software reset
    uint8_t resetReg = MPL3115A2_CTRL_REG1_RST;
    cmd[0] = MPL3115A2_CTRL_REG1;
    cmd[1] = MPL3115A2_CTRL_REG1_RST;
    i2c_write_blocking(&this->i2c,this->devAddress,cmd,2,false);
    sleep_ms(10);
    while(resetReg == MPL3115A2_CTRL_REG1_RST){
        i2c_write_blocking(&this->i2c,this->devAddress,cmd,1,false);
        i2c_read_blocking(&this->i2c,this->devAddress,&resetReg,1,false);
    }
    // set oversampling and altitude mode
    currentMode = MPL3115A2_ALTIMETER;
    _ctrl_reg1.reg = MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT;
    cmd[0] = MPL3115A2_CTRL_REG1;
    cmd[1] = _ctrl_reg1.reg;
    i2c_write_blocking(&this->i2c,this->devAddress,cmd,2,false);
    // enable data ready events for pressure/altitude and temperature
    cmd[0] = MPL3115A2_PT_DATA_CFG;
    cmd[1] = MPL3115A2_PT_DATA_CFG_TDEFE | MPL3115A2_PT_DATA_CFG_PDEFE | MPL3115A2_PT_DATA_CFG_DREM;
    i2c_write_blocking(&this->i2c,this->devAddress,cmd,2,false);
    sleep_ms(10);

    return true;
}

float RP2040_MPL3115a2_hw::getPressure(void){
    if (currentMode != MPL3115A2_BAROMETER)
        setMode(MPL3115A2_BAROMETER);
    startOneShot();
    while (!conversionComplete())
        sleep_ms(10);
    return getLastConversionResults(MPL3115A2_PRESSURE);

}
float RP2040_MPL3115a2_hw::getAltitude(void){
    if (currentMode != MPL3115A2_ALTIMETER)
        setMode(MPL3115A2_ALTIMETER);
    startOneShot();
    while (!conversionComplete())
        sleep_ms(10);
    return getLastConversionResults(MPL3115A2_ALTITUDE);
}

int8_t RP2040_MPL3115a2_hw::getAltitudeOffset(void){
    uint8_t AltitudeOffset;
    uint8_t reg = MPL3115A2_OFF_H;
    i2c_write_blocking(&this->i2c,this->devAddress,&reg,1,true);
    i2c_read_blocking(&this->i2c,this->devAddress,&AltitudeOffset,1,true);
    return int8_t(AltitudeOffset);
}
void RP2040_MPL3115a2_hw::setAltitudeOffset(int8_t offset){
    uint8_t reg[2];
    reg[0] = MPL3115A2_OFF_H;
    reg[1] = (uint8_t)offset;
    i2c_write_blocking(&this->i2c,this->devAddress,reg,2,false);
}
float RP2040_MPL3115a2_hw::getTemperature(void){
    startOneShot();
    while (!conversionComplete())
    sleep_ms(10);
    return getLastConversionResults(MPL3115A2_TEMPERATURE);
}
void RP2040_MPL3115a2_hw::setSeaPressure(float SLP){
      // multiply by 100 to convert hPa to Pa
  // divide by 2 to convert to 2 Pa per LSB
  // convert to integer
  uint16_t bar = SLP * 50;

  // write result to register
  uint8_t buffer[3];
  buffer[0] = MPL3115A2_BAR_IN_MSB;
  buffer[1] = bar >> 8;
  buffer[2] = bar & 0xFF;
  i2c_write_blocking(&this->i2c,this->devAddress,buffer,3,false);
}
void RP2040_MPL3115a2_hw::setMode(mpl3115a2_mode_t mode){
    // assumes STANDBY mode
    uint8_t reg;
    uint8_t cmd[2];
    cmd[0] = MPL3115A2_CTRL_REG1;
    i2c_write_blocking(&this->i2c,this->devAddress,cmd,1,true);
    i2c_read_blocking(&this->i2c,this->devAddress,&reg,1,false);
    _ctrl_reg1.reg = reg;
    _ctrl_reg1.bit.ALT = mode;
    cmd[0] = MPL3115A2_CTRL_REG1;
    cmd[1] = _ctrl_reg1.reg;
    i2c_write_blocking(&this->i2c,this->devAddress,cmd,2,true);
    currentMode = mode;
}

void RP2040_MPL3115a2_hw::startOneShot(void){
    uint8_t reg;
    uint8_t cmd[2];
    // wait for one-shot to clear before proceeding
    cmd[0] = MPL3115A2_CTRL_REG1;
    i2c_write_blocking(&this->i2c,this->devAddress,cmd,1,true);
    i2c_read_blocking(&this->i2c,this->devAddress,&reg,1,false);
    _ctrl_reg1.reg = reg;

    while (_ctrl_reg1.bit.OST) {
        i2c_write_blocking(&this->i2c,this->devAddress,cmd,1,true);
        i2c_read_blocking(&this->i2c,this->devAddress,&reg,1,false);
        sleep_us(10);
        _ctrl_reg1.reg = reg;
  }
    // initiate one-shot measurement
    _ctrl_reg1.bit.OST = 1;
    cmd[0] = MPL3115A2_CTRL_REG1;
    cmd[1] = _ctrl_reg1.reg;
    i2c_write_blocking(&this->i2c,this->devAddress,cmd,2,false);
    sleep_ms(10);
}
bool RP2040_MPL3115a2_hw::conversionComplete(void){
    // PTDR bit works for either pressure or temperature
    // 0: No new set of data ready
    // 1: A new set of data is ready
    uint8_t reg = 0;
    uint8_t cmd[2];
    cmd[0] = MPL3115A2_REGISTER_STATUS;
    i2c_write_blocking(&this->i2c,this->devAddress,cmd,1,true);
    i2c_read_blocking(&this->i2c,this->devAddress,&reg,1,false);

    return (reg & MPL3115A2_REGISTER_STATUS_PTDR) != 0;
}
float RP2040_MPL3115a2_hw::getLastConversionResults(mpl3115a2_meas_t value){
    uint8_t buffer[5] = {MPL3115A2_REGISTER_PRESSURE_MSB, 0, 0, 0, 0};
    i2c_write_blocking(&this->i2c,this->devAddress,buffer,1,true);
    i2c_read_blocking(&this->i2c,this->devAddress,buffer,5,false);
    
    switch (value) {
        case MPL3115A2_PRESSURE:
            uint32_t pressure;
            pressure = uint32_t(buffer[0]) << 16 | uint32_t(buffer[1]) << 8 | uint32_t(buffer[2]);
            return float(pressure) / 6400.0;
        case MPL3115A2_ALTITUDE:
            int32_t alt;
            alt = uint32_t(buffer[0]) << 24 | uint32_t(buffer[1]) << 16 | uint32_t(buffer[2]) << 8;
            return float(alt) / 65536.0;
        case MPL3115A2_TEMPERATURE:
        default:
            int16_t t;
            t = uint16_t(buffer[3]) << 8 | uint16_t(buffer[4]);
        return float(t) / 256.0;
  }
}