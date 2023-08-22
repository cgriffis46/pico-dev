

#include "pico/stdlib.h"
#include  "pico/util/datetime.h"
#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>
#include 
#define SHT31_PIN_SDA (uint)2
#define SHT31_PIN_SCL (uint)3

RP2040_SHT31_hw SHT31 = RP2040_SHT31_hw(i2c1);

float temp, humidity;
int main(){

    stdio_init_all();

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(SHT31_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(SHT31_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SHT31_PIN_SDA);
    gpio_pull_up(SHT31_PIN_SCL);
    
    uint8_t cmd[2];
    if(!SHT31.begin(SHT31_DEFAULT_ADDR)){
      printf("could not initialize SHT31\n");
    }
      
    while(true){
         
      if(SHT31.readTempHum(&temp, &humidity)){
           printf("temp: %f\n",temp);
           printf("humidity: %f\n",humidity);
           stdio_flush();
        }
        else {
          printf("could not read SHT31!\n");
        }
        sleep_ms(2000);
    }
}



