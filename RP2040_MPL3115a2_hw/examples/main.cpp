

#include "pico/stdlib.h"
#include  "pico/util/datetime.h"
#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>

#include <include/RP2040_MPL3115a2_hw.h>

#define MPL3115a2_PIN_SDA (uint)2
#define MPL3115a2_PIN_SCL (uint)3
#define INT1_PIN _u(16)

void gpio_callback(uint gpio, uint32_t events);

RP2040_MPL3115a2_hw MP3115a2 = RP2040_MPL3115a2_hw(i2c1);

float temp, humidity, pressure, pressureOffset;
DateTime now;
int main(){

    stdio_init_all();

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(MPL3115a2_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(MPL3115a2_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(MPL3115a2_PIN_SDA);
    gpio_pull_up(MPL3115a2_PIN_SCL);
    gpio_init(INT1_PIN);
    gpio_pull_up(INT1_PIN); // pull it up even more!
    
    sleep_ms(10);

    pressureOffset = 0.10;
    uint8_t cmd[2];

    if(!MP3115a2.begin(MPL3115A2_ADDRESS)){
      sleep_ms(10);
      printf("could not initialize MP3115A2!");
    }
    MP3115a2.setMode(MPL3115A2_BAROMETER);
    gpio_set_irq_enabled_with_callback(INT1_PIN, GPIO_IRQ_LEVEL_LOW, true, &gpio_callback);

    while(true){
        printf("pressure: %f\n",MP3115a2.data.pressure);
        sleep_ms(2000);
    }
}

void gpio_callback(uint gpio, uint32_t events) {
    // if we had enabled more than 2 interrupts on same pin, then we should read
    // INT_SOURCE reg to find out which interrupt triggered

    // we can filter by which GPIO was triggered
    if (gpio == INT1_PIN) {
        if(MP3115a2.conversionComplete()){
            MP3115a2.data.pressure = MP3115a2.getLastConversionResults(MPL3115A2_PRESSURE)*0.02953+pressureOffset;
            MP3115a2.has_new_data = true;
        }

    }
}

