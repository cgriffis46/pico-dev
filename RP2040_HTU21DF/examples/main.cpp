
#define RP2040

#include "pico/stdlib.h"
#include  "pico/util/datetime.h"
#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>
#include "math.h"
#include <include/RP2040_HTU21DF_hw.h>

Adafruit_HTU21DF htu21df(i2c1);

#define INT1_PIN _u(16)

void gpio_callback(uint gpio, uint32_t events);

int main(){
    float temperature, humidity; 

    stdio_init_all();

    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);
    i2c_init(i2c1, 100 * 1000);

    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
    i2c_init(i2c0, 100 * 1000);

    htu21df.begin(HTU21DF_I2CADDR);

    while(true){
        temperature = htu21df.readTemperature();
        humidity = htu21df.readHumidity();
        printf("temp: %f\n",temperature);
        printf("humidity: %f\n",humidity);

        sleep_ms(2000);
    }
}

void gpio_callback(uint gpio, uint32_t events) {
    // if we had enabled more than 2 interrupts on same pin, then we should read
    // INT_SOURCE reg to find out which interrupt triggered

    // we can filter by which GPIO was triggered
    if (gpio == INT1_PIN) {
    
    }
}