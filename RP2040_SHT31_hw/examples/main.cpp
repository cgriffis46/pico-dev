

#include "pico/stdlib.h"
//#include  "pico/util/datetime.h"
#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>

#include <include/RP2040_SHT31.h>
//#include <include/RP2040_MPL3115a2_hw.h>
//#include "hardware/pio.h"
//#include <include/RTClib.h>
#define SHT31_PIN_SDA (uint)2
#define SHT31_PIN_SCL (uint)3
#define INT1_PIN _u(16)
#define SHT31_PIN _u(17)

void SHT31_callback(uint gpio, uint32_t events);
void gpio_callback(uint gpio, uint32_t events);

RP2040_SHT31_hw SHT31 = RP2040_SHT31_hw(i2c1);
//RP2040_MPL3115a2_pio MP3115a2 = RP2040_MPL3115a2_pio(pio0,0);
//RP2040_MPL3115a2_hw MP3115a2 = RP2040_MPL3115a2_hw(i2c1);
//RTC_DS3231 rtc(i2c0);
float temp, humidity, pressure, pressureOffset;
DateTime now;
int main(){

    stdio_init_all();

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);
    gpio_init(INT1_PIN);
    gpio_pull_up(INT1_PIN); // pull it up even more!
    
    gpio_init(SHT31_PIN);
    gpio_pull_up(SHT31_PIN); // pull it up even more!
    // gpio_set_irq_enabled_with_callback(SHT31_PIN, GPIO_IRQ_LEVEL_LOW, true, &SHT31_callback);
    // gpio_set_irq_enabled_with_callback(SHT31_PIN, GPIO_IRQ_LEVEL_HIGH, true, &SHT31_callback);
    sleep_ms(10);

    gpio_init(0);
    gpio_set_dir(0,true);
    // gpio_pull_up(0);
    gpio_put(0,true);

    SHT31_Alert_t HighAlert;
    SHT31_Alert_t LowAlert;

    HighAlert.SetHumidity = 90;
    HighAlert.ClearHumidity = 85;
    HighAlert.SetTemp = 40;
    HighAlert.ClearTemp = 35;

    LowAlert.SetHumidity = 75;
    LowAlert.ClearHumidity = 80;
    LowAlert.SetTemp = 0;
    LowAlert.ClearTemp = 5;

    pressureOffset = 0.10;
    uint8_t cmd[2];
   // rtc.begin(0x68);
    if(!SHT31.begin(SHT31_DEFAULT_ADDR)){
        printf("could not initialize SHT31\n");
        //
        // SHT31.setLowAlert(&LowAlert);
        
    }
    else {
        
    }

SHT31.clearStatus();
SHT31.setHighAlert(&HighAlert);
SHT31.setLowAlert(&LowAlert);
SHT31.PeriodicMode(_1mps_high_Res);


   // if(!MP3115a2.begin(MPL3115A2_ADDRESS)){
   //   sleep_ms(10);
    //  printf("could not initialize MP3115A2!");
    //}
    //MP3115a2.setMode(MPL3115A2_BAROMETER);
   // gpio_set_irq_enabled_with_callback(INT1_PIN, GPIO_IRQ_LEVEL_LOW, true, &gpio_callback);

    while(true){
        //gpio_set_irq_enabled (INT1_PIN, GPIO_IRQ_LEVEL_LOW, false);

        if(SHT31.FetchData(&temp,&humidity)){
            printf("temp: %f\nhumidity: %f\n",temp,humidity);}     

        // printf("pressure: %f\n",MP3115a2.data.pressure);
        printf("SHT31 Status Byte: %x\n",SHT31.readStatus());
        //gpio_set_irq_enabled (INT1_PIN, GPIO_IRQ_LEVEL_LOW, true);
        sleep_ms(1000);
        if(gpio_get(SHT31_PIN)){
            printf("Alert Active\n");
    }
    else{
    }
    }
}

void gpio_callback(uint gpio, uint32_t events) {
    // if we had enabled more than 2 interrupts on same pin, then we should read
    // INT_SOURCE reg to find out which interrupt triggered

    // we can filter by which GPIO was triggered
    if (gpio == INT1_PIN) {
        // FIFO overflow interrupt
        // watermark bits set to 0 in F_SETUP reg, so only possible event is an overflow
        // otherwise, we would read F_STATUS to confirm it was an overflow
        //printf("FIFO overflow!\n");
        // drain the fifo
        //MP3115a2.mpl3115a2_read_fifo(MP3115a2.fifo_data);
        // read status register to clear interrupt bit
        //MP3115a2.mpl3115a2_read_reg(MPL3115A2_F_STATUS);
        if(MP3115a2.conversionComplete()){
            MP3115a2.data.pressure = MP3115a2.getLastConversionResults(MPL3115A2_PRESSURE)*0.02953+pressureOffset;
            // printf("pressure: %f\n",MP3115a2.data.pressure);
            MP3115a2.has_new_data = true;
        }
    
    }
    else if(gpio == SHT31_PIN){
        SHT31.FetchData(&temp, &humidity);
        printf("temp: %d  humidity %d\n", temp, humidity);    
    }
}

void SHT31_callback(uint gpio, uint32_t events) {

}