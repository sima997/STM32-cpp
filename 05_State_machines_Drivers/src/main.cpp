#include "gpio_template.hpp"
#include "uart_template.hpp"
#include "i2c_template_new.hpp"
//#include "spi_template.hpp"
#include "systick.hpp"
#include "arduino_sensor.hpp"
#include "sensor_fsm.hpp"
#include <cstdint>

/*
    Enable Pins
    USART1 -> Tx = PA9 (AF7)  ; Rx = PA10 (AF7) 
    USART2 -> Tx = PA2 (AF7)  ; Rx = PA3 (AF7) 
    USART3 -> Tx = PB9 (AF7) ; Rx = PB8 (AF7) 
    */
//Gpio teplates
using Led = Gpio<'A', 5, GpioMode::Output>;

//Uart templates
using Uart1_115200 = Uart<1,115200>;
using Uart3_9600 = Uart<3,9600>;

//I2C templates
using I2C1Bus = I2C<1, 128'000'000, 100'000>;

//SPI templates
//using Spi1 = Spi<1,true>;

//Arduino sensor
//ArduinoSensor sensorAR97(0x53,0x42);




int main() {
    const uint32_t SystemClock = 128'000'000;
    Led led1;
    Uart1_115200 uart1;
    I2C1Bus i2c1;
    ArduinoSensor sensorAR97(i2c1,0x53,0x42);
    SensorFSM state = SensorFSM::INIT;

    uart1.init(SystemClock);
    led1.init();

    uint32_t sensor_out = 0;

    //Initialize SysTick
    systick_init(128'000'000);
    uint32_t last = millis();

    __enable_irq();


    while(1) {
      fsm_step(state, sensorAR97,led1, uart1);
    }
}