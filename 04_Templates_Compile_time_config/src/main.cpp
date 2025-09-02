#include "gpio_template.hpp"
#include "uart_template.hpp"
#include <cstdint>

/*
    Enable Pins
    USART1 -> Tx = PA9 (AF7)  ; Rx = PA10 (AF7) 
    USART2 -> Tx = PA2 (AF7)  ; Rx = PA3 (AF7) 
    USART3 -> Tx = PB9 (AF7) ; Rx = PB8 (AF7) 
    */
//Gpio teplates
using Tx1_pin = Gpio<'A', 9, GpioMode::Alt,GpioSpeed::Low,GpioPull::None,GpioAF::AF7>;
using Rx1_pin = Gpio<'A', 10, GpioMode::Alt,GpioSpeed::Low,GpioPull::None,GpioAF::AF7>;
using Tx3_pin = Gpio<'B', 9, GpioMode::Alt,GpioSpeed::Low,GpioPull::None,GpioAF::AF7>;
using Rx3_pin = Gpio<'B', 8, GpioMode::Alt,GpioSpeed::Low,GpioPull::None,GpioAF::AF7>;
using Led = Gpio<'A', 5, GpioMode::Output>;

//Uart templates
using Uart1_115200 = Uart<1,115200, Tx1_pin, Rx1_pin>;
using Uart3_9600 = Uart<3,9600, Tx3_pin, Rx3_pin>;


int main() {
    volatile unsigned i = 0;
    Uart1_115200::init();
    Uart3_9600::init();
    Led::init();


    while(1) {
        Uart1_115200::send("Hello fro template Uart 1!\r\n\0");
        Uart3_9600::send("Hello from template Uart 3!\r\n\0");
        Led::toggle();
       
        while(i<5000000) i++;
        i = 0;
                    
        
    }
}