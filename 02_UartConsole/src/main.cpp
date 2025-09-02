#include "gpio.hpp"
#include "led.hpp"
#include "uart.hpp"
#include <cstdint>

#define RCC ((RCC_TypeDef *) RCC_BASE)

int main() {
    volatile uint32_t i = 0;
    
    Led led(Gpio::A, 5);
    Gpio clk(Gpio::A,8,Gpio::Mode::Alt);

    Uart& uart1 = Uart::getInstance(USART1,115200);
    Uart& uart3 = Uart::getInstance(USART3,9600);

    const char message[50] = "Hello C++ Embedded!\0";

    //Clock confirmation on PA8
    RCC->CFGR &= ~(0b111 << 28); //Prescaler clear
    RCC->CFGR |= (0b011 << 28); //Prescaler set (8)
    RCC->CFGR &= ~(0b1111 << 24); //Output source clear
    RCC->CFGR |= (0b0001 << 24); //Output source set (sysClk)

    while(1) {
        
        i = 0;
        led.on();
        uart1.print(message);
        uart1.newline();
        uart1.sendInt(-22);
        uart1.newline();
        char rec = uart1.receiveChar();
        uart1.newline();
        uart1.print("Received char: \0");
        uart1.sendChar(rec);
        uart1.newline();

        uart3.print("Hello from uart2!\0");
        uart3.newline();
        while(i<5000000) i++;
        
        
        led.off();
        while(i<10000000) i++;
        
        
        
    }
}