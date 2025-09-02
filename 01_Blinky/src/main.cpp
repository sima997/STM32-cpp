#include "gpio.hpp"
#include "led.hpp"
#include "uart.hpp"
#include <cstdint>

#define RCC ((RCC_TypeDef *) RCC_BASE)

int main() {
    volatile uint32_t i = 0;
    
    Led led(Gpio::A, 5);
    Gpio clk(Gpio::A,8,Gpio::Mode::Alt);

    

    //Clock confirmation on PA8
    RCC->CFGR &= ~(0b111 << 28); //Prescaler clear
    RCC->CFGR |= (0b011 << 28); //Prescaler set (8)
    RCC->CFGR &= ~(0b1111 << 24); //Output source clear
    RCC->CFGR |= (0b0001 << 24); //Output source set (sysClk)

    while(1) {
        //led.toggle();
        //delay(80000000);
        i = 0;
        led.on();
        while(i<5000000) i++;
        
        
        led.off();
        while(i<10000000) i++;
        
        
        
    }
}