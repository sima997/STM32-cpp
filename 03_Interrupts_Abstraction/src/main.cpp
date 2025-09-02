#include "gpio.hpp"
#include "led.hpp"
#include "uart.hpp"
#include "button.hpp"
#include <cstdint>

#define RCC ((RCC_TypeDef *) RCC_BASE)

class MyButtonHandler : public IButtonHandler {
    public:
        void onButtonPressed() override{
            Led led(Gpio::A,5);
            led.toggle();
        }

        void onButtonRelease() override{

        }
};

int main() {
       
    //Gpio clk(Gpio::A,8,Gpio::Mode::Alt);

    //Uart& uart1 = Uart::getInstance(USART1,115200);
   
    MyButtonHandler handler;
    Button btn(13, &handler);

    
    //Clock confirmation on PA8
    RCC->CFGR &= ~(0b111 << 28); //Prescaler clear
    RCC->CFGR |= (0b011 << 28); //Prescaler set (8)
    RCC->CFGR &= ~(0b1111 << 24); //Output source clear
    RCC->CFGR |= (0b0001 << 24); //Output source set (sysClk)

    while(1) {
        
                
        
    }
}