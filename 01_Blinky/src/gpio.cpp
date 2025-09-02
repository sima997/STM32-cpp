#include "gpio.hpp"

//Assign GPIO ports to addresses
GPIO_TypeDef* Gpio::A = reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE);
GPIO_TypeDef* Gpio::B = reinterpret_cast<GPIO_TypeDef*>(GPIOB_BASE);

//Clas constructor with initialization
Gpio::Gpio(GPIO_TypeDef* port, uint8_t pin, Gpio::Mode mode)
    : port_(port), pin_(pin)
{
    //Enable clock to selected port
    enable_clock(port_);

    //Set pin mode
    port_->MODER &= ~(0b11u << (pin_ * 2));  // clear
    port_->MODER |=  (static_cast<uint32_t>(mode) << (pin_ * 2));  // set

}

void Gpio::set() {
    port_->BSRR = (1u << pin_);
}

void Gpio::clear() {
    port_->BSRR = (1u << (pin_ + 16));
}

void Gpio::toggle() {
    port_->ODR ^= (1u << pin_);
}




void Gpio::enable_clock(GPIO_TypeDef* port) {
        if (port == Gpio::A) {
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
        } else if(port == Gpio::B) {
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
        }
    }