#include "gpio.hpp"

//Assigne GPIO ports to addresses
GPIO_TypeDef* Gpio::A = reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE);
GPIO_TypeDef* Gpio::B = reinterpret_cast<GPIO_TypeDef*>(GPIOB_BASE);
GPIO_TypeDef* Gpio::C = reinterpret_cast<GPIO_TypeDef*>(GPIOC_BASE);
GPIO_TypeDef* Gpio::D = reinterpret_cast<GPIO_TypeDef*>(GPIOD_BASE);
GPIO_TypeDef* Gpio::E = reinterpret_cast<GPIO_TypeDef*>(GPIOE_BASE);
GPIO_TypeDef* Gpio::F = reinterpret_cast<GPIO_TypeDef*>(GPIOF_BASE);
GPIO_TypeDef* Gpio::G = reinterpret_cast<GPIO_TypeDef*>(GPIOF_BASE);

//Clas constructor with initialization
Gpio::Gpio(GPIO_TypeDef* port, uint8_t pin, Gpio::Mode mode, Gpio::AFx af)
    : port_(port), pin_(pin)
{
    //Enable clock to selected port
    enable_clock(port_);

    //Set pin mode
    port_->MODER &= ~(0b11u << (pin_ * 2));  // clear
    port_->MODER |=  (static_cast<uint32_t>(mode) << (pin_ * 2));  // set

    //Set Alternate function if asked
    if (mode == Mode::Alt) {
        if(pin <= 7 && pin_ > 0) {
            port_->AFR[0] &= ~ (GPIO_AFRL_AFSEL0_Msk << (pin_*4));
            port_->AFR[0] |= (static_cast<uint32_t>(af) << (pin_*4));
        } else if(pin_ > 7 && pin_ <= 15) {
            port_->AFR[1] &= ~ (GPIO_AFRL_AFSEL0_Msk << ((pin_-8)*4));
            port_->AFR[1] |= (static_cast<uint32_t>(af) << ((pin_-8)*4));
        }
    }

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
        } else if(port == Gpio::C) {
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
        } else if(port == Gpio::D) {
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
        } else if(port == Gpio::E) {
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
        } else if(port == Gpio::F) {
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
        } else if(port == Gpio::G) {
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
        }
}

void Gpio::enable_interrupt(IrqEdge edge) {
    uint8_t register_num = pin_ / 4;
    uint8_t register_offset = pin_ % 4;

    //Enable clock to syscfg if not enabled already
    if(!(RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN)) RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    //Map pin from port to a corresponding EXTI line
    if(port_ == GPIOA){
        SYSCFG->EXTICR[register_num] &= ~(SYSCFG_EXTICR1_EXTI0_Msk << (register_offset*4));
        SYSCFG->EXTICR[register_num] |= (0b0000 << (register_offset*4));
    } else if(port_ == GPIOB) {
        SYSCFG->EXTICR[register_num] &= ~(SYSCFG_EXTICR1_EXTI0_Msk << (register_offset*4));
        SYSCFG->EXTICR[register_num] |= (0b0001 << (register_offset*4));
    } else if(port_ == GPIOC) {
        SYSCFG->EXTICR[register_num] &= ~(SYSCFG_EXTICR1_EXTI0_Msk << (register_offset*4));
        SYSCFG->EXTICR[register_num] |= (0b0010 << (register_offset*4));
        //SYSCFG->EXTICR[register_num] = static_cast<uint32_t>(0x20);
    } else if(port_ == GPIOD) {
        SYSCFG->EXTICR[register_num] &= ~(SYSCFG_EXTICR1_EXTI0_Msk << (register_offset*4));
        SYSCFG->EXTICR[register_num] |= (0b0011 << (register_offset*4));
    } else if(port_ == GPIOE) {
        SYSCFG->EXTICR[register_num] &= ~(SYSCFG_EXTICR1_EXTI0_Msk << (register_offset*4));
        SYSCFG->EXTICR[register_num] |= (0b0100 << (register_offset*4));
    } else if(port_ == GPIOF) {
        SYSCFG->EXTICR[register_num] &= ~(SYSCFG_EXTICR1_EXTI0_Msk << (register_offset*4));
        SYSCFG->EXTICR[register_num] |= (0b0101 << (register_offset*4));
    } else if(port_ == GPIOG) {
        SYSCFG->EXTICR[register_num] &= ~(SYSCFG_EXTICR1_EXTI0_Msk << (register_offset*4));
        SYSCFG->EXTICR[register_num] |= (0b0110 << (register_offset*4));
    }

    //Configure trigger
    if (edge == IrqEdge::Rising) {
        //Clear rising and faling triggers registers
        EXTI->RTSR1 &= ~(1 << pin_);
        EXTI->FTSR1 &= ~(1 << pin_);

        //Set rising edge register
        EXTI->RTSR1 |= (1 << pin_);
    } else if(edge == IrqEdge::Faling) {
        //Clear rising and faling triggers registers
        EXTI->RTSR1 &= ~(1 << pin_);
        EXTI->FTSR1 &= ~(1 << pin_);

        //Set faling edge register
        EXTI->FTSR1 |= (1 << pin_);
    }

    //unmask the EXTI line (write 1 to unmask)
    EXTI->IMR1 |= (1 << pin_);

    //Enable NVIC line
    NVIC_EnableIRQ(EXTI15_10_IRQn);


}