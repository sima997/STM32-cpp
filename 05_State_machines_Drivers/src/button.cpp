#include "gpio.hpp"
#include "button.hpp"

IButtonHandler* Button::handler_ = nullptr; //Global handler since there is only one button.

Button::Button(uint8_t pin, IButtonHandler* handler)
    : pin_(pin)
{
    handler_ = handler;

    Gpio button_pin(Gpio::C,13,Gpio::Mode::Input);
    button_pin.enable_interrupt(Gpio::IrqEdge::Faling);

}

void Button::handleIrq() {
    if(EXTI->PR1 & (1 << 13)) {
        EXTI->PR1 = (1 << 13); //Clear pending
        if(handler_) handler_->onButtonPressed();
    }
}
