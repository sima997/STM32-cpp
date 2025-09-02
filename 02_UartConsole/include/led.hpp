#include "gpio.hpp"

class Led {
public:
    Led(GPIO_TypeDef* port,uint8_t pin)
        : gpio_(port, pin, Gpio::Mode::Output) {}

    void on() {gpio_.set();}
    void off() {gpio_.clear();}
    void toggle() {gpio_.toggle();}

private:
    Gpio gpio_;
};

