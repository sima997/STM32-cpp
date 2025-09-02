#pragma once

#include <cstdint>
#include "stm32g431xx.h"

class Gpio {
public:
    enum class Mode : uint32_t { Input=0b00, Output=0b01, Alt=0b10, Analog=0b11 };
    static GPIO_TypeDef* A;
    static GPIO_TypeDef* B;

    Gpio(GPIO_TypeDef* port, uint8_t pin, Mode mode);

    void set();
    void clear();
    void toggle();

private:
    GPIO_TypeDef* port_;
    uint8_t pin_;

    static void enable_clock(GPIO_TypeDef* port);
};
