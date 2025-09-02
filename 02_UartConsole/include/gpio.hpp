#pragma once

#include <cstdint>
#include "stm32g431xx.h"

class Gpio {
public:
    enum class Mode : uint32_t { Input=0b00, Output=0b01, Alt=0b10, Analog=0b11 };
    enum class AFx : uint32_t {AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, AF8, AF9, AF10, AF11, AF12, AF13, AF14, AF15};
    static GPIO_TypeDef* A;
    static GPIO_TypeDef* B;

    Gpio(GPIO_TypeDef* port, uint8_t pin, Mode mode, AFx af = AFx::AF0);

    void set();
    void clear();
    void toggle();

    
private:
    GPIO_TypeDef* port_;
    uint8_t pin_;

    static void enable_clock(GPIO_TypeDef* port);
    
};
