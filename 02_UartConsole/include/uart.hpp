#pragma once

#include <cstdint>
#include "stm32g431xx.h"
#include "gpio.hpp"

class Uart {

    enum class Parity {None, Even, Odd};

public :
    //Access unique instance for a given uart
    static Uart& getInstance(USART_TypeDef* usart, uint32_t baudrate = 115200, Parity parity = Parity::None);

    void sendChar(const char c);
    void send(const char* str);
    void sendInt(int val);
    void print(const char* str);
    void newline();
    char receiveChar();



private :
    USART_TypeDef* uart_;

    //Private constructor => Cannot be called directly
    Uart(USART_TypeDef* usart, uint32_t baudrate, Parity parity);

    void init(uint32_t baudrate, Parity parity);

    void intToStr(int value, char* buffer);
    
    // No copying allowed
    Uart(const Uart&) = delete;
    Uart& operator=(const Uart&) = delete;

};