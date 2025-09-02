#pragma once

#include <cstdint>
//#include "gpio.hpp"
#include "stm32g431xx.h"
#include "gpio_template.hpp"

template<int instance, int baudrate, typename TxPin, typename RxPin>
class Uart {
private:
    static USART_TypeDef* uart() {
        if constexpr(instance == 1) return USART1;
        else if constexpr(instance == 2) return USART2;
        else if constexpr(instance == 3) return USART3;
    }

public:
    static void init() {
        //Enable clock and configure gpio
        if constexpr(instance == 1) {
                RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
                //Gpio tx1(Gpio::A,9,Gpio::Mode::Alt,Gpio::AFx::AF7);
                //Gpio rx1(Gpio::A,10,Gpio::Mode::Alt,Gpio::AFx::AF7);
        } else if constexpr(instance == 2) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
                //Gpio tx2(Gpio::A,2,Gpio::Mode::Alt,Gpio::AFx::AF7);
                //Gpio rx2(Gpio::A,3,Gpio::Mode::Alt,Gpio::AFx::AF7);
        } else if constexpr(instance == 3) {
                RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
                //Gpio tx3(Gpio::B,9,Gpio::Mode::Alt,Gpio::AFx::AF7);
                //Gpio rx3(Gpio::B,8,Gpio::Mode::Alt,Gpio::AFx::AF7);
        }

        TxPin::init();
        RxPin::init();

        //Reset control register
        uart()->CR1 = 0;
        uart()->CR2 = 0;
        uart()->CR3 = 0;

        //Calculate baudrate
        constexpr uint32_t periph_clk = 170000000;
        constexpr uint32_t brr = periph_clk/baudrate;
        uart()->BRR = brr;

        

        //Enable TX, RX
        uart()->CR1 |= USART_CR1_RE | USART_CR1_TE;

        //Enable USART
        uart()->CR1 |= USART_CR1_UE;

    }

    static void sendChar(char c) {
        while(!(uart()->ISR & (USART_ISR_TXE_TXFNF)));
        uart()->TDR = c;
    }

    static void send(const char* str) {
        while(*str) sendChar(*str++);
    }



};