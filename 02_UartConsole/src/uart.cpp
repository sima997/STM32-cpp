#include "uart.hpp"

//USART_TypeDef* Uart::USART1 = reinterpret_cast<USART_TypeDef*>(USART1_BASE);
//USART_TypeDef* Uart::USART2 = reinterpret_cast<USART_TypeDef*>(USART2_BASE);
//USART_TypeDef* Uart::USART3 = reinterpret_cast<USART_TypeDef*>(USART3_BASE);

/*
* This function creates a static class member based on USARTx, It uses singleton pattern to use peripheral only once.
* It is modified to be able to instantiate with different parity and baudrate
*/
Uart& Uart::getInstance(USART_TypeDef* usart, uint32_t baudrate, Parity parity) { 

    if(usart == USART1){
        static Uart uart1_instance(USART1, baudrate, parity);
        return uart1_instance;
    } else if(usart == USART2) {
        static Uart uart2_instance(USART2, baudrate, parity);
        return uart2_instance;
    }else if (usart == USART3) {
        static Uart uart3_instance(USART3, baudrate, parity);
        return uart3_instance;
    } else while(1); //Unsuported peripheral -> trap  
    
}

//Constructor
Uart::Uart(USART_TypeDef* usart, uint32_t baudrate, Parity parity)
    : uart_(usart)
{
    init(baudrate, parity);
};

/**
 * Main UART registers initialization
 */
void Uart::init(uint32_t baudrate, Parity parity) {

    /*
    Enable clock to uart
    USART1 -> RCC_APB2ENR bit 14
    USART2 -> RCC_APB1ENR1 bit 17
    USART3 -> RCC_APB1ENR1 bit 18
    */


   /*
    Enable Pins
    USART1 -> Tx = PA9 (AF7)  ; Rx = PA10 (AF7) 
    USART2 -> Tx = PA2 (AF7)  ; Rx = PA3 (AF7) 
    USART3 -> Tx = PB9 (AF7) ; Rx = PB8 (AF7) 
    */
   if(uart_ == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        Gpio tx1(Gpio::A,9,Gpio::Mode::Alt,Gpio::AFx::AF7);
        Gpio rx1(Gpio::A,10,Gpio::Mode::Alt,Gpio::AFx::AF7);
   } else if(uart_ == USART2) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
        Gpio tx1(Gpio::A,2,Gpio::Mode::Alt,Gpio::AFx::AF7);
        Gpio rx1(Gpio::A,3,Gpio::Mode::Alt,Gpio::AFx::AF7);
   } else if(uart_ == USART3) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
        Gpio tx1(Gpio::B,9,Gpio::Mode::Alt,Gpio::AFx::AF7);
        Gpio rx1(Gpio::B,8,Gpio::Mode::Alt,Gpio::AFx::AF7);
   }

   uint32_t periph_clk = 170000000;
   uart_->BRR = periph_clk/baudrate;

   //Reset control register
   uart_->CR1 = 0;

   //Set parity
   if(parity == Parity::Even) {
        uart_->CR1 |= USART_CR1_PCE;
   }else if(parity == Parity::Odd) {
        uart_->CR1 |= USART_CR1_PCE | USART_CR1_PS;
   }

   //Enable TX, RX, USART
   uart_->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

}

/**
 * Sends one character
 */
void Uart::sendChar(const char c) {
    while(!(uart_->ISR & (USART_ISR_TXE_TXFNF)));
    uart_->TDR = c;

}

/**
 * Sends string ended with null terminator
 */
void Uart::send(const char* str) {
    while(*str) sendChar(*str++);
}

/**
 * Sends integer as ASCII
 */

void Uart::sendInt(int val){
    char buffer[12];
    intToStr(val, buffer);
    print(buffer);
}

/**
 * Prints string
 */

void Uart::print(const char* str) {
    send(str);
}
/**
 * Prints newline character
 */
void Uart::newline() {
    print("\r\n");
}

/**
 * Polling for a character
 */
char Uart::receiveChar() {
    while(!(uart_->ISR & USART_ISR_RXNE_RXFNE));
    return static_cast<char>(uart_->RDR);
}

/**
 * Converts Integer to String
 */
void Uart::intToStr(int value, char* buffer) {
    char tmp[12];
    int i = 0;
    bool negative = false;

    if(value < 0) {
        negative = true;
        value = -value;
    }

    do {
        tmp[i++] = '0' + (value % 10);
        value /= 10;
    } while (value > 0);

    int j = 0;
    if (negative) buffer[j++] = '-';

    while(i > 0) {
        buffer[j++] = tmp[--i];
    }
    buffer[j] = '\0';
}