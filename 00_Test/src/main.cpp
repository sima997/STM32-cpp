#include <cstdint>
#include "system_stm32g4xx.h"

// RCC (clock control)
constexpr uint32_t RCC_BASE   = 0x40021000;
volatile uint32_t* const RCC_AHB2ENR = reinterpret_cast<uint32_t*>(RCC_BASE + 0x4C);

// GPIOA
constexpr uint32_t GPIOA_BASE = 0x48000000;
volatile uint32_t* const MODER = reinterpret_cast<uint32_t*>(GPIOA_BASE + 0x00);
volatile uint32_t* const ODR   = reinterpret_cast<uint32_t*>(GPIOA_BASE + 0x14);

/*
extern "C" void SystemInit() {
    // Empty: system clock config can go here if needed
}
*/

int main() {
    SystemInit();

    // 1. Enable GPIOA clock in RCC
    *RCC_AHB2ENR |= (1 << 0);  // Bit 0 = GPIOAEN

    // 2. Set PA5 as general purpose output (MODER[11:10] = 01)
    *MODER &= ~(0b11 << (5 * 2)); // clear mode bits for pin 5
    *MODER |=  (0b01 << (5 * 2)); // set as output

    // 3. Blink loop
    while (1) {
        *ODR |= (1 << 5); // toggle PA5
        for (volatile int i = 0; i < 1000000; ++i); // crude delay
    }
}
