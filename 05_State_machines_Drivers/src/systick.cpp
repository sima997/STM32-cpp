#include "systick.hpp"
#include <cstdint>

volatile uint32_t g_ms = 0;

//Handler to systick interrupt
extern "C" void SysTick_Handler(void) {
     g_ms += 1; 
}

void systick_init(uint32_t core_clk_hz) {
    if(static_cast<uint32_t>(CtrlMask::CLKSRC) == 0) SysTick->LOAD = core_clk_hz/8000;
    else SysTick->LOAD = (core_clk_hz/1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 0;
    SysTick->CTRL = CtrlMask::ENABLE | CtrlMask::TICKINT | CtrlMask::CLKSRC;
}

uint32_t millis() { return g_ms; }

bool due(uint32_t &last, uint32_t period_ms) {
    uint32_t now = g_ms;
    uint32_t tmp = now - last;
    if (static_cast<uint32_t>(now - last) >= period_ms) {
        last = now;
        return true;
    }
    return false;
    
}