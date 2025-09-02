#pragma once
#include <cstdint>
#include "stm32g431xx.h"
//NOTE: Systick runs on ckl = HCLK/8 by default

struct CtrlMask {
    static constexpr uint32_t ENABLE = (1UL << SysTick_CTRL_ENABLE_Pos); //0-HCLK/8, 1=HCLK
    static constexpr uint32_t TICKINT = (1UL << SysTick_CTRL_TICKINT_Pos); //0 = no exception request, 1= exception request enable
    static constexpr uint32_t CLKSRC = (1UL << SysTick_CTRL_CLKSOURCE_Pos); //0 = counter disabled, 1= counter enabled
};

void systick_init(uint32_t core_clk_hz);
uint32_t millis();
bool due(uint32_t &last, uint32_t period_ms); //true when periode elapsed