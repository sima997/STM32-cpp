#include "button.hpp"

extern "C" {

    //Interrupt handler for EXTI lines 10-15.
    void EXTI15_10_IRQHandler(void) {
        Button::handleIrq();
    }


}