# Interrupts & Abstraction
## Goals
- Learn how to handle ISRs in C++

## Theory 
### 1. `extern "C"`and C++ ISRs
- STM32 startup files (`startup_stm32g431xx.s) expect **C linkage** for interrupt handlers
- In C++ you must declare them like:
```cpp
extern "C" void EXTI15_10_IRQHandler(void) {
    //interrupt code
}
```
- **Why?:** Because without `extern "C"`, the compiler mangles the function name (`_Z17EXTI15_10_IRQHandlerv`), and the vector table can't find it.
  - - C++ changes function name due to overloadin, namespaces, templates, etc. C not
   

### 2. Observer pattern for events
- Problem: You don't want all interrupt logic inside ISR -> too messy.
- Solution: **Observer pattern** -> ISR notifies a "listener" (callback)

Example idea:
```cpp

class IButtonHandler {
public :
    virtual void onButtonPressed() = 0;
};
```
- `Button` keeps a pointer to such handler and calls it when EXTI fires

### Interface design (`IButtonHandler`)
- Define a clean interface for any class that reacts to a button press
- Example: `LedController` implements it, so button press -> toggles LED


## Practice Project – Button + LED controller
### Project idea
- Wrap EXTI (interrupt) into `Button` class
- Register a callback in C++ style
- On button (pin `PC13`) press, toggle LED (pin `PA5`)
  