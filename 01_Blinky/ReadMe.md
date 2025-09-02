# C++ Warm-Up & Toolchain
## Goals
- Refresh essential C++ features that matter for embedded.
- Understand `volatile` and why it matters for hardware.
- Write your first **OOP-style GPIO class** for STM32.
- Blink an LED using C++ instead of C.

## Theory 
### 1. C++ essentials for embedded
   - **Classes** = bundle data + functions
   - **Constructors** = run at object creation, good for peripheral init
   - **Destructors** = usually empty in embedded (no dynamic cleanup)
   - **Encapsulation** = hide HAL/CMSIS details inside methods
```cpp
class Led {
public:
    Led() {}                  // constructor
    void on();                // turn on
    void off();               // turn off
private:
    // hardware pin details
};
```

### 2. `volatile`
- Used for hardware registers or variables modified in ISRs.
- Without it, compiler optimizes reads/writes incorrectly.
```cpp
volatile uint32_t* gpio_odr = reinterpret_cast<volatile uint32_t*>(0x48000014);
*gpio_odr |= (1 << 5);
```

### 3. Memory model
- **.text** = flash code
- **.data** = initialized globals
- **.bss** = zero-initialized globals
- **stack** = function calls/local vars
- **heap** = avoid in embedded (new/malloc risky)

### 4. Object-Oriented GPIO
- In C, you do:
```C
c

HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
```
- In C++:
```cpp
cpp

Led led(GPIOA, GPIO_PIN_5);
led.on();
```
  - Cleaner, reusable and testable


## Practice Project – Blinker++
### Project idea
- Create a `Gpio` class (constructor sets pin mode, methods for set/reset/toggle)
- Create and `Led` class using `Gpio`
- Blink LED every 500 ms