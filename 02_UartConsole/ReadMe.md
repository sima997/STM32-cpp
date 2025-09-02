# Encapsulation & Drivers
## Goals
- Learn **encapsulation** by hiding hardware details inside a class.
- Write a `Uart` class that initializes UART, and provides `send` / `print` methods.
- Practice **constructor/destructor** roles in embedded.
- Understand **singleton pattern** (only one UART driver per hardware block).

## Theory 
### 1. Encapsulation
Encapsulation = hide registers and low-level config inside a class so user code looks clean:
```cpp
Uart uart1(USART1, 115200);
uart1.print("Hello!\r\n");
```
Instead of having `USART1->BRR = ...` scattered all over.
   

### 2. Constructors & Destructors
- **Constructor:** Enable peripheral clock, configure pins, set baudrate, enable UART.
- **Destructor** (rarely used in bare-metal): Could disable clock, reset registers (only useful if you want to "release" hardware).
- On embedded, usually only constructor does work, destructor is trivial. 

### 3. Singleton pattern
Each hardware peripheral exists **once** in the MCU. You don't want two `Uart` objects controling the same `USART1`.

Method using only **constexpr + header-only singleton** is better so that the compiler can completely optimize it away for each USART you use. More complex to implement

Options:
1. Hide constructor, expose `getInstance()`.
2. Or: allow construction but document "only one per peripheral"


## Practice Project – UART Console
### Project idea
- Create `Uart` class (no malloc)
- Send `"Hello C++ Embedded!"` every second
- Add method `print(const char*)`
- Add `newline()` method that sends `\r\n`
- Add `sendInt(int val)` that converts integer to decimal string and prints
- Implement `Uart::receiveChar()`(polling RXNE flag)
- Add second UART (e.g., `USART2` on PA2/PA3). Try instatiating `Uart uart2(USART2,9600)`.