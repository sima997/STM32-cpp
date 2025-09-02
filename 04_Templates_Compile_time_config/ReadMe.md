# Templates & Compile-time config
## Goals
- Learn how to use **C++ templates** and `constexpr` to configure peripherals **at compile time** instead of runtime
- See how this removes runtime overhead (important in embedded)
- Apply it by writing a **templated UART driver** where baudrate is a compile-time parameter.

## Theory 
### 1. Function and Class Templates
- Templates let you write **generic code** where the type or value is parameter.
- Example:
  ```cpp
  template<typename T>
  T add(T a, T b) { return a + b; }
  ```
- Value template parameters are also possible:
  ```cpp
  template<int N>
  int multiply(int x) {return N * x;}
  ```
Here, `N` is **known at compile time**


### 2. `constexpr` Config
- `constexpr` means "evaluated at compile time if possible"
- Example:
  ```cpp
  constexpr uint32_t clockHz = 170'000'000; //170 MHz
  constexpr uint32_t uartDivisor(uint32_t baudrate) {
    return clockHz / baudrate;
  }

  This ensure the compiler precalculates the divisor.

### 3. Why is this good for embedded?
- **Zero runtime cost** (no extra multiplications/divisions at runtime).
- **Type safety** (the compiler enforces correct parameters)
- **Optimized code** (each instantion is specialized)

## Practice Project – Templated UART
We'll make a **UART driver template** where:
- The **baudrate** is a template parameter (`template<int baudrate>`)
- The **USART peripheral** is a parameter (passed as pointer);
- Extra: Create a **GPIO driver template**