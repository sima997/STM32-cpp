# C++ for embedded systems
## 1. Languague Basics for Embedded C++
These are the "don't get caught" without them" fundamentals:
- **Const-correctness**
  - Use `const` to protect read-only data (often placed in flash memory)
  - `constexpr` for compile-time evaluation - avoid run time overhead
- **References vs Pointers**
  - References guaranteed binding to an onject (no `nullptr`), pointers when optional or for hardware register access
- **Enums and enum class**
  - Prefer `enum class` to avoid name pollution and implicit conversions
- **Inline & `static`**
  - `inline` for small, performance-critical functions
  - `static` at file scope to keep symbols private


## 2. Memory management & Safety
- **No dynamic allocation (or limited, controlled use)**
  - In embedded, `new`/`delete` are often banned or replaced with fixed-size allocators
- **Placement new**
  - Construct an object in pre-allocated memory region (e.g., a specific buffer)
- **Memory qualifiers (volatile)**
  - Use `volatile` for hardware registers or variables modified by ISRs
- **Struct packing & alignment**
  - `#pragma pack#` or `alignas()` to match hardware protocol layouts

## 3. Object-Oriented Principles (Lightweight)
- **Classes with no RTTI and no exceptions**
  - Usually `-fno-rtti` an `-fno-exceptions` for small firmware
- **Static polymorphism (CRTP)**
  - Avoid virtual tables when possible (saves RAM/Flash)
  - Use templates or constexpr to achieve compile-time polymorphism
- **Encapsulation**
  - Hide hardware registers behind clean interfaces

## 4. Compile-Time Programming
- **Templates**
  - For zero-cost abstractions (e.g., GPIO configuretion at compile time)
  - `constexpr` **functions**
  - Let the compiler precompute values (lookup tables, masks, etc.)
- **Type traits & `std::enable_if`/concepts**
  - Enforce compile-time constraints without runtime cost

## 5. Concurency & Interrupts
- **Interrupt-safe design**
  - Shared variables between ISR and main loop must be `volatile` + atomic or protected
- **Memory barriers**
  - `std::atomic_signam_fence`/hardware-specific barriers for multi-core or DMA sync
- **Bare-metal scheduling patterns**
  - Superloop, cooperative multitasking, or RTOS tasks

## 6. Hardware Interactions
- **Memory-mapped I/O**
    ```C++
    #define REG(x)  (*reinterpret_cast<volatile uint32_t*>(x))
    REG(0x40021000) = 0x01;
    ```
- **Bit manimulation helpers**
    ```C++
    constexpr uint32_t BIT(int n) {return 1u << n;}
    ```
- **Register structs**
    ```C++
    struct GPIO {
        volatile uint32_t MODER;
        volatile uint32_t OTYPER;
    };

    inline GPIO& gpio = *reinterpret_cast<GPIO*>(0x48000000);
    ```
  - **Why not `static_cast`?**
      ```C++
      static_cast<GPIO*>(0x48000000);
      ```
    - `static_cast` is for **safe, compile-time-checked conversions** (e.g., `int -> double`,base class -> derived class)
    - It *cannot cast from an integer (0x48000000) to a pointer type** - the compiler will reject int
    - This is why you can't use it for memory-mapped registers
  - **Why not C-style cast?**
    ```C++
    (GPIO*)0x48000000;
    ```
    - This works, but:
      - A C-style cast in C++ is **ambiguous**: it could mean `static_cast`, `reinterpret_cast`, or even `const_cast` depending on the concept
      - This makes it **harder to read and reason about** (and can hide bugs)
  - **Why `reinterpret_cast`?**
    ```C++
    reinterpret_cast<GPIO*>(0x48000000);
    ```
    - `reinterpret_cast` is the **only cast that explicitly means:**
        
        **"I know what I'm doing - treat these bits as if they were of another type"**
    - It's exacly what we want for hardware programming:
      - We know that `0x48000000` is the start address of peripheral register block
      - We want to **reinterpret** that address to a `GPIO` struct
    
    **Possible use in production firmware**
    ```C++
    constexpr GPIO& getGPIO() {
        return *reinterpret_cast<GPIO*>(0x48000000);
    }
    ```
    ```C++
    getGPIO().MODER = 0x55555555;
    ```

## 7. Toolchain & Build Considerations
- **Linker scripts**
  - Control where code/data lives (flash vs RAM)
- **Startup code**
  - C++ static constructors run before `main()` - be aware of their cost
- **`constexpr` & `inline` reduce flash use**
  - Fewer global symbols -> smaller image

## 8. Modern C++ Features Worth Using
- `enum class` - safer enums
- `constexpr` - compile-time work
- `[[nodiscard]]` - warn if a return value is ignored
- **Structured bindings** - nice for parsing data without extra variables
- `span` **(C++20)** - safe array view without heap

## Examples

**Example: Core Syntax & Qualifiers**
```C++
cpp

const int val = 42;         //Read-only (often in flash)
constexpr int mask() {return 1 << 3; }  //Compile-time calculation
volatile uint32_t reg;      //Prevents compiler from optimizing out accesses
static int counter = 0;     //File or function scope only
inline int add(int a, int b) {return a+b;} //No call overhead
```

**Example: Pointers, References & Structs**
```C++
cpp

int a = 10;
int* p = &a;    //Pointer
int& r = a;     //Reference (never null)

//Memory-mapped register
#define REG32(addr) (*reinterpret_cast<volatile uint32_t*>(addr))
REG32(0x4002100) = 0x01;

//Hardware register struct

struct GPIO {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
};
inline GPIO& gpio = *reinterpret_cast<GPIO*>(0x48000000);
gpio.MODER = 0x55555555;
```

**Example: Enums & Type Safety**
```C++
cpp

enum class Mode : uint8_t {Input, Output};
Mode m = Mode::Input;
```

**Example: Templates & Compile-Time Tricks**
```C++
template<int PIN> struct Gpio {
    static void set() {/*set bit PIN*/}
};
Gpio<5>::set(); //Compiled for pin 5 only

constexpr int lut[4] = {0,1,1,2}; //Precomputed lookup table
```

**Example: Interrupt Safety**
```C++
cpp

volatile bool flag = false;

extern "C" void ISR_Handler() {
    flag = true;    //volatile prevents caching
}

int main() {
    while(!flag) {/*wait*/}
}
```

**Example: Bit Manipulation**
```C++
cpp

constexpr uint32_t BIT(int n) {return 1u << n; }
reg |= BIT(3);  // Set bit 3
reg &= ~BIT(3); //Clear bit 3