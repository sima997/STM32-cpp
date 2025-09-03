# ARM (STM32) C/C++ build system configuration

## CMakeLists.txt 
To create automatic build system generator, CMake is used. This file describes all configurations required for the build

```CMake
Cmake

cmake_minimum_required(VERSION 3.20)
project(stm32_cpp_baremetal LANGUAGES C CXX)

#Set the MCU and toolchain
set(MCU "cortex-m4")
set(CPU_FLAGS "-mcpu=${MCU} -mthumb -O2 -ffunction-sections -fdata-sections")
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#Set toolchain prefix if needed (Absolute paths included because on local version used)
set(CMAKE_C_COMPILER "C:/ST/GCC/bin/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "C:/ST/GCC/bin/arm-none-eabi-g++.exe")
set(CMAKE_OBJCOPY "C:/ST/GCC/bin/arm-none-eabi-objcopy.exe")

#Include directories
include_directories(include)

#Source files
set(SOURCES
    src/main.cpp 
    src/system_stm32g4xx.c
    src/crt_stubs.c     #newlib support
    src/startup_stm32g431xx.s 
)

#Enable assembly languague
enable_language(ASM)

#Add executables
add_executable(firmware.elf ${SOURCES})

#Linker script
target_link_options(firmware.elf PRIVATE
    -T${CMAKE_SOURCE_DIR}/linker/stm32f407xx.ld
    -Wl,-Map=firmware.map 
    -nostartfiles
    -Wl,--gc-sections
)

#Post-build: convert to .bit
add_custom_command(TARGET firmware.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary firmware.elf firmware.bin 
)
```

### 1. Minimum CMake version and project definition
- `cmake_minimum_required(VERSION 3.20)`
    Ensures that the CMake version used is at least 3.20. This avoids compatibility issues with newer CMake commands

- `project(stm32_cpp_baremetal LANGUAGES C CXX)` 
    Defines the project name (`stm32_cpp_baremetal`) and specifies that the project will use C and C++ languages.

### 2. MCU and compiler flags
- `set(MCU "cortex-m4")`
    Defines a variable `MCU` with the target microcontroller architecture
- `set(CPU_FLAGS "-mcpu=${MCU} -mthumb -O2 -ffunction-sections -fdata-sections")`
    Defines compiler flags:
    - `-mcpu=cortex-m4` -> target Cortex-M4 CPU
    - `-mthumb` -> generate Thumb instruction set (16-bit).
    - `-O2` -> optimization level 2
    - `-ffunction-sections -fdata-sections` -> put functions and data into separate sections (helps the linker remove unused code).
- `set(CMAKE_C_STANDARD 11)`
    Sets C language standard to C11
- `set(CMAKE_CXX_STANDARD 11)`
Sets C++ language standard to C++11.
- `set(CMAKE_CXX_STANDARD_REQUIRED ON)`
Forces CMake to require C++11; if the compiler doesn’t support it, it will fail.

### 3. Toolchain configuration
- Sets the **absolute paths** for the ARM GCC toolchain (if in system PATHS, can call only commands):
  - `CMAKE_C_COMPILER` → C compiler
  - `CMAKE_C_COMPILER` → C compiler
  - `CMAKE_OBJCOPY` → converts ELF to binary or hex formats after build

### 4. Include directories
- `include_directories(include)` adds `include` folder to the compiler’s include path.
Any `#include "file.h"` in sources will search this folder.

### 5. Source files
```cmake
    set(SOURCES
        src/main.cpp 
        src/system_stm32g4xx.c
        src/crt_stubs.c
        src/startup_stm32g431xx.s 
    )
```

- Creates a `SOURCES` variable listing all files to be compiled:
  - `main.cpp` → main C++ application
  - `system_stm32g4xx.c` → system initialization (clock, peripherals)
  - `crt_stubs.c` → C runtime stubs (minimal `newlib` support)
    - Newlib is a C standard library implementation for embedded systems. It provides things like:
      - Standard I/O (`printf`, `scanf`)
      - String and memory functions (`memcpy`, `strlen`)
      - Math library (`sin`, `sqrt`)
      - Basic C runtime support

        It’s designed for bare-metal environments (no OS), unlike glibc which assumes an operating system is present.
  - `startup_stm32g431xx.s` → assembly startup file (reset handler, vector table)

### 6. Enable assembly language support
```cmake
    enable_language(ASM)
```
- Tells CMake to compile `.s` (assembly) files. Needed for `startup_stm32g431xx.s`.

### 7. Create the executable
```cmake
add_executable(firmware.elf ${SOURCES})
```
- Builds the final **ELF binary** named `firmware.elf` from all SOURCES.

### 8. Linker options
```cmake
target_link_options(firmware.elf PRIVATE
    -T${CMAKE_SOURCE_DIR}/linker/stm32f407xx.ld
    -Wl,-Map=firmware.map 
    -nostartfiles
    -Wl,--gc-sections
)
```
- `-T${CMAKE_SOURCE_DIR}/linker/stm32f407xx.ld` → specifies the linker script. Defines memory layout, sections, and entry point
- `-Wl,-Map=firmware.map` → tells the linker to generate a map file showing memory usage
  - `-Wl`, tells the compiler: “Pass the following option(s) directly to the **linker** (ld)”.
  - Everything after the comma is forwarded as-is to the linker.
- `-nostartfiles` → do not use default C runtime startup files; we provide our own (CRT stubs and startup `.s`).
- `-Wl,--gc-sections` → remove **unused sections** to reduce firmware size.

### 9. Post-build: generate binary
```cmake
add_custom_command(TARGET firmware.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary firmware.elf firmware.bin 
)
```
- After building `firmware.elf`, this command runs objcopy to create a raw binary (`firmware.bin`).

-`O binary` → output format is **raw binary** (used for flashing to MCU).


## arm-gcc-toolchain.cmake
This is a **CMake toolchain file** — it tells CMake what compilers to use and what flags to apply for your cross-compilation target (STM32 with ARM GCC). Let’s go through it line by line

```cmake
# arm-gcc-toolchain.cmake - CMake toolchain for STM32 using CubeIDE GCC

# Target system
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Path to CubeIDE GCC (replace with your path)
set(TOOLCHAIN_BIN "C:/ST/GCC/bin")

# Compiler
set(CMAKE_C_COMPILER ${TOOLCHAIN_BIN}/arm-none-eabi-gcc.exe)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_BIN}/arm-none-eabi-g++.exe)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_BIN}/arm-none-eabi-gcc.exe)

# CPU / Architecture flags (example for Cortex-M4)
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=soft")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_FLAGS} -O2 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CPU_FLAGS} -O2 -Wall -fno-exceptions -fno-rtti")
```

### 1. File header comment
```cmake
# arm-gcc-toolchain.cmake - CMake toolchain for STM32 using CubeIDE GCC
```
- Just a comment for humans: this file configures CMake to use the ARM GCC toolchain that comes with STM32CubeIDE.

### 2. Target system definition
```cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
```
- `CMAKE_SYSTEM_NAME Generic` Tells CMake that the target system is a **bare-metal system**, not Linux/Windows/macOS.(If you set `Linux`, CMake would assume system libraries exist.)
- `CMAKE_SYSTEM_PROCESSOR arm`Tells CMake that the target CPU architecture is ARM. This helps tools and find-scripts know we’re cross-compiling.

### 3. Toolchain path
```cmake
# Path to CubeIDE GCC (replace with your path)
set(TOOLCHAIN_BIN "C:/ST/GCC/bin")
```
- Defines where the ARM GCC toolchain executables live.

### 4. Compiler selection
```cmake
set(CMAKE_C_COMPILER ${TOOLCHAIN_BIN}/arm-none-eabi-gcc.exe)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_BIN}/arm-none-eabi-g++.exe)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_BIN}/arm-none-eabi-gcc.exe)
```
- Specifies the actual compiler executables:
  - `CMAKE_C_COMPILER` → C compiler (`arm-none-eabi-gcc`)
  - `CMAKE_CXX_COMPILER` → C++ compiler (`arm-none-eabi-g++`)
  - `CMAKE_ASM_COMPILER` → Assembler (`arm-none-eabi-gcc` is reused for `.s` files)
- These are cross-compilers, targeting ARM Cortex-M, not the host PC.

### 5. CPU / Architecture flags
```cmake
# CPU / Architecture flags (example for Cortex-M4)
set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=soft")
```
- Defines CPU-specific flags:
- `-mcpu=cortex-m4` → target the **Cortex-M4 core**
- `-mthumb` → generate **Thumb instruction set** (smaller 16-bit instructions).
- `-mfloat-abi=soft` → use **software floating-point**, not hardware FPU.

    (If your STM32 has an FPU, you might use -mfloat-abi=hard -mfpu=fpv4-sp-d16.)

### 6. Compiler flags for C
```cmake
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CPU_FLAGS} -O2 -Wall")
```
- Adds default flags for **C compilation**:
  - `${CPU_FLAGS}` → CPU architecture options defined earlier.
  - `-O2` → optimize for speed without going too aggressive.
  - `-Wall` → enable most common compiler warnings.

### 7. Compiler flags for C++
```cmake
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CPU_FLAGS} -O2 -Wall -fno-exceptions -fno-rtti")
```
- Adds default flags for **C++ compilation**:
  - Same as C (`CPU_FLAGS`, `-O2`, `-Wall`).
  - `-fno-exceptions` → disables C++ exception handling (smaller code, no runtime overhead).
  - `-fno-rtti` → disables Run-Time Type Information (saves memory and code size).

These are very common in embedded C++ because exceptions and RTTI are rarely needed and increase flash/RAM usage.

## Cmake execution
```bash
bash

cmake -S . -B build   -G "Unix Makefiles"   -DCMAKE_TOOLCHAIN_FILE=arm-gcc-toolchain.cmake
```

1. `cmake` -> → This calls the cmake.exe program from your installed CMake. (locally can be call as `C:/ST/CMake/bin/cmake`)
2. `-S .` → -S specifies the source directory. Here `"."` means current directory.→ So CMake will look in your project folder for a `CMakeLists.txt`.
3. `-B build`
   - ` -B` specifies the **binary (build) directory**.
   - It tells CMake to put all generated files (Makefiles, object files, executables) inside a folder called `build`. This way your source directory stays clean.
   - **Result:**
     - Source code stays in `.` (current folder).
     - Build artifacts go into `./build/`.
  4. `-G "Unix Makefiles"` → This sets the CMake generator.
      - `"Unix Makefiles"` tells CMake to generate Makefiles that can be built with `make`.
      - On Windows, this works if you have **MSYS2/MinGW** make or CMake’s bundled `make`.
      - Alternatives could be `"Ninja"` or `"MinGW Makefiles"`, but here you force `"Unix Makefiles"`.
  5. `-DCMAKE_TOOLCHAIN_FILE=arm-gcc-toolchain.cmake` → -D defines a CMake variable.
      - `CMAKE_TOOLCHAIN_FILE` tells CMake to use a toolchain file that describes your cross-compiler.
      - Here it’s `arm-gcc-toolchain.cmake`.
      - That file usually sets:
        ```cmake
        set(CMAKE_SYSTEM_NAME Generic)
        set(CMAKE_SYSTEM_PROCESSOR arm)
        set(CMAKE_C_COMPILER arm-none-eabi-gcc)
        set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
        set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
        ```

and possibly paths to linker scripts, flags, etc.


## Linker script
Linker scripts are the “map” that tells the linker where in memory each section of your program should go. For embedded systems like STM32, this is critical because you don’t have an OS managing memory for you — you must place code/data explicitly in Flash and RAM.

```ld
/* Minimal STM32G431 linker script */

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

/* Entry point */
ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Vector table */
SECTIONS
{
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    .text :
    {
        *(.text*)
        *(.rodata*)
    } > FLASH

    .data : AT(ADDR(.text) + SIZEOF(.text))
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM

    .bss :
    {
        _sbss = .;
        *(.bss*)
         ebss = .;
    } > RAM

    _sidata = LOADADDR(.data);
}
```

### 1. Define memory regions
```
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}
```
- **FLASH**: starts at `0x08000000` (STM32F4 internal Flash base address), 128 KB size.→ marked `(rx)` meaning **read + execute** (code + constants go here).
- **RAM**: starts at `0x20000000` (STM32F4 SRAM base address), 32 KB size.→ marked `(rwx)` meaning **read/write/execute** (stack, globals, .data, .bss).

### 2. Define entry point
```
ENTRY(Reset_Handler)
```
- Tells the linker that the program starts at the symbol `Reset_Handler`.
- This is defined in **startup_stm32g421xx.s** (the startup assembly file).
- The vector table points to it, and after reset, the MCU executes it.

### 3. Define stack top
```
_estack = ORIGIN(RAM) + LENGTH(RAM);
```
- `_estack` is a symbol = RAM start + RAM length = **end of RAM**.
- This is used as the **initial stack pointer**.
- The first word of the **vector table** is loaded into the CPU’s SP register on reset.

### 4. Sections mapping
#### ISR vector table
```
.isr_vector :
{
    KEEP(*(.isr_vector))
} > FLASH
```
- Places `.isr_vector section` (defined in startup code) at the start of Flash.
- `KEEP()` prevents the linker from discarding it.
- Contains initial stack pointer + addresses of exception/IRQ handlers.

#### Code + constants
```
.text :
{
    *(.text*)
    *(.rodata*)
} > FLASH
```
- `.text` = functions/code.
- `.rodata` = read-only data (like `const char *msg = "Hello";`).
- Both go into Flash, since they are immutable.

#### Initialized variables
```
.data : AT(ADDR(.text) + SIZEOF(.text))
{
    _sdata = .;
    *(.data*)
    _edata = .;
} > RAM
```
- `.data` = initialized global/static variables (e.g. `int counter = 42;`).
- These **live in RAM at runtime**, but initial values come from Flash.
- `AT(...)` tells the linker: the **load address** is right after `.text` in Flash.
- Symbols:
  - `_sdata` → start of `.data` in RAM
  - `_edata` → end of `.data` in RAM
- On reset, startup code copies data from Flash → RAM using these symbols.

#### Uninitialized variables
```
.bss :
{
    _sbss = .;
    *(.bss*)
    ebss = .;
} > RAM
```
- `.bss` = uninitialized globals/statics (e.g. `int counter`;).
- Placed in RAM but **not stored in Flash**.
- Must be **zero-initialized** at runtime.
- Symbols:
  - `_sbss` → start of `.bss`
  - `ebss` → end of `.bss`
- Startup code loops from `_sbss` to `ebss` and sets memory to `0`.

#### Data load address
```
_sidata = LOADADDR(.data);
```
- `_sidata` = Flash address where initial `.data` values are stored.
- Used by startup code to copy data from Flash → RAM.


### 5. Symbol origins (who uses them)
- `_estack` → used in vector table as initial SP.
- `_sidata` → start of `.data` init values in Flash.
- `_sdata`, `_edata` → range in RAM for `.data` (copy here).
- `_sbss`, `ebss` → range in RAM for `.bss` (zero this).
- `Reset_Handler` → startup code entry point, defined in `startup_stm32f407xx.s`

### Typical startup flow:
- CPU resets → loads SP = `_estack` and PC = `Reset_Handler`.
- `Reset_Handler` (from startup file) runs:
  - Copies `.data` from `_sidata` → `_sdata`...`_edata`.
  - Zeroes `.bss` (`_sbss`...`ebss`).
  - Calls `SystemInit()`.
  - Calls `main()`.

## OpenOCD
For flashing and debugging openOCD requires these configuration files

### stlink.cfg
```
#stlink.cfg
adapter driver st-link
transport select swd
```

### stm32g4x.cfg
```cfg
# stm32g4x.cfg
set _CHIPNAME stm32g4
source [find target/stm32f4x.cfg]   ;# STM32G4 is similar to F4 in memory layout for OpenOCD
```

### firmware_flash.tcl
```tcl
#firmware_flash.tcl
# Initialize the interface and target
init
reset halt

# Flash the firmware (adjust filename and address)
flash write_image erase "firmware.bin" 0x08000000

# Reset and run the MCU
reset run

# Close OpenOCD
exit
```

### Flash loading
```bash
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -f flash_firmware.tcl
```

