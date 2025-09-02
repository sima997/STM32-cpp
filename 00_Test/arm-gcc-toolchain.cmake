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

# Linker
#set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/linker/stm32f407xx.ld")
#set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -nostdlib -Wl,--gc-sections")
