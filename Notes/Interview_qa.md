# Interview questions

## C/C++ and Embedded programming

### 1. What is difference between `volatile` and `const volatile`?
- `volatile` tells the compiler: "*This variable can change at any time outside program control (e.g., by hardware, interrupts). Don't optimize it anyway.*"
  ```C
  volatile uint32_t *UART_STATUS = (uint32_t*)0x4000;
  while ((*UART_STATUS & 0x01) == 0); //Wait until ready
  ```
- `const volatile` means: "*It can change unpredictably, but you can't assign to it.*" Useful for **read-only hardware registers** that the CPU must not write to.
  ```C
  const volatile uint32_t *ADC_RESULT = (uint32_t*)0x5000;
  uint32_t value = *ADC_RESULT; //OK
  *ADC_RESULT = 10;             //Compile error
  ```

### 2. How does static `static` keyword behave in embedded code?
- **Inside a function** -> variable persist between calls (keeps value in RAM/flash)
  ```C
  void counter(void) {
    static int count = 0; //Not reinitialized on each call
    count++;
  }
  ```
- **At file scope** -> restrict symbol visibility to that file (internal linkage)
 
  ```C
  static int localVar; //Only visible inside this .c file
  ```

### 3. How do you prevent a function from being inlined?
Compilers may inline small function for optimization. To prevent:
- In GCC/Clang
    ```C
  __attribute__((noinline)) void foo(void) {...}
    ```
- In MSVC
    ```C
    __declspec(noinline) void foo(void) {...}
    ```
Useful when:
- You want a **consistent function call boundry** (e.g., for debugging or profiling)
- Function is **called via a pointer** 

### 4. What are memory sections like `.text`, `.data`, `.bss`?
- **.text** -> Code (read-only, usually in flash/ROM)
- **.data** -> Initialized global/static variables (stored in flash, copied to RAM at startup)
- **.bss** -> Uninitialized global/static (allocated in RAM, zeroed at startup)

Example
```C
int global_uninit;          //in .bss
int global_init = 5;        //in .data
const int constVal = 10;    //often in flash (.rodata)
```

### 5. What are the risk of using dynamic memory (e.g., `malloc`) in embedded systems?
- **Fragmentation** -> small alloc/free cycles leave unusable gaps.
- **Unpredictable latency** -> allocation time not deterministic.
- **Memory exhaustion** -> system crash if heap runs out.
- Many embedded systems **avoid `malloc`** and **static allocation** or **custom memory pools.**

### 6. How do you implement a circular buffer?
Efficient structure for FIFO (e.g., UART RX/TX).
```C
#define SIZE    6
uint8_t buf[SIZE];
int head = 0, tail = 0;

void put(uint8_t val) {
    buf[head] = val;
    head = (head + 1) % SIZE;
}

uint8_t get(void) {
    uint8_t val = buf[tail];
    tail = (tail + 1) % SIZE;
}
```
- Works in constant time O(1).
- Common for **interrupt-driven I/O.**


### 7. What's the difference between a soft reset and hard reset?
 - **Soft Reset** -> Reset by software (watchdong, special register write, jump to reset vector). Hardware remains powered, but system state is re-initialized.
 - **Hard Reset** -> Power cycle or reset pin asserted. Everything reboots from cold state.


## RTOS and Concurency
### 1. How does a context switch works?
- **Definition:** A context switch is when  the CPU stops executing one task and switches to another.
- **What happens:**

  1.  Save the current task's CPU state (registers, program counter, stack pointer) into its task control block (TCB)
  2.  Load the next task's svaed state from its TCB
  3.  Update the scheduler's bookkeeping (e.g., ready queue).
   
- **Triggers** 
  - Timer interrupt (preemption)
  - Task voluntarily yielding (e.g., waiting on semaphore)
  - Higher-priority task becoming ready
- **Example in RTOS**
  - In **FreeRTOS**, the SysTick timer interrupt periodically triggers the scheduler to decide if another task should run.
   
  

### 2. What are priority inversion and how do you solve it?
- **Problem:** A high-priority task is blocked because a low-priority task holds a resource (e.g., mutex), while  a medium-priority task keeps running and preventing the low priority task from releasing it.
- **Classic example:**
  - Low priority holds a mutex
  - High priority wants the mutex -> gets blocked
  - Medium priority keeps running -> prevents low from ever finishing -> high is starved.
- **Solutions:**
  1. **Priority inheritance**: Temporarily raise the low task's priority to that of the highest waiting task
  2. **Priority cancelling:** Assign a resource the highest priority of tasks that might use it.
  3. **Avoid blocking:** when possible (use lock-free queues or disabling interrupts briefly for short critical sections).

### 3. What's the difference between mutexes, semaphores and spinlocks?
- **Mutex (Mutual Exclusion Lock):**
  - Ensure **only one task** can access a resource at a time
  - Usually has ownership (task that locks must unlock)
  - May support **priority inheritance**
  - Used for protecting **shared data**
- **Semaphore:**
  - **Counting semaphore:** Allows limited number of concurent accesses (e.g., a pool of buffers)
  - **Binary semaphore:** Similar to mutex, but no ownership -> any task can release.
  - Often used for **signaling between tasks** or between **ISR and task.**
- **Spinlock:**
  - A lock where the task **busy-waits (spins)** until available.
  - Good for SMP (multi-core) systems where waiting is short.
  - Wasteful in single-core embedded systems (ties up CPU)

### 4. Explain interrupt latency and how to reduce it.
- **Interrupt latency:** The delay between when hardware raises an interrupt and when corresponding ISR starts executing.
- **Causes:**
  - Interrupts disabled (e.g., critical section)
  - Higher priority ISR already running
  - Long context-save/restore operations
- **Reduction strategies:**
  - Keep ISRs **short and fast** (defer work to task)
  - Use **nested interrupts** if supported
  - Minimize time whith **global interrupts disabled**
  - Optimize context switch mechanism

### 5. Describe how you would implement a task scheduler.
- **Data structures:**
  - Task Control Block (TCB) for each task (holds stack pointer, state, priority, etc.).
  - Ready queue (tasks ready to run).
- **Algorithm:**
  - On timer tick or event:
    - Choose highest-priority ready task.
    - Perform context switch if it's not the current task.
- **Types of scheduler:**
  - **Round robin:** Equal time slices among tasks.
  - **Priority-based preemptive:** Always run highest-priority ready task (common in RTOS)
  - **Cooperative:** Tasks yield voluntarily (simpler, but less real-time).
- **Optimization:**
  - Use bitmap priority queues (O(1) scheduling). Example: FreeRTOS `uxTopReadyPriority`

## Hardware & Peripherals
### 1. How does memory-mapped I/O work?
- Many microcontrollers/peripherals expose registers as special memory addresses
- CPU reads/writes to those addresses as if they were normal RAM, but instead it configures hardware or exchanges data.
- Example:
    ```C
    #define GPIO_OUT    (*(volatile uint32_t*)0x40020000)
    GPIO_OUT = 0x01;    //Sets pin 0 high
    ```
- Key idea: **Load/Store instructions to these addresses trigger hardware behaviour**

### 2. How would you set up and handle an interrupt from a GPIO pin?
1. **Configure the pin** as input with interrupt capability (rising/falling edge)
2. **Enable interrupt** in the peripheral and NVIC (ARM Cortex)
3. **Write an ISR** (Interrupt Service Routine):
    ```C
    void GPIO_IRQHandler(void) {
        if (GPIO->IF & (1 << 3)) {  //Check if pin 3 caused interrupt
            GPIO->IFC = (1 << 3);   //Clear interrupt flag
            handle_button_press();
        }
    }
4. **Keep ISR short** - defer long work to a task or queue.

### 3. How do you initialize a UART or SPI peripheral?
- **UART init (example):**
  1. Enable clock for UART peripheral
  2. Configure baud rate, data bits, parity, stop bits
  3. Enable TX/RX
    ```C
    UART->BAUD = calc_baud(115200);
    UART->CTRL = UART_CTRL_TXEN | UART_CTRL_RXEN;
    ```

- **SPI init (example)**
  1. Configure pins: MOSI, MISO, SCK, CS
  2. Set mode (clock polarity & phase), data order
  3. Set clock speed, enable master mode
    ```C
    SPI->CTRL = SPI_CTRL_MSTR | SPI_CTRL_CPOL | SPI_CTRL_CPHA;
    SPI->BAUD = spi_div(1000000); //1Mhz
    ```

### 4. How would you debug a non-responding I2C device?
- **Check hardware basics**
  - Are pull-up resistors on SDA/SCL present
  - Is the bus powered? 
- **Check addressing**
  - Device may use 7-bit vs 8-bit addressing confusion
  - Use an I2C scanner routine to probe all addresses
- **Check signals**
  - Use logic analyzer to cofirm start/stop conditions, ACK bits
- **Fallbacks**
  - Try restarting bus (toggle SCL manually if stuck low)
  - Power-cycle device if it latched

### 5. What is the difference between polling and interrupt-driven I/O?
- **Polling:**
  - CPU repeatedly checks a status flag
  - Simple, predictable, but wastes CPU cycles.
```C
while (!(UART->STATUS & RX_READY)) {}
char c = UART->DATA;
```

- **Interrupt-driven:**
  - CPU does useful work until peripheral signals event via interrupt
  - Efficient, but requires ISR and queue handling.
```C
void UART_IRQHandler(void){
    char c = UART->DATA;
    enqueue( rx_buffer, c);
}
```

## Linux-Based Firmware
### 1. What is device tree in Linux? How it is used in embedded systems?
- The **device tree (DT)** is a data structure describing the hardware to the Linux kernel
- Instead of hardcoding peripherals in kernel, the DT provides a machine-readable description (in .dts source -> compiled to .dtb binary)
- Example snippet:
    ```dts
    uart0: serial@4000 {
        compatible = "ns16550";
        reg = <0x4000 0x1000>;
        interrupts = <5>;
    };
    ```
- Kernel drivers match against `compatible` and know how to initialize hardware
- **Why important:** It allows the same kernel binary to run on multiple boards by swapping only the device tree

### 2. How do you cross-compile a Linux kernel for an ARM-based board?
1. Install a cross-compiler toolchain (e.g., arm-linux-gnueabihf-gcc)
2. Configure kernel to the target:
    ```bash
    make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- deconfig
    ```
3. Optionally tweak config (`menuconfig` or `xconfig`)
4. Build kernel + device tree:
    ```bash
    make ARCH=arm CROSS_COPILE=arm-linux-gnueabihf- zImage dtbs
    ```
5. Build modules:
    ```bash
    make ARCH=arm CROSS_COPILE=arm-linux-gnueabihf- modules
    ```
6. Copy kernel image, DTB, and modules into rootfs or boot partition

**Tip in interview:** Mention the importance of matching **kernel version** with toolchain and bootloader (e.g., U-Boot).


### 3. What is `udev` and how does it relate to device management?
- `udev` is the **user-space device manager** for Linux
- It dynamically creates/removes device nodes in /dev ad hardware appears/disappears (hotplug, USB insertion)
- Uses rules (/etc/udev/rules.d) to assign permisions, symlinks, or run scripts.
- Example: Creating a persistent name for a USB device:
    ```C
    SUBSYSTEM = 'tty', ATTRS{idVendor}="067b", SIMLINK+="usb_serial"
    ```
- In embedded Linux, this allows flexible device management without rebuilding kernel

### 4. How would you bring up a board with a custom Linux bootloader and Linux image?
1. **Bootloader stage (e.g., U-Boot):**
   - Initialize DRAM, clocks, UART
   - Load kernel (zImage/uImage) and DTB into RAM
   - Pass control to linux kernel
2. **Kernel stage:**
   - Kernel uncompresses, mount root file system
   - Initializes drivers based on DTB
3. **Root filesystem:**
   - Minimal rootfs with /sbin/init (could be BusyBox)
   - provide shell or service start scripts
4. **Debugging bring-up:**
   - Use UART console for boot logs
   - Test step by step: first bootloader -> kernel boot -> mount rootf
   - Common first target: "I got shell prompt!"

### 5. What are the roles of `init`, `systemd`, or `BusyBox` in embedded Linux?
  - **init:** The very first user-space process (PID 1). Resposible for starting system services, spawning shells, mounting filesystems
  - **systemd:** Modern, full-featured init system. Handles parallel service startup, dependencies, logging. Often too heavy for very constrained devices.
  - **BusyBox:** A single binary providing many Unix utilities (ls, cp, sh, init). Very common in embedded Linux to save space. Often used with a *simple init system** (scripts in /etc/init.d).

## Low-Level & Reverse Engineering
### 1. How would you reverse-engineer a binary from a firmware blob?
- **Static analysis:**
  - Inspect with `binwalk`, `strings`, `file`, `hexdump` to indentify patterns (magic headers, ASCII text, compression)
  - Disassemble with Ghidra, IDA or radare2 to see machine code
  - Look for recognizable libraries (e.g., newlib, uClibc)
- **Dynamic analysis:** (if hardware available)
  - Load into an emulator (QEMU, Uicorn Engine)
  - Observe syscalls, memory access, logs
- **Approach in interview:** *"I'd first look for a file system or ELF section with binwalk. Then I'd check compression/encryption, and finally try to disassemble to understand functions."*

### 2. What are common file formats for firmware images (e.g., ELF, bin, hex)?
- **ELF** (Executable Linkable Format) -> contains headers, symbol tables, sections (.text, .bss, .data). Used durring development/debugging
- **BIN** -> raw binary dump. No metadata, just bytes as they go into flash
- **HEX** (Intel HEX / Motorola S-record) -> ASCII-encoded representation of memory with checksums. Easier to transfer reliably over serial/bootloaders.
- **DFU** (Device Firmware Update) -> USB-based format for flashing

In interviews, it's good to say: "The development toolchain often produces ELF, but deployed images are usually stripped down to BIN or HEX."

### 3. What tools would you use to analyze memory maps or flash dumps?
- **binwalk** -> extract partitions, compressed blobs
- **readelf / objdump** -> inspect ELF sections, symbols
- **xxd / hexdump** -> raw inspection
- **Ghidra / IDA / radare2** -> reverse disassembly/decompilation
- **Flashrom, OpenOCD** -> dump/flash directly from hardware
- **Logic analyzer + datasheets** -> correlate flash contents with bus activity


### 4. How can you recover a bricked device with no serial output?
- **Check boot order:** Many MCUs/SoCs have boot ROMs that try USB/UART/SPI in order -- force device into recovery mode with strapping pins
- **JTAG/SWD:** Attach debugger to halt CPU, reflash memory
- **Use external programmer:** Desolder or in-circuit program SPI flash
- **Fallback bootloaders:** Some chips have immutable ROM code that can load a minimal image over USB/serial even if main flash is corrupted
- **In interview:** Show that you'd try non-invasive recovery first (reset, force bootloader), then escalate to JTAG/flash programmer
 
### 5. Explain how JTAG or SWD works and how you'd use it.
- **JTAG** (Joint Test Action Group):
  - A debug interface standartized for boundry scan and chip level debugging
  - Provides access to registers, memory, and allows halting CPU
- **SWD** (Serial Wire Debug):
  - ARM-specific, 2-wire alternative to JTAG with similar debug capabilities
  - Widely used in Cortex-M microcontrollers
- **Usage:**
  - Attach debugger (e.g.,ST-Link, J-Link, or OpenOCD-compatible probe)
  - Load firmware, set brakepoints, inspect memory/peripherals
  - Essential for bring-up, debugging hard faults or unbricking

## Tooling and debugging
### 1. What's your workflow for debugging hard faults on ARM Cortex-M?
- **Step 1: Capture fault info**
  - On Cortex-M, hard faults trap into `HardFault_Handler`
  - Extract fault registers (HFSR, CFSR, MMFAR, BFAR) to see cause (e.g., invalid memory access, bus fault)
- **Step 2: Inspect stacked frame**
  - On exception, Cortex-M pushes registers R0-R3, R12, LR, PC, xPSR onto stack
  - Print or log these to identify the exact instruction causing the crash
- **Step 3: Debug with GDB/OpenOCD**
  - Halt MCU, inspect program counter, backtrace
- **Step 4: Fix**
  - Look for null pointer dereferences, stack overflows, invalid peripheral access

### 2. How do you use gdb or OpenOCD for on-target debugging?
- **OpenOCD:** Talks to the debug probe (JTAG/SWD) and provides GDB server
- **GDB:**
  1. Connect to target
    ```bash
    arm-none-eabi-gdb firmware.elf
    (gdb) target remote :3333
    ```
  2. Load symbols and firmware
    ```bash
    (gdb) load
    ```
  3. Debug
    ```bash
    (gdb) break main
    (gdb) continue
    (gdb) info registers
    (gdb) backtrace
    ```
- **Workflow:** Build firmware with debug symbols, connect via probe, step through code, inspect memory/peripherals

### 3. What are the pros/cons of using strace, dmesg, or perf on embedded Linux?
- **strace**
  - *Pro:* Shows system calls and process makes (great for debugging missing files, permissions, unexpected kernel errors)
  - *Con:* Adds overhead, may not work well on time sensitive processes
- **dmesg**
  - *Pro:* Shows kernel log buffer (dricer messages, hardware initialization, kernel panics) 
  - *Con:* Buffer limited, logs may roll over, only kernel space (not user)
- **perf**
  - *Pro:* Performance profiling tool (CPU cycles, cache misses, hotspots)
  - *Con:* Requires kernel support, can be heavy on very small systems

### 4. Describe how to use objdump or readelf for firmware inspection
- **readelf:** Prints ELF headers, section headers, symbol tables
    ```bash
    readelf -h firmware.elf     #ELF header
    readelf -S firmware.elf     #Section headers
    readelf -s firmware.elf     #Symbol table
    ```

- **objdump:** Disassembles instructions, shows relocation info
    ```bash
    arm-none-eabi-objdump -d firmware.elf > disasm.S
    arm-none-eabi-objdump -x firmware.elf #all headers
    ```

- **Use case:** Verify memory layout, check which functions got linked, inspect interrupt vectors

## Bahavioral / Experience Questions
### 1. "Tell me about a time you debugged a difficult hardware issue."
**What they want:** Problem-solving process, persistance, and collaboration. 

**How to answer (STAR method)**
- **Situation:** "We had a prototype board where the SPI peripheral wasn't responding."
- **Task:** "I need to identify wheter it was firmware, wiring or the ASIC itself."
- **Action:** "I scoped the signals, compared with reference timing, isolated the issue to incorrect clock polarity, then patched the firmware."
- **Result:** "We fixed the communication, unblocked development, and docummented the bug for hardware rev 2."

### 2. "How do you approach debugging a black-box system."
**What they want:** Logical thinking, working without full visibility.

**Strong answer outline:**
- **Observe inputs/outputs** - treat system like function
- **Form hypotheses** - it is software, hardware or timing?
- **Instrument** - add logging, use debug pins, bus sniffers
- **Narrow down scope** - binary search through system components
- **Fallback** - datasheets, schematics, reverse engineering if needed

### 3. "Describe your experience with custom PCBs or bring-up of new boards"
**What they want:** Practical familiarity with **first silicon/board bring-up**

**Points to cover**
- Power sequencing checks (multimeter, current probes)
- Verify clock/crystal is running
- Get UART or JTAG working first ("Hello World" LED/UART print)
- Incrementally test peripherals
- Document findings for hardware/PCB designers


### 4. "What's your experience with version control in firmware development?"
**What they want:** Do you use Git professionaly and do you follow good practices?

**Key talking points:**
- Git basics: branching merging, rebasing
- Workflow: feature branches, pull request, code reviews
- Tags/releases for firmware builds
- Submodules (common for SDKs or drivers).
- Handling binary artifacts (don't commit compiled images, use CI/CD pipelines)

