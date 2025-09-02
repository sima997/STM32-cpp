/* Minimal STM32F407 linker script */

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
