/* STM32F103C8: 12 KiB protected bootloader, 20 KiB RAM. */
ENTRY(Reset_Handler)
MEMORY {
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 12K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
    .isr_vector : { KEEP(*(.isr_vector)) } >FLASH
    .text : { *(.text*) *(.rodata*) . = ALIGN(4); } >FLASH
    .data : {
        . = ALIGN(4); _sdata = .;
        *(.data*) . = ALIGN(4); _edata = .;
    } >RAM AT>FLASH
    _sidata = LOADADDR(.data);
    .bss (NOLOAD) : {
        . = ALIGN(4); _sbss = .;
        *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .;
    } >RAM
    /DISCARD/ : { *(.note*) *(.comment*) *(.eh_frame*) }
}
ASSERT(SIZEOF(.isr_vector) >= 0xC0, "Incomplete vector table")
