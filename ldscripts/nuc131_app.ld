/* Linker script for NUC131 canbox-core Application (0x2000 - 0xFFFF, 56 KB) */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00002000, LENGTH = 56K  /* 0x2000 to 0xFFFF */
    RAM   (rwx) : ORIGIN = 0x20000100, LENGTH = 7936 /* Reserve 0x20000000 - 0x200000FF for token */
}

/* Library configurations */
GROUP(libgcc.a libc.a libm.a libnosys.a)

ENTRY(Reset_Handler)

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.vectors))
        __Vectors = .;
        __Vectors_End = .;
        __Vectors_Size = __Vectors_End - __Vectors;
        __end__ = .;

        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    __etext = .;

    .data : AT (__etext)
    {
        . = ALIGN(4);
        __data_start__ = .;
        _sdata = .;
        *(vtable)
        *(.data*)

        . = ALIGN(4);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))
        . = ALIGN(4);
        __data_end__ = .;
        _edata = .;
    } > RAM
    _sidata = LOADADDR(.data);

    .bss :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
        _ebss = .;
    } > RAM

    .heap (COPY):
    {
        __HeapBase = .;
        __end__ = .;
        end = __end__;
        KEEP(*(.heap*))
        __HeapLimit = .;
    } > RAM

    .stack_dummy (COPY):
    {
        KEEP(*(.stack*))
    } > RAM

    __StackTop = ORIGIN(RAM) + LENGTH(RAM);
    __StackLimit = __StackTop - SIZEOF(.stack_dummy);
    PROVIDE(__stack = __StackTop);
    _estack = __StackTop;

    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
}

