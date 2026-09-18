/* Memory layout for the rrkernel bare-metal demo.
 *
 * Defaults target the LM3S6965EVB that QEMU emulates
 * (`qemu-system-arm -M lm3s6965evb -kernel ...`), because flash aliases at
 * address 0 there — which is also true of most STM32 parts, so the same image
 * links happily for real hardware. Change the two origins for anything else.
 */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 256K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Initial stack pointer: top of RAM, growing down. Task 0 (`main`) and every
 * ISR use this region; spawned tasks get their own stacks from the kernel
 * arena. */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

ENTRY(__reset)

SECTIONS
{
  /* The vector table must be the first thing in flash: the core reads the
   * initial SP from address 0 on reset. */
  .vector_table ORIGIN(FLASH) :
  {
    KEEP(*(.vector_table))
  } > FLASH

  .text :
  {
    *(.text .text.*)
  } > FLASH

  .rodata :
  {
    *(.rodata .rodata.*)
  } > FLASH

  /* Initialised statics live in RAM but are stored in flash; __reset copies
   * them across. */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data .data.*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT> FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss .bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /* Kernel state and the arena land here; `_ebss` must stay below
   * `_stack_start` or the stack silently overwrites the kernel. */
  ASSERT(_ebss <= _stack_start, "rrkernel: RAM overflows into the stack region")

  /DISCARD/ :
  {
    *(.ARM.exidx*)
    *(.ARM.extab*)
  }
}
