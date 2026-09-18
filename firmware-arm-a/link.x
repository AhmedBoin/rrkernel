/* Memory layout for the ARM A/R-profile demo firmware.
 *
 * QEMU's 32-bit `virt` machine puts RAM at 0x4000_0000; a Cortex-R board will
 * have its own map (TCM/DRAM), so change ORIGIN/LENGTH here and nothing else.
 */
MEMORY
{
  RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 16M
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);

ENTRY(_start)

SECTIONS
{
  /* The exception vectors are installed via VBAR, so they do not have to be at
   * address 0 — but keeping them first is tidy and free. */
  .text ORIGIN(RAM) :
  {
    KEEP(*(.text._start))
    *(.text .text.*)
  } > RAM

  .rodata :
  {
    *(.rodata .rodata.*)
  } > RAM

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(8);
    _sdata = .;
    *(.data .data.*)
    . = ALIGN(8);
    _edata = .;
  } > RAM

  .bss (NOLOAD) :
  {
    . = ALIGN(8);
    _sbss = .;
    *(.bss .bss.*)
    *(COMMON)
    . = ALIGN(8);
    _ebss = .;
  } > RAM

  ASSERT(_ebss <= _stack_start, "rrkernel: RAM overflows into the stack region")

  /DISCARD/ :
  {
    *(.ARM.exidx*)
    *(.ARM.extab*)
    *(.gnu.attributes)
  }
}
