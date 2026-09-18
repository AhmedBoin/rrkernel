/* Memory layout for the RISC-V demo firmware.
 *
 * Defaults match QEMU's `virt` machine, whose RAM starts at 0x8000_0000 (128 MiB
 * by default). Only the first 16 MiB are described here: enough for the kernel,
 * the arena and the stack, and small enough that the link-time assertion below is
 * meaningful.
 *
 * For `sifive_e` (also 0x8000_0000, but only 16 KiB of RAM) or a real board,
 * change ORIGIN/LENGTH here — nothing else in the kernel needs to know.
 */
MEMORY
{
  RAM (rwx) : ORIGIN = 0x80000000, LENGTH = 16M
}

/* Boot stack: the top of the described RAM, growing down. _start loads it. */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

ENTRY(_start)

SECTIONS
{
  /* The entry point must be first: QEMU jumps to the ELF entry point, and a
   * firmware without a boot ROM needs nothing else in front of it. */
  .text ORIGIN(RAM) :
  {
    KEEP(*(.text._start))
    *(.text .text.*)
  } > RAM

  .rodata :
  {
    *(.rodata .rodata.*)
    *(.srodata .srodata.*)
  } > RAM

  /* Initialised data: stored in RAM, but copied by _start from its load address
   * (the same address here, so the copy is a no-op — it matters on real boards
   * with flash, which is why the code is there). */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(8);
    _sdata = .;
    *(.data .data.*)
    *(.sdata .sdata.*)
    . = ALIGN(8);
    _edata = .;
  } > RAM

  .bss (NOLOAD) :
  {
    . = ALIGN(8);
    _sbss = .;
    *(.bss .bss.*)
    *(.sbss .sbss.*)
    *(COMMON)
    . = ALIGN(8);
    _ebss = .;
  } > RAM

  /* The kernel state, the arena and every task stack live in .bss, so this must
   * hold or the boot stack would silently overwrite the kernel. */
  ASSERT(_ebss <= _stack_start, "rrkernel: RAM overflows into the stack region")

  /DISCARD/ :
  {
    *(.eh_frame*)
    *(.riscv.attributes)
  }
}
