/* Memory layout for the ESP32 demo firmware, for Espressif's QEMU `-M esp32` and
 * for real silicon (the addresses are the chip's, not the emulator's).
 *
 * Unlike every other port in this kernel, the ESP32's vector table is *linked at a
 * fixed layout*, because the hardware dispatches to `VECBASE + VECOFS` rather than
 * through a vector register. `VECBASE` resets to 0x4000_0000 (mask ROM, 512 KB away
 * from our code), so the firmware *moves* it to the bottom of IRAM and this script
 * puts the table there:
 *
 *   0x4008_0000  window overflow 4      +0x300  kernel exception vector
 *   0x4008_0040  window underflow 4     +0x340  user exception vector (unused)
 *   0x4008_0080  window overflow 8      +0x3C0  double exception vector
 *   0x4008_00C0  window underflow 8     +0x400  code, rodata, data and bss
 *   0x4008_0100  window overflow 12
 *   0x4008_0140  window underflow 12    (0x180..0x2C0: levels 2..7, unused)
 *
 * With the table next to the code, the kernel vector's `j` reaches its handler
 * directly — no literal, no indirect jump, no scratch register (there is none to
 * spare at that moment).
 *
 *   DRAM (0x3FFB_0000) holds only the boot stack; task stacks come from the kernel
 *   arena, which lives in IRAM with everything else.
 *
 * Each vector slot is 0x40 bytes; the ASSERTs make an oversized handler a link error
 * rather than a silently shifted vector table.
 */

ENTRY(_start)

MEMORY
{
  IRAM (rwx) : ORIGIN = 0x40080000, LENGTH = 128K
  DRAM (rw)  : ORIGIN = 0x3FFB0000, LENGTH = 128K
}

/* Boot stack: grows down from the top of DRAM. `_start` loads it. */
_stack_start = ORIGIN(DRAM) + LENGTH(DRAM);

SECTIONS
{
  .vectors :
  {
    . = ORIGIN(IRAM);

    /* Each vector gets an explicit address. That is not just documentation: if a
     * handler grows past its 0x40-byte slot, the *next* `. = ...` would have to move
     * the location counter backwards, which GNU ld refuses - so an oversized handler
     * is a link error, which is exactly the guarantee we want (and the reason this
     * script uses explicit addresses rather than an ASSERT: Xtensa's ld rejects
     * ASSERTs inside an output section). */
    KEEP(*(.WindowOverflow4.text))
    . = ORIGIN(IRAM) + 0x040;
    KEEP(*(.WindowUnderflow4.text))
    . = ORIGIN(IRAM) + 0x080;
    KEEP(*(.WindowOverflow8.text))
    . = ORIGIN(IRAM) + 0x0C0;
    KEEP(*(.WindowUnderflow8.text))
    . = ORIGIN(IRAM) + 0x100;
    KEEP(*(.WindowOverflow12.text))
    . = ORIGIN(IRAM) + 0x140;
    KEEP(*(.WindowUnderflow12.text))

    /* Level-2..7 vectors are unused by the kernel, but they are *filled* rather than
     * left empty: a trap that lands in unmapped space executes whatever bytes follow
     * the table, which is how a diagnosable bug turns into a silent hang. Each slot
     * now reports its identity and the machine state (see `rrkernel_slot_trap`). */
    . = ORIGIN(IRAM) + 0x180;
    KEEP(*(.Level2Vector.text))
    . = ORIGIN(IRAM) + 0x1C0;
    KEEP(*(.Level3Vector.text))
    . = ORIGIN(IRAM) + 0x200;
    KEEP(*(.Level4Vector.text))
    . = ORIGIN(IRAM) + 0x240;
    KEEP(*(.Level5Vector.text))
    . = ORIGIN(IRAM) + 0x280;
    KEEP(*(.Level6Vector.text))
    . = ORIGIN(IRAM) + 0x2C0;
    KEEP(*(.Level7Vector.text))

    . = ORIGIN(IRAM) + 0x300;
    KEEP(*(.KernelExceptionVector.text))

    /* 0x340 is the user exception vector: unused (this kernel stays in kernel mode),
     * filled for the same reason as the level vectors above. */
    . = ORIGIN(IRAM) + 0x340;
    KEEP(*(.UserExceptionVector.text))

    . = ORIGIN(IRAM) + 0x3C0;
    KEEP(*(.DoubleExceptionVector.text))

    . = ORIGIN(IRAM) + 0x400;
  } > IRAM

  /* Code, read-only data and mutable data, all in IRAM (it is writable on the chip,
   * and this bare-metal image has no flash-resident copy to load from).
   *
   * Literals come *first*, and that ordering is load-bearing, not tidiness: `l32r`
   * (the instruction behind every 32-bit constant, i.e. every `movi reg, symbol`)
   * reaches backwards only, so a literal placed after its use is a "dangerous
   * relocation" link error. Collecting them all at the front is safe here because
   * the whole image lives in one 128 KB IRAM region - well inside l32r's 256 KB
   * window. A flash-resident layout would have to interleave `.literal.<fn>` with
   * `.text.<fn>` instead. */
  .text :
  {
    *(.literal .literal.*)
    KEEP(*(.text._start))
    *(.text .text.*)
  } > IRAM

  .rodata :
  {
    *(.rodata .rodata.*)
    *(.srodata .srodata.*)
  } > IRAM

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data .data.*)
    *(.sdata .sdata.*)
    . = ALIGN(4);
    _edata = .;
  } > IRAM

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss .bss.*)
    *(.sbss .sbss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > IRAM

  /* The kernel state, the arena and every task stack live in .bss, so this must hold
   * or the firmware would silently overflow IRAM. */
  ASSERT(_ebss <= ORIGIN(IRAM) + LENGTH(IRAM), "rrkernel: IRAM overflows")

  /DISCARD/ :
  {
    *(.eh_frame*)
  }
}
