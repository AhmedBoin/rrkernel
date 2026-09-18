/* The only board-specific file in this project: where flash and RAM are.
 *
 * `cortex-m-rt`'s linker script `INCLUDE`s this file and derives the vector table's
 * initial stack pointer from the top of `RAM`, so getting these two numbers right is
 * the whole "porting" step for a new Cortex-M part.
 *
 * STM32F103C8 "Blue Pill":
 *   64 KiB flash, 20 KiB RAM. Change LENGTH to 128K if your "C8" is really a CB.
 *
 * Other parts, for reference:
 *   STM32F103CB   64K flash wait: 128K flash, 20K RAM
 *   STM32F401RE   512K flash, 96K RAM
 *   STM32F407VG   1M flash, 128K RAM (+64K CCMRAM, not covered by this region)
 *   nRF52840      1M flash, 256K RAM
 *   RP2040        2M flash, 264K RAM (this region is the SRAM at 0x2000_0000)
 */

MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM   : ORIGIN = 0x20000000, LENGTH = 20K
}
