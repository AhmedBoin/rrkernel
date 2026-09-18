/* The only board-specific file in this project: where flash and RAM are.
 *
 * The chip identifies itself as DEV_ID 0x433 in DBGMCU_IDCODE (0xE004_2000), which is the
 * STM32F401xD/E family: 512 KiB flash, 96 KiB SRAM.
 *
 *     0x0800_0000   512 KiB flash (raise nothing; lower it to 256K for an F401CC)
 *     0x2000_0000    96 KiB SRAM — declared as 64K on purpose:
 *
 * The smaller, common "Black Pill" (F401CC, DEV_ID 0x423) has 64 KiB, and this image is
 * nowhere near either limit, so declaring the conservative 64K keeps the same binary valid
 * on both parts. Raise RAM to 96K only if you need it.
 */

MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   : ORIGIN = 0x20000000, LENGTH = 64K
}
