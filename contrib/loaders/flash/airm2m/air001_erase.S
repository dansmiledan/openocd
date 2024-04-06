/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2024 by LuYao                                           *
 *   themaluyao@outlook.com                                          *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m0plus
	.thumb

	/* Params:
	 * r0 - flash register base (in), status (out)
	 * r1 - IWDG register base(in)
	 * r2 - first of erasing sector(in)
	 * r3 - last  of erasing sector(in)
     * Clobbered:
	 * r4 - tmp
	 * r5 - tmp
     * r6 - tmp
     * r7 - unused
	 */

#define AIR001_FLASH_KEYR_OFFSET    0x08    /* offset of KEYR register to unlock CR        */
#define AIR001_FLASH_SR_OFFSET      0x10    /* offset of SR register from flash reg base   */
#define AIR001_FLASH_CR_OFFSET      0x14    /* offset of CR register from flash reg base   */

#define AIR001_IWDG_KR_OFFSET       0x00    /* offset of KR register from IWDG reg base    */

	.thumb_func
	.global _start

_start:
    movs    r4, #1          /* clear FLASH_CR_SER(bit11)            */
    lsls    r4, #11
    ldr 	r5, [r0, #AIR001_FLASH_CR_OFFSET]
    orrs    r5, r4
    str     r5, [r0, #AIR001_FLASH_CR_OFFSET]
	movs    r6, #0


busy:
	movs	r4, #1          /* wait FLASH_SR_BSY(bit16) reset      */
	lsls    r4, #16
	ldr 	r5, [r0, #AIR001_FLASH_SR_OFFSET]
	tst 	r5, r4
	bne 	reload_iwdg     /* jump when (r5 & (1 << 16)) != 0  is busy    */


erase_sector:
    cmp     r2, r3          /* compare current addr and last addr   */
    bgt     exit            /* exit if r2 > r3                      */

    movs    r4, #0xFF       /* write any byte to current sector     */
    str     r4, [r2]

    dsb

    movs    r4, #1
    lsls    r4, #12
    adds    r2, r4          /* sector_address += sector_size        */

    b       busy


reload_iwdg:
    adds    r6, #1
    movs    r4, #0xAA
    lsls    r4, #8
    movs    r5, #0xAA
    orrs    r4, r5
    str     r4, [r1, #AIR001_IWDG_KR_OFFSET]

    b       busy


error:                      /* return FLASH_SR value                */
	ldr 	r0, [r0, #AIR001_FLASH_SR_OFFSET]

exit:
	ldr 	r0, [r0, #AIR001_FLASH_SR_OFFSET]
    movs    r4, #1          /* clear FLASH_CR_SER(bit11)            */
    lsls    r4, #11
    ldr 	r5, [r0, #AIR001_FLASH_CR_OFFSET]
    bics    r5, r4
    str     r5, [r0, #AIR001_FLASH_CR_OFFSET]

	bkpt	#0

