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
     * r1 - IWDG base
	 * r2 - target address(in)
	 * r3 - source address(in)
	 * r4 - word count(in)
     * Clobbered:
	 * r5 - written word count
	 * r6 - tmp
     * r7 - tmp
	 */

#define AIR001_FLASH_SR_OFFSET      0x10    /* offset of SR register from flash reg base   */
#define AIR001_FLASH_CR_OFFSET      0x14    /* offset of CR register from flash reg base   */

#define AIR001_IWDG_KR_OFFSET       0x00    /* offset of KR register from IWDG reg base    */

	.thumb_func
	.global _start

_start:
    movs    r5, #1          /* init written count                   */

    movs    r6, #1          /* set FLASH_CR_PG(bit0)                */
    ldr 	r7, [r0, #AIR001_FLASH_CR_OFFSET]
    orrs    r7, r6
    str     r7, [r0, #AIR001_FLASH_CR_OFFSET]


busy:
	movs	r6, #1          /* wait FLASH_SR_BSY(bit16) reset      */
	lsls    r6, #16
	ldr 	r7, [r0, #AIR001_FLASH_SR_OFFSET]
	tst 	r7, r6
	bne 	reload_iwdg     /* jump when (r7 & (1 << 16)) != 0      */


write_page:                 /* write flash by page(32 word)         */
    cmp     r5, r4
    bgt     exit            /* exit if has write done               */

    movs    r6, #0x1F
    tst     r5, r6          /* set PGSTRT before writting 32nd word */
    beq     write_last_word /* jump when (r5 & 31) == 0, r5 = n*32  */

	ldr	    r6, [r3]	    /* *target_address++ = *source_address++*/
	str 	r6, [r2]
	adds	r2, #4          /* target_address +=4                   */
	adds	r3, #4          /* source_address +=4                   */

    adds    r5, #1          /* the written word count plus 1        */
	b       write_page


write_last_word:            /* write n*32 word                      */
    movs    r6, #1          /* set FLASH_CR_PGSTRT(bit19)           */
    lsls    r6, #19
    ldr 	r7, [r0, #AIR001_FLASH_CR_OFFSET]
    orrs    r7, r6
    str 	r7, [r0, #AIR001_FLASH_CR_OFFSET]

	ldr	    r6, [r3]	    /* *target_address = *source_address    */
	str 	r6, [r2]
	adds	r2, #4          /* target_address += 4                  */
    adds	r3, #4          /* source_address += 4                  */
    adds    r5, #1          /* written_word++                       */

    dsb
    b       busy


reload_iwdg:
    movs    r6, #0xAA
    lsls    r6, #8
    movs    r7, #0xAA
    orrs    r6, r7
    str     r6, [r1, #AIR001_IWDG_KR_OFFSET]

    b       busy


error:                      /* return FLASH_SR value                */
	ldr 	r0, [r0, #AIR001_FLASH_SR_OFFSET]


exit:
    movs    r6, #1          /* clear FLASH_CR_PG(bit0)              */
    ldr 	r7, [r0, #AIR001_FLASH_CR_OFFSET]
    bics    r7, r6
    str     r7, [r0, #AIR001_FLASH_CR_OFFSET]

	bkpt	#0

