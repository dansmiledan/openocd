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
	 * r1 - count (word-32bit)
	 * r2 - workarea start
	 * r3 - workarea end
	 * r4 - target address
	 * Clobbered:
	 * r5 - rp
	 * r6 - wp, tmp
	 * r7 - written word count, tmp
	 */

#define AIR001_FLASH_SR_OFFSET 0x10 /* offset of SR register from flash reg base */
#define AIR001_FLASH_CR_OFFSET 0x14 /* offset of CR register from flash reg base */

	.thumb_func
	.global _start
_start:
    movs    r6, #1          /* set FLASH_CR_PG(bit0)            */
    ldr 	r7, [r0, #AIR001_FLASH_CR_OFFSET]
    orrs    r7, r6
    str     r7, [r0, #AIR001_FLASH_CR_OFFSET]

    movs    r7, #0          /* init written count               */


wait_fifo:
    ldr     r6, [r2, #0]    /* read wp(write pointer)           */
	cmp 	r6, #0			/* abort if wp == 0                 */
	beq     exit

	ldr 	r5, [r2, #4]	/* read rp(read pointer)            */
	cmp 	r5, r6			/* wait until rp != wp              */
	beq 	wait_fifo


    cmp     r7, #31         /* prepare to write the 32nd word   */
    beq     write_32nd_word

	ldr	    r6, [r5]	    /* *target_address   = *rp          */
	str 	r6, [r4]
	adds	r5, #4          /* rp               += 4            */
	adds	r4, #4          /* target_address   += 4            */
	adds    r7, #1          /* written_word     += 1            */

word_32nd_done:
	cmp 	r5, r3			/* wrap rp at end of buffer         */
	bcc	    no_wrap         /* jump when r5 < r3                */
	mov	    r5, r2          /* r5>=r3, rp is workarea end       */
	adds	r5, #8          /* rp = workarea_start + 8          */

no_wrap:
	str 	r5, [r2, #4]	/* store rp to [start + 4]          */
	subs	r1, r1, #1		/* word_count -= 1                  */
	cmp     r1, #0
	beq     exit		    /* loop if not done                 */
	b	    wait_fifo


write_32nd_word:
    movs    r7, #1          /* set FLASH_CR_PGSTRT(bit19)       */
    lsls    r7, #19
    ldr 	r6, [r0, #AIR001_FLASH_CR_OFFSET]
    orrs    r6, r7
    str 	r6, [r0, #AIR001_FLASH_CR_OFFSET]

	ldr	    r6, [r5]        /* *target_address   = *rp          */
	str 	r6, [r4]
	adds	r5, #4          /* rp               += 4            */
	adds	r4, #4          /* target_address   += 4            */


busy:
	movs	r7, #1          /* wait FLASH_SR_BSY(bit16) reset   */
	lsls    r7, #16
	ldr 	r6, [r0, #AIR001_FLASH_SR_OFFSET]
	tst 	r6, r7
	bne 	busy            /* jump when (r6 & (1 << 16)) != 0  */

	movs    r7, #0x10       /* check FLASH_SR_WRPERR(bit4)      */
	ldr 	r6, [r0, #AIR001_FLASH_SR_OFFSET]
    tst 	r6, r7
    bne 	error           /* jump when (r6 & (1 << 4)) != 0   */

    movs    r7, #0          /* clear written word               */
	b       word_32nd_done


error:
	movs	r0, #0
	str 	r0, [r2, #4]	/* set rp = 0 on error              */

exit:
	mov		r0, r6			/* return status in r0              */

    movs    r6, #1          /* clear FLASH_CR_PG(bit0)          */
    ldr 	r7, [r0, #AIR001_FLASH_CR_OFFSET]
    bics    r7, r6
    str     r7, [r0, #AIR001_FLASH_CR_OFFSET]

	bkpt	#0
