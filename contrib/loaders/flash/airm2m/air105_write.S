/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2024 by LuYao                                           *
 *   themaluyao@outlook.com                                                *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m4
	.thumb

	.thumb_func
	.global _start

	/* Params:
	 * r0 - param of QSPI_CommandTypeDef
	 * r1 - param of DMA_TypeDef
	 * r2 - param of target address
	 * r3 - param of count(default 256 bytes)
	 * sp - param of source address
	 * Clobbered(Warning: The ROM API may use low-address registers.):
	 * r7  - rp
	 * r8  - wp
	 * r9  - workarea start
	 * r10 - workarea end
	 * r11 - target address
	 * r12 - count(pages)
	 */

#define AIR105_FLASH_PAGE_SIZE 256  /* flash page size */

_start:
wait_fifo:
    ldr     r8, [r9, #0]    /* read wp from workarea start offset 0 byte    */
	cmp     r8, #0          /* abort if wp == 0                             */
	beq     exit
	ldr 	r7, [r9, #4]	/* read rp from workarea start offset 4 byte    */
	cmp 	r8, r7			/* wait until rp != wp                          */
	beq 	wait_fifo

    b       write_page      /* call ROM API to write a page(256 bytes)      */

write_success:
    ldr 	r7, [r9, #4]    /* read rp from workarea start offset 4 byte    */
    adds	r7, #0x100      /* rp += 256                                    */
	adds	r11, #0x100     /* target address += 256                        */

	cmp 	r7, r10			/* wrap rp at end of buffer                     */
	bcc	    no_wrap         /* jump when r7 < r10                           */
	mov	    r7, r9          /* when r7 >= r10, also rp is workarea end      */
	adds	r7, #8          /* set r8 work start offset 8 byte, fifo start  */

no_wrap:
	str 	r7, [r9, #4]	/* store rp                                     */
	subs	r12, r12, #1	/* decrement page count                         */
	cmp     r12, #0         /* r12==0, all pages write done                 */
	beq     exit		    /* loop if not done                             */
	b	    wait_fifo

write_page:
    mov     r0, #0x8000
    ldr     r5, [r0, #0x24] /* load ROM API at 0x8024       */

    movs    r0, #0x00       /* param of QSPI_CommandTypeDef */
 // movs    r1, #0x4000     /* param of DMA_TypeDef         */
 // lsls    r1, #16
 // adds    r1, #0x800      /* DMA always failure           */
    movs    r1, #0x00       /* param of DMA_TypeDef         */
    mov     r2, r11         /* param of target address      */
    movs    r3, #0x100      /* param of count, default 256  */
    str     r7, [sp, #0x00] /* param of source address      */
	blx     r5
	cmp     r0, #0
	bne     error           /* ROM API return value not 0   */
	b       write_success

error:
    movs    r7, #0          /* clear rp, wait exit          */
    str     r7, [r9, #4]
	bkpt    #0              /* call ROM API failed          */

exit:
	bkpt	#0

