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

_start:
    /* Recursively call the ROM API ROM_QSPI_EraseSector() @ 0x801C
     * Params:
     *  r0 - param of QSPI_CommandTypeDef
     *  r1 - param of erase address
     * Clobbered:
     *  r11 - the first sector address that waiting for erasing
     *  r12 - the last sector address that waiting for erasing
     */
// erase_sector:
//     nop
//     cmp     r11, r12
//     bgt     exit            /* r11 > r12, all sectors erased, exit. */
//
//     mov     r0, #0x8000     /* call ROM_QSPI_EraseSector()          */
//     ldr     r2, [r0, #0x1C] /* load function pointer address to r2  */
//     movs    r0, #0x00       /* the first param can be NULL          */
//     mov     r1, r11         /* the second param must be a address   */
//     blx     r2              /* run                                  */
//
//     mov     r0, #0x1000     /* add sector address                   */
//     adds    r11, r0
//     b       erase_sector
//
// exit:
//     bkpt    #0


/* Call ROM API ROM_QSPI_Init() @ 0x8010 */
    mov     r0, #0x8000
    ldr     r1, [r0, #0x10]
    movs    r0, #0x00
    blx     r1
    bkpt    #0

/* Call ROM API ROM_QSPI_ReadID() @ 0x8014 */
//    mov     r0, #0x8000
//    ldr     r1, [r0, #0x14]
//    movs    r0, #0x00
//    blx     r1
//    bkpt    #0

/* Call ROM API ROM_QSPI_WriteParam() @ 0x8018 */
//  mov     r0, #0x8000
//  ldr     r2, [r0, #0x18]
//  movs    r0, #0x00
//  blx     r2          /* warning: the r1 must not be NULL. */
//  mov     r0, r2
//  bkpt	#0


/* Call ROM API ROM_QSPI_EraseSector() @ 0x801C */
//    mov     r0, #0x8000
//    ldr     r2, [r0, #0x1C]
//    movs    r0, #0x00
//    blx     r2
//    mov     r0, r2
//    bkpt    #0


/* Call ROM API ROM_QSPI_EraseChip() @ 0x8020 */
//    mov     r0, #0x8000
//    ldr     r1, [r0, #0x20]
//    movs    r0, #0x00
//    blx     r1
//    mov     r0, r1
//    bkpt    #0


/* Call ROM API ROM_QSPI_ProgramPage() @ 0x8024 */
//    mov     r0, #0x8000
//    ldr     r5, [r0, #0x24]
//    movs    r1, #0x00
//    mov     r0, r1
//    str     r4, [sp, #0x00]
//    blx     r5
//    mov     r0, r5
//    bkpt    #0


/* Call ROM API ROM_QSPI_ReleaseDeepPowerDown() @ 0x802C */
//    mov     r0, #0x8000
//    ldr     r1, [r0, #0x2C]
//    movs    r0, #0x00
//    blx     r1
//    mov     r0, r1
//    bkpt    #0



