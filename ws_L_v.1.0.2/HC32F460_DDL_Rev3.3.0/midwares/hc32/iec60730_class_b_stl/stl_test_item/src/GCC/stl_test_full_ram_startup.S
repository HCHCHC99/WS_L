/**
 ******************************************************************************
  @file  stl_test_full_ram_startup.S
 verbatim
  Change Logs:
  Date             Author          Notes
  2024-10-31       CDT             First version
 endverbatim
 *****************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

.syntax     unified
.arch       armv7e-m
.cpu        cortex-m4
.thumb

.align      2
data0x00000000: .word    0x00000000
data0xFFFFFFFF: .word    0xFFFFFFFF

/*
 * function    : uint32_t STL_FullRamTestStartup(void);
 * Description : STL_FullRamTestStartup
 * Input       : None.
 * Output      : Perform routine when detect failure at set of self test cases
 * Return      : STL_OK (=0):test pass; STL_ERR (=1):test fail;
 * WARNING     : all registers destroyed when exiting this function (including
 *               preserved registers R4 to R11) and excluding stack point R13)
 */
.align      2
.global STL_FullRamTestStartup
.type STL_FullRamTestStartup, %function
STL_FullRamTestStartup:
            MOVS  R4, #0x0               /* STL_OK:Test success status by default */

            LDR   R2, =data0x00000000    /* Prepares background pattern */
            LDR   R2, [R2]
            LDR   R3, =data0xFFFFFFFF    /* Prepares inverted background pattern */
            LDR   R3, [R3]

            /**** Step 1 ****/
            /* Write background pattern with addresses increasing */
            MOVS  R5, R0
_step1_loop:
            CMP   R5, R1
            BHI   _step_2
            STR   R2, [R5, #+0]
            ADDS  R5, R5, #+4
            B     _step1_loop

            /**** Step 2 ****/
            /* Verify background and write inverted background with addresses increasing */
_step_2:
            MOVS  R5, R0
_step_2_loop:
            CMP   R5, R1
            BHI   _step_3
            LDR   R6, [R5, #+0]
            CMP   R6, R2
            BNE   _full_ram1_test_fail
            STR   R3, [R5, #+0]
            LDR   R6, [R5, #+4]
            CMP   R6, R2
            BNE   _full_ram1_test_fail
            STR   R3, [R5, #+4]

            LDR   R6, [R5, #+8]
            CMP   R6, R2
            BNE   _full_ram1_test_fail
            STR   R3, [R5, #+8]
            LDR   R6, [R5, #+12]
            CMP   R6, R2
            BNE   _full_ram1_test_fail
            STR   R3, [R5, #+12]

            ADDS  R5, R5, #+16
            B     _step_2_loop

            /**** Step 3 ****/
            /* Verify inverted background and write background with addresses increasing */
_step_3:
            MOVS  R5, R0
_step_3_loop:
            CMP   R5, R1
            BHI   _step_4
            LDR   R6, [R5, #+0]
            CMP   R6, R3
            BNE   _full_ram1_test_fail
            STR   R2, [R5, #+0]
            LDR   R6, [R5, #+4]
            CMP   R6, R3
            BNE   _full_ram1_test_fail
            STR   R2, [R5, #+4]

            LDR   R6, [R5, #+8]
            CMP   R6, R3
            BNE   _full_ram1_test_fail
            STR   R2, [R5, #+8]
            LDR   R6, [R5, #+12]
            CMP   R6, R3
            BNE   _full_ram1_test_fail
            STR   R2, [R5, #+12]

            ADDS  R5, R5, #+16
            B     _step_3_loop

            /**** Step 4 ****/
            /* Verify background and write inverted background with addresses decreasing */
_step_4:
            MOVS  R5, R1
            SUBS  R5, R5, #+15
_step_4_loop:
            CMP   R5, R0
            BLO   _step_5

            LDR   R6, [R5, #+12]
            CMP   R6, R2
            BNE   _full_ram1_test_fail
            STR   R3, [R5, #+12]
            LDR   R6, [R5, #+8]
            CMP   R6, R2
            BNE   _full_ram1_test_fail
            STR   R3, [R5, #+8]

            LDR   R6, [R5, #+4]
            CMP   R6, R2
            BNE   _full_ram1_test_fail
            STR   R3, [R5, #+4]
            LDR   R6, [R5, #+0]
            CMP   R6, R2
            BNE   _full_ram1_test_fail
            STR   R3, [R5, #+0]

            SUBS  R5, R5, #+16
            B     _step_4_loop

            /**** Step 5 ****/
            /* Verify inverted background and write background with addresses decreasing */
_step_5:
            MOVS  R5, R1
            SUBS  R5, R5, #+15
_step_5_loop:
            CMP   R5, R0
            BLO   _step_6

            LDR   R6, [R5, #+12]
            CMP   R6, R3
            BNE   _full_ram1_test_fail
            STR   R2, [R5, #+12]
            LDR   R6, [R5, #+8]
            CMP   R6, R3
            BNE   _full_ram1_test_fail
            STR   R2, [R5, #+8]

            LDR   R6, [R5, #+4]
            CMP   R6, R3
            BNE   _full_ram1_test_fail
            STR   R2, [R5, #+4]
            LDR   R6, [R5, #+0]
            CMP   R6, R3
            BNE   _full_ram1_test_fail
            STR   R2, [R5, #+0]
            SUBS  R5, R5, #+16
            B     _step_5_loop

            /**** Step 6 ****/
            /* Verify background with addresses increasing */
_step_6:
            MOVS  R5, R0
_step_6_loop:
            CMP   R5, R1
            BHI   _full_ram1_test_pass
            LDR   R6, [R5, #+0]
            CMP   R6, R2
            BNE   _full_ram1_test_fail
            ADDS  R5, R5, #+4
            B     _step_6_loop

_full_ram1_test_fail:
            MOVS  R4, #1       /* STL_ERR */

_full_ram1_test_pass:
            MOVS  R0, R4
            BX    LR           /* return to the caller */
