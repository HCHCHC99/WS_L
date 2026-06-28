/**
 *******************************************************************************
 * @file  stl_utility.c
 * @brief This file provides utility functions for STL.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2023-01-15       CDT             Modify USART_SR_TXE to USART_SR_TC in STL_ConsoleOutputChar()
   2023-05-31       CDT             Modify register USART DR to USART TDR
   2024-06-04       CDT             Use STL_RetargetPrintf() replace printf() of the C lib.
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include "stl_utility.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_usart.h"
#include "hc32_ll_utility.h"

/**
 * @addtogroup IEC60730_STL
 * @{
 */

/**
 * @defgroup IEC60730_STL_Utility IEC60730 STL Utility
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 * @defgroup STL_IEC60730_Utility_Global_Functions STL IEC60730 Utility Global Functions
 * @{
 */

/**
 * @brief Delay function, delay us approximately
 * @param [in] u32Count                 us
 * @retval None
 */
void STL_DelayUS(uint32_t u32Count)
{
    DDL_DelayUS(u32Count);
}

/**
 * @brief Delay function, delay ms approximately
 * @param [in] u32Count                 ms
 * @retval None
 */
void STL_DelayMS(uint32_t u32Count)
{
    DDL_DelayMS(u32Count);
}

/**
 * @brief STL test safety failure handle
 * @param None
 * @retval None
 */
void STL_SafetyFailure(void)
{
#if (STL_RESET_AT_FAILURE == STL_ON)
    NVIC_SystemReset(); /* Generate system reset */
#endif
}

#if (STL_PRINT_ENABLE == STL_ON)

/**
 * @brief  Transmit character.
 * @param  [in] cData                   The character for transmitting
 * @retval uint32_t:
 *           - STL_OK:                  Transmit successfully.
 *           - STL_ERR:                 Transmit timeout.
 */
__WEAKDEF uint32_t STL_ConsoleOutputChar(char cData)
{
    uint32_t u32Ret = STL_ERR;
    uint32_t u32TxEmpty = 0UL;
    __IO uint32_t u32TmpCount = 0UL;
    uint32_t u32Timeout = 10000UL;

    /* Wait TX data register empty */
    while ((u32TmpCount <= u32Timeout) && (0UL == u32TxEmpty)) {
        u32TxEmpty = READ_REG32_BIT(STL_PRINTF_DEVICE->SR, USART_SR_TC);
        u32TmpCount++;
    }

    if (0UL != u32TxEmpty) {
        WRITE_REG16(STL_PRINTF_DEVICE->TDR, (uint16_t)cData);
        u32Ret = STL_OK;
    }

    return u32Ret;
}

/**
 * @brief  STL printf.
 * @param  fmt                  String format
 * @retval None
 */
int32_t STL_RetargetPrintf(const char *fmt, ...)
{
    va_list arg;
    int32_t i;
    int32_t i32Len;
    static char acString[STL_PRINTF_BUF_MAX_LEN];

    va_start(arg, fmt);
    i32Len = vsnprintf(acString, STL_PRINTF_BUF_MAX_LEN - 1UL, fmt, arg);
    va_end(arg);

    for (i = 0; i < i32Len; i++) {
        if (STL_OK != STL_ConsoleOutputChar(acString[i])) {
            break;
        }
    }

    return i;
}

/**
 * @brief  Initialize printf function
 * @param  None
 * @retval None
 */
uint32_t STL_PrintfInit(void)
{
    uint32_t u32Div;
    float32_t f32Error;
    uint32_t u32Ret = STL_ERR;
    stc_usart_uart_init_t stcUartInit;

    /* Set TX port function */
    GPIO_SetFunc(STL_PRINTF_PORT, STL_PRINTF_PIN, STL_PRINTF_PORT_FUNC);

    /* Enable clock  */
    STL_PRINTF_DEVICE_FCG_ENALBE();

    /***************************************************************************
     * Configure UART
     ***************************************************************************
     * Baud rate: STL_PRINTF_BAUDRATE
     * Bit direction: LSB
     * Data bits: 8
     * Stop bits: 1
     * Parity: None
     * Sampling bits: 8
     **************************************************************************/
    /* Configure UART */
    (void)USART_UART_StructInit(&stcUartInit);
    stcUartInit.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    (void)USART_UART_Init(STL_PRINTF_DEVICE, &stcUartInit, NULL);

    for (u32Div = 0UL; u32Div <= USART_CLK_DIV64; u32Div++) {
        USART_SetClockDiv(STL_PRINTF_DEVICE, u32Div);
        if ((LL_OK == USART_SetBaudrate(STL_PRINTF_DEVICE, STL_PRINTF_BAUDRATE, &f32Error)) && \
            ((-STL_PRINTF_BAUDRATE_ERR_MAX <= f32Error) && (f32Error <= STL_PRINTF_BAUDRATE_ERR_MAX))) {
            USART_FuncCmd(STL_PRINTF_DEVICE, USART_TX, ENABLE);
            u32Ret = STL_OK;
            break;
        }
    }

    return u32Ret;
}

#endif /* STL_PRINT_ENABLE */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
