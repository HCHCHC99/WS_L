/**
 *******************************************************************************
 * @file  stl_utility.h
 * @brief This file contains all the functions prototypes of utility.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
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

#ifndef __STL_UTILITY_H__
#define __STL_UTILITY_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stl_common.h"
#include "stl_conf.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup IEC60730_STL
 * @{
 */

/**
 * @addtogroup IEC60730_STL_Utility
 * @{
 */

/*******************************************************************************
 * Global type definitions ('typedef')
*******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @addtogroup STL_UTILITY_Global_Functions
 * @{
 */

#if (STL_PRINT_ENABLE == STL_ON)
#ifndef STL_PRINTF_BUF_MAX_LEN
#define STL_PRINTF_BUF_MAX_LEN          (256UL)
#endif
#define STL_Printf                      (void)STL_RetargetPrintf
#else
#define STL_Printf(...)
#endif

/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
 * Global function prototypes (definition in C source)
 ******************************************************************************/

/**
 * @addgroup STL_Utility_Global_Functions
 * @{
 */
void STL_DelayUS(uint32_t u32Count);
void STL_DelayMS(uint32_t u32Count);

void STL_SafetyFailure(void);

uint32_t STL_PrintfInit(void);

int32_t STL_RetargetPrintf(const char *fmt, ...);
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __STL_UTILITY_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
