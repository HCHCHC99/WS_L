/**
 *******************************************************************************
 * @file  Sysclk.h
 * @brief 系统滴答定时器配置接口
 * @note  第一套时钟配置：200MHz系统时钟，100MHz APB1
 *******************************************************************************
 */

#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include "hc32_ll_clk.h"
#include "Adapter.h"

#define DEBUG_CLOCK              // 开启时钟调试信息
#define CLOCK_OUTPUT             // 开启时钟输出信息

#ifdef DEBUG_CLOCK
    #define CLOCK_DEBUG(fmt, ...)    MAIN_D("[CLOCK_DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define CLOCK_DEBUG(fmt, ...)    ((void)0)
#endif

#ifdef CLOCK_OUTPUT
    #define CLOCK_OUT(fmt, ...)      MAIN_D("[CLOCK_OUT] " fmt, ##__VA_ARGS__)
#else
    #define CLOCK_OUT(fmt, ...)      ((void)0)
#endif

/*==============================================================================
 * 用户配置宏定义 - 时钟源使能(是否启用该时钟源)
 *============================================================================*/

#define SYSTICK_ENABLE_HRC            (0UL)   /*!< 使能HRC(高速RC振荡器) - 第一套不使用 */
#define SYSTICK_ENABLE_MRC            (0UL)   /*!< 使能MRC(中速RC振荡器) */
#define SYSTICK_ENABLE_LRC            (1UL)   /*!< 使能LRC(低速RC振荡器) */
#define SYSTICK_ENABLE_XTAL           (1UL)   /*!< 使能XTAL(外部高速晶振) - 第一套使用XTAL */
#define SYSTICK_ENABLE_XTAL32         (1UL)   /*!< 使能XTAL32(外部32.768kHz晶振) */
#define SYSTICK_ENABLE_PLL            (1UL)   /*!< 使能PLL(锁相环) - 第一套使用PLL倍频 */

/*==============================================================================
 * 用户配置宏定义 - 系统时钟源选择(选哪个作为SYSCLK)
 *============================================================================*/

#define SYSTICK_SYSCLK_SRC           CLK_SYSCLK_SRC_PLL   /*!< 系统时钟源选择: 第一套使用PLL输出200MHz */

/*==============================================================================
 * 用户配置宏定义 - PLL参数(第一套时钟: XTAL(8MHz) * 25 = 200MHz)
 *============================================================================*/

#define SYSTICK_PLL_SRC              CLK_PLL_SRC_XTAL   /*!< PLL输入时钟源: XTAL(8MHz) */
#define SYSTICK_PLL_M                (1UL)              /*!< PLL分频系数M (1~24) - 不分频 */
#define SYSTICK_PLL_N                (50UL)             /*!< PLL倍频系数N (20~480) - 50倍频 */
#define SYSTICK_PLL_P                (2UL)              /*!< PLL分频系数P (2~16) - 2分频输出200MHz */
#define SYSTICK_PLL_Q                (2UL)              /*!< PLL分频系数Q (2~16) - USB等使用 */
#define SYSTICK_PLL_R                (2UL)              /*!< PLL分频系数R (2~16) - 其他外设使用 */

/*==============================================================================
 * 用户配置宏定义 - 总线时钟分频(第一套时钟配置)
 *============================================================================*/

#define SYSTICK_HCLK_DIV           CLK_HCLK_DIV1      /*!< HCLK = 200MHz (不分频) */
#define SYSTICK_PCLK0_DIV         CLK_PCLK0_DIV1      /*!< PCLK0 = 200MHz (APB0) */
#define SYSTICK_PCLK1_DIV         CLK_PCLK1_DIV2      /*!< PCLK1 = 100MHz (APB1) - Timer0使用 */
#define SYSTICK_PCLK2_DIV         CLK_PCLK2_DIV4      /*!< PCLK2 = 50MHz (APB2) */
#define SYSTICK_PCLK3_DIV         CLK_PCLK3_DIV4      /*!< PCLK3 = 50MHz (APB3) */
#define SYSTICK_PCLK4_DIV         CLK_PCLK4_DIV2      /*!< PCLK4 = 100MHz (APB4) */
#define SYSTICK_EXCLK_DIV         CLK_EXCLK_DIV2      /*!< EXCLK = 100MHz (外部时钟2分频) */

/*==============================================================================
 * 用户配置宏定义 - SysTick工作参数
 *============================================================================*/

#define SYSTICK_CLK_SRC             (1UL)   /*!< SysTick时钟源: 0-HCLK/8, 1-HCLK */
#define SYSTICK_INT_INTERVAL_MS     (1UL)   /*!< SysTick中断间隔(毫秒), 推荐1ms */

/*==============================================================================
 * 对外接口函数声明
 *============================================================================*/

void SysTick_Handler(void);

/**
 * @brief SysTick定时器初始化(唯一对外接口)
 * @param 无
 * @retval int32_t 0:成功, -1:失败
 */
int32_t Systick_config(void);
void Print_All_Clock_Freq(void);

#endif /* __SYSTICK_H__ */
