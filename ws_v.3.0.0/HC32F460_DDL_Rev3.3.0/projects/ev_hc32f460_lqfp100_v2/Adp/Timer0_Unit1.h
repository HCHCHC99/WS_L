/**
 *******************************************************************************
 * @file  Timer0_Unit1.h
 * @brief Timer0单元1的周期中断驱动 (包含通道A和通道B)
 *******************************************************************************
 */

#ifndef __TIMER0_UNIT1_H__
#define __TIMER0_UNIT1_H__

#include "hc32_ll.h"

/*==============================================================================
 * 配置宏定义
 *============================================================================*/

#define TMR0_UNIT1                          (CM_TMR0_1)
#define TMR0_UNIT1_CLK                      (FCG2_PERIPH_TMR0_1)
#define TMR0_UNIT1_IRQn                     (INT006_IRQn)

#define TMR0_CLK_SRC_INTERN                  TMR0_CLK_SRC_INTERN_CLK

/*==============================================================================
 * 枚举类型定义 - 加上 Unit1 后缀
 *============================================================================*/

typedef enum {
    TICK_RESET_1,      /* Unit1 的 TICK_RESET */
    TICK_NO_RESET_1    /* Unit1 的 TICK_NO_RESET */
} ReCount1_t;

typedef enum {
    TMR0_CHANNEL_A_1,  /* Unit1 的通道A */
    TMR0_CHANNEL_B_1   /* Unit1 的通道B */
} TMR0_Channel1_t;

/*==============================================================================
 * 对外接口函数声明
 *============================================================================*/

int32_t TMR0_Unit1_Init(TMR0_Channel1_t ch, uint32_t period_us, ReCount1_t cmd);
int32_t TMR0_Unit1_DeInit(TMR0_Channel1_t ch);
int32_t TMR0_Unit1_Reconfig(TMR0_Channel1_t ch);
void TMR0_Unit1_Start(TMR0_Channel1_t ch);
void TMR0_Unit1_Stop(TMR0_Channel1_t ch);
uint16_t TMR0_Unit1_GetCount(TMR0_Channel1_t ch);

#endif /* __TIMER0_UNIT1_H__ */
