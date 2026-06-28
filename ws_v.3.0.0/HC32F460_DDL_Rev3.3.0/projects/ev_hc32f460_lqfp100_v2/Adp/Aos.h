/**
 *******************************************************************************
 * @file  Aos.h
 * @brief AOS (阿波罗事件路由系统) 适配层
 *******************************************************************************
 */

#ifndef __AOS_H__
#define __AOS_H__

#include "hc32_ll.h"

/*==============================================================================
 * 事件源定义 - 常用事件源别名，方便使用
 *============================================================================*/

/* Timer0 事件 */
#define AOS_EVT_TMR0_1_CMP_A        INT_SRC_TMR0_1_CMP_A
#define AOS_EVT_TMR0_1_CMP_B        INT_SRC_TMR0_1_CMP_B
#define AOS_EVT_TMR0_2_CMP_A        INT_SRC_TMR0_2_CMP_A
#define AOS_EVT_TMR0_2_CMP_B        INT_SRC_TMR0_2_CMP_B

/* ADC 事件 */
#define AOS_EVT_ADC1_EOCA           INT_SRC_ADC1_EOCA
#define AOS_EVT_ADC1_EOCB           INT_SRC_ADC1_EOCB
#define AOS_EVT_ADC2_EOCA           INT_SRC_ADC2_EOCA
#define AOS_EVT_ADC2_EOCB           INT_SRC_ADC2_EOCB

/* DMA 事件 */
#define AOS_EVT_DMA1_BTC0           INT_SRC_DMA1_BTC0
#define AOS_EVT_DMA1_BTC1           INT_SRC_DMA1_BTC1
#define AOS_EVT_DMA2_BTC0           INT_SRC_DMA2_BTC0
#define AOS_EVT_DMA2_BTC1           INT_SRC_DMA2_BTC1

/*==============================================================================
 * 目标定义 - 常用目标别名，方便使用
 *============================================================================*/

/* ADC 目标 */
#define AOS_TARGET_ADC1_0           AOS_ADC1_0
#define AOS_TARGET_ADC1_1           AOS_ADC1_1
#define AOS_TARGET_ADC2_0           AOS_ADC2_0
#define AOS_TARGET_ADC2_1           AOS_ADC2_1

/* DMA 目标 */
#define AOS_TARGET_DMA1_0           AOS_DMA1_0
#define AOS_TARGET_DMA1_1           AOS_DMA1_1
#define AOS_TARGET_DMA1_2           AOS_DMA1_2
#define AOS_TARGET_DMA1_3           AOS_DMA1_3
#define AOS_TARGET_DMA2_0           AOS_DMA2_0
#define AOS_TARGET_DMA2_1           AOS_DMA2_1
#define AOS_TARGET_DMA2_2           AOS_DMA2_2
#define AOS_TARGET_DMA2_3           AOS_DMA2_3 

/* Timer 目标 */
#define AOS_TARGET_TMR0              AOS_TMR0
#define AOS_TARGET_TMR6_0            AOS_TMR6_0
#define AOS_TARGET_TMR6_1            AOS_TMR6_1

/*==============================================================================
 * 函数声明
 *============================================================================*/

void AOS_Init(void);
void AOS_DeInit(void);
void AOS_Enable(void);
void AOS_Disable(void);

void AOS_Connect(uint32_t u32Target, en_event_src_t enSource);
void AOS_Disconnect(uint32_t u32Target);

void AOS_EnableCommonTrigger(uint32_t u32Target, uint32_t u32CommonTrigger);
void AOS_DisableCommonTrigger(uint32_t u32Target, uint32_t u32CommonTrigger);

void AOS_SoftwareTrigger(void);

#endif /* __AOS_H__ */
