/**
 *******************************************************************************
 * @file  Timer0_Unit1.c
 * @brief Timer0单元1的周期中断驱动 (包含通道A和通道B)
 *******************************************************************************
 */

#include "Timer0_Unit1.h"
#include "TickTimer.h"

/*==============================================================================
 * 静态变量
 *============================================================================*/

static uint16_t s_saved_count[2] = {0, 0};
static uint32_t s_target_period[2] = {1000, 1000};

/*==============================================================================
 * 静态函数
 *============================================================================*/

static void TMR0_Unit1_IRQHandler(void);
static uint32_t TMR0_GetChannelConfig(TMR0_Channel1_t ch, uint32_t *int_src, uint32_t *int_flag, uint32_t *int_type);

/*==============================================================================
 * 宏定义
 *============================================================================*/

#define USEC_TO_COUNT(us, clockFreqInHz) (uint16_t)(((uint64_t)(us) * (clockFreqInHz)) / 1000000U)

/*==============================================================================
 * 函数实现
 *============================================================================*/

static uint32_t TMR0_GetChannelConfig(TMR0_Channel1_t ch, uint32_t *int_src, uint32_t *int_flag, uint32_t *int_type)
{
    if (ch == TMR0_CHANNEL_A_1) {
        *int_src = INT_SRC_TMR0_1_CMP_A;
        *int_flag = TMR0_FLAG_CMP_A;
        *int_type = TMR0_INT_CMP_A;
        return TMR0_CH_A;
    } else {
        *int_src = INT_SRC_TMR0_1_CMP_B;
        *int_flag = TMR0_FLAG_CMP_B;
        *int_type = TMR0_INT_CMP_B;
        return TMR0_CH_B;
    }
}

int32_t TMR0_Unit1_Init(TMR0_Channel1_t ch, uint32_t period_us, ReCount1_t cmd)
{
    stc_tmr0_init_t stcTmr0Init;
    stc_irq_signin_config_t stcIrqSignConfig;
    stc_clock_freq_t stcClkFreq;
    uint32_t u32Pclk1, u32Compare;
    uint32_t timer_ch, int_src, int_flag, int_type;
    static uint8_t s_irq_registered = 0;
    uint8_t ch_idx = (ch == TMR0_CHANNEL_A_1) ? 0 : 1;
    
    s_target_period[ch_idx] = period_us;
    timer_ch = TMR0_GetChannelConfig(ch, &int_src, &int_flag, &int_type);
    
    FCG_Fcg2PeriphClockCmd(TMR0_UNIT1_CLK, ENABLE);
    
    CLK_GetClockFreq(&stcClkFreq);
    u32Pclk1 = stcClkFreq.u32Pclk1Freq;
    
    /* 修改：使用 DIV1 (不分频) 实现最高精度 */
    u32Compare = USEC_TO_COUNT(period_us, u32Pclk1);  /* 直接使用 PCLK1，不分频 */
    u32Compare = u32Compare - 1UL;
    
    (void)TMR0_StructInit(&stcTmr0Init);
    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_INTERN;
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV1;  /* 修改：使用 DIV1 */
    stcTmr0Init.u32Func = TMR0_FUNC_CMP;
    stcTmr0Init.u16CompareValue = (uint16_t)u32Compare;
    
    if (LL_OK != TMR0_Init(TMR0_UNIT1, timer_ch, &stcTmr0Init)) {
        return -1;
    }
    
    TMR0_SetCountValue(TMR0_UNIT1, timer_ch, s_saved_count[ch_idx]);
    TMR0_ClearStatus(TMR0_UNIT1, int_flag);
    TMR0_IntCmd(TMR0_UNIT1, int_type, ENABLE);
    
    if (!s_irq_registered) {
        stcIrqSignConfig.enIntSrc = int_src;
        stcIrqSignConfig.enIRQn = TMR0_UNIT1_IRQn;
        stcIrqSignConfig.pfnCallback = &TMR0_Unit1_IRQHandler;
        
        if (LL_OK != INTC_IrqSignIn(&stcIrqSignConfig)) {
            return -1;
        }
        
        NVIC_ClearPendingIRQ(TMR0_UNIT1_IRQn);
        NVIC_SetPriority(TMR0_UNIT1_IRQn, 1UL);
        NVIC_EnableIRQ(TMR0_UNIT1_IRQn);
        
        s_irq_registered = 1;
    }
    
    TMR0_Start(TMR0_UNIT1, timer_ch);
    
    if (cmd == TICK_RESET_1) {
        tickTimer_Init();
    }
    
    return 0;
}

int32_t TMR0_Unit1_DeInit(TMR0_Channel1_t ch)
{
    uint32_t timer_ch, int_src, int_flag, int_type;
    uint8_t ch_idx = (ch == TMR0_CHANNEL_A_1) ? 0 : 1;
    
    timer_ch = TMR0_GetChannelConfig(ch, &int_src, &int_flag, &int_type);
    
    s_saved_count[ch_idx] = TMR0_GetCountValue(TMR0_UNIT1, timer_ch);
    
    TMR0_Stop(TMR0_UNIT1, timer_ch);
    TMR0_IntCmd(TMR0_UNIT1, int_type, DISABLE);
    TMR0_ClearStatus(TMR0_UNIT1, int_flag);
    
    (void)TMR0_DeInit(TMR0_UNIT1);
    
    if (ch == TMR0_CHANNEL_B_1) {
        NVIC_DisableIRQ(TMR0_UNIT1_IRQn);
        NVIC_ClearPendingIRQ(TMR0_UNIT1_IRQn);
    }
    
    return 0;
}

int32_t TMR0_Unit1_Reconfig(TMR0_Channel1_t ch)
{
    stc_clock_freq_t stcClkFreq;
    uint32_t u32Pclk1, u32Compare;
    uint32_t timer_ch, int_src, int_flag, int_type;
    uint8_t ch_idx = (ch == TMR0_CHANNEL_A_1) ? 0 : 1;
    
    timer_ch = TMR0_GetChannelConfig(ch, &int_src, &int_flag, &int_type);
    
    TMR0_Stop(TMR0_UNIT1, timer_ch);
    
    CLK_GetClockFreq(&stcClkFreq);
    u32Pclk1 = stcClkFreq.u32Pclk1Freq;
    
    /* 修改：使用 DIV1 重新计算 */
    u32Compare = USEC_TO_COUNT(s_target_period[ch_idx], u32Pclk1);
    u32Compare = u32Compare - 1UL;
    
    TMR0_SetCompareValue(TMR0_UNIT1, timer_ch, (uint16_t)u32Compare);
    TMR0_SetCountValue(TMR0_UNIT1, timer_ch, s_saved_count[ch_idx]);
    TMR0_Start(TMR0_UNIT1, timer_ch);
    
    return 0;
}

void TMR0_Unit1_Start(TMR0_Channel1_t ch)
{
    uint32_t timer_ch = (ch == TMR0_CHANNEL_A_1) ? TMR0_CH_A : TMR0_CH_B;
    TMR0_Start(TMR0_UNIT1, timer_ch);
}

void TMR0_Unit1_Stop(TMR0_Channel1_t ch)
{
    uint32_t timer_ch = (ch == TMR0_CHANNEL_A_1) ? TMR0_CH_A : TMR0_CH_B;
    TMR0_Stop(TMR0_UNIT1, timer_ch);
}

uint16_t TMR0_Unit1_GetCount(TMR0_Channel1_t ch)
{
    uint32_t timer_ch = (ch == TMR0_CHANNEL_A_1) ? TMR0_CH_A : TMR0_CH_B;
    return TMR0_GetCountValue(TMR0_UNIT1, timer_ch);
}

static void TMR0_Unit1_IRQHandler(void)
{
    if (TMR0_GetStatus(TMR0_UNIT1, TMR0_FLAG_CMP_A) == SET) {
        TMR0_ClearStatus(TMR0_UNIT1, TMR0_FLAG_CMP_A);
        /* 通道A的用户代码 */
    }
    
    if (TMR0_GetStatus(TMR0_UNIT1, TMR0_FLAG_CMP_B) == SET) {
        TMR0_ClearStatus(TMR0_UNIT1, TMR0_FLAG_CMP_B);
        /* 通道B的用户代码 */
    }
}

/*******************************************************************************
 * 中断服务函数
 ******************************************************************************/

void TMR0_1_IRQHandler(void)
{
    TMR0_Unit1_IRQHandler();
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
