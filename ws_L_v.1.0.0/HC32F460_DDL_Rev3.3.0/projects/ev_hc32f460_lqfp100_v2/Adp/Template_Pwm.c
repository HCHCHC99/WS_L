#include "Template_Pwm.h"

/*******************************************************************************
 * Function implementation
 ******************************************************************************/
void Template_Pwm_TmrAConfig(void)
{
    stc_tmra_init_t stcTmraInit;
    stc_tmra_pwm_init_t stcPwmInit;
    stc_clock_freq_t stcClkFreq;
    uint32_t u32TimerClk;
    uint32_t u32PeriodValue;
    
    /* 获取PCLK1频率 (TimerA使用PCLK1) */
    (void)CLK_GetClockFreq(&stcClkFreq);
    u32TimerClk = stcClkFreq.u32Pclk1Freq;
    
    /* 计算周期值以实现设定的PWM频率 */
    u32PeriodValue = u32TimerClk / PWM_FREQ;
    
    /* 根据计数模式调整周期值 */
    if (TMRA_MD == TMRA_MD_TRIANGLE) {
        u32PeriodValue = u32PeriodValue / 2U;
    }
    
    /* 周期值减1 */
    u32PeriodValue = u32PeriodValue - 1U;
    
    /* 使能TimerA时钟 */
    FCG_Fcg2PeriphClockCmd(TMRA_PERIPH_CLK, ENABLE);

    /* 初始化TimerA基本配置 */
    (void)TMRA_StructInit(&stcTmraInit);
    stcTmraInit.sw_count.u8CountMode = TMRA_MD;
    stcTmraInit.sw_count.u8CountDir  = TMRA_DIR;
    stcTmraInit.u32PeriodValue = u32PeriodValue;
    (void)TMRA_Init(TMRA_UNIT, &stcTmraInit);

    /* 配置PWM输出 */
    (void)TMRA_PWM_StructInit(&stcPwmInit);
    
#if (APP_FUNC == APP_FUNC_NORMAL_SINGLE_PWM)
    stcPwmInit.u32CompareValue = (u32PeriodValue * TMRA_PWM_DUTY) / 100U;
    GPIO_SetFunc(TMRA_PWM_PORT, TMRA_PWM_PIN, TMRA_PWM_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWM_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWM_CH, ENABLE);

#elif (APP_FUNC == APP_FUNC_SINGLE_EDGE_ALIGNED_PWM)
    stcPwmInit.u32CompareValue = (u32PeriodValue * TMRA_PWMX_DUTY) / 100U;
    GPIO_SetFunc(TMRA_PWMX_PORT, TMRA_PWMX_PIN, TMRA_PWMX_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWMX_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWMX_CH, ENABLE);

    stcPwmInit.u32CompareValue = (u32PeriodValue * TMRA_PWMY_DUTY) / 100U;
    GPIO_SetFunc(TMRA_PWMY_PORT, TMRA_PWMY_PIN, TMRA_PWMY_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWMY_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWMY_CH, ENABLE);

#elif (APP_FUNC == APP_FUNC_TWO_EDGE_SYMMETRIC_PWM)
    stcPwmInit.u32CompareValue        = (u32PeriodValue * TMRA_PWMX_DUTY) / 100U;
    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HOLD;
    GPIO_SetFunc(TMRA_PWMX_PORT, TMRA_PWMX_PIN, TMRA_PWMX_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWMX_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWMX_CH, ENABLE);

    stcPwmInit.u32CompareValue  = (u32PeriodValue * TMRA_PWMY_DUTY) / 100U;
    stcPwmInit.u16StartPolarity = TMRA_PWM_LOW;
    stcPwmInit.u16StopPolarity  = TMRA_PWM_LOW;
    GPIO_SetFunc(TMRA_PWMY_PORT, TMRA_PWMY_PIN, TMRA_PWMY_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWMY_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWMY_CH, ENABLE);

#elif (APP_FUNC == APP_FUNC_COMPLEMENTARY_POLARITY_PWM)
    /* 配置通道1 - 高有效 */
    stcPwmInit.u32CompareValue        = (u32PeriodValue * TMRA_PWMX_DUTY) / 100U;
    stcPwmInit.u16StartPolarity       = TMRA_PWM_LOW;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_LOW;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_LOW;
    
    GPIO_SetFunc(TMRA_PWMX_PORT, TMRA_PWMX_PIN, TMRA_PWMX_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWMX_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWMX_CH, ENABLE);

    /* 配置通道2 - 低有效 */
    stcPwmInit.u32CompareValue        = (u32PeriodValue * TMRA_PWMY_DUTY) / 100U;
    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
    
    GPIO_SetFunc(TMRA_PWMY_PORT, TMRA_PWMY_PIN, TMRA_PWMY_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWMY_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWMY_CH, ENABLE);

#elif (APP_FUNC == APP_FUNC_FOUR_CH_LOW_PWM)
    /* 配置通道1 (PB6) - 低有效 */
    stcPwmInit.u32CompareValue        = (u32PeriodValue * TMRA_PWMX_DUTY) / 100U;
    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
    
    GPIO_SetFunc(TMRA_PWMX_PORT, TMRA_PWMX_PIN, TMRA_PWMX_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWMX_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWMX_CH, ENABLE);

    /* 配置通道2 (PB7) - 低有效 */
    stcPwmInit.u32CompareValue        = (u32PeriodValue * TMRA_PWMY_DUTY) / 100U;
    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
    
    GPIO_SetFunc(TMRA_PWMY_PORT, TMRA_PWMY_PIN, TMRA_PWMY_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWMY_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWMY_CH, ENABLE);

    /* 配置通道3 (PB8) - 低有效 */
    stcPwmInit.u32CompareValue        = (u32PeriodValue * TMRA_PWMZ_DUTY) / 100U;
    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
    
    GPIO_SetFunc(TMRA_PWMZ_PORT, TMRA_PWMZ_PIN, TMRA_PWMZ_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWMZ_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWMZ_CH, ENABLE);

    /* 配置通道4 (PB9) - 低有效 */
    stcPwmInit.u32CompareValue        = (u32PeriodValue * TMRA_PWMW_DUTY) / 100U;
    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
    
    GPIO_SetFunc(TMRA_PWMW_PORT, TMRA_PWMW_PIN, TMRA_PWMW_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWMW_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWMW_CH, ENABLE);
#endif

#if (TMRA_MD == TMRA_MD_TRIANGLE)
    TMRA_SetCountValue(TMRA_UNIT, 1UL);
#endif

    TMRA_Start(TMRA_UNIT);  
}
