/**
 *******************************************************************************
 * @file  Adc.c
 * @brief ADC Driver for HC32F460 - Generic framework with multiple instances
 *        DMA 相关操作委托给 Dma 层处理
 *******************************************************************************
 */

#include "Adc.h"
#include "Dma.h"
#include "rtt_log.h"
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 * Local variables ('static')
 ******************************************************************************/

/* ADC 实例数组 */
static stc_adc_instance_t s_astcAdcInstances[ADC_MAX_INSTANCES];
static uint8_t s_u8AdcInstanceCount = 0;

/* ADC 初始化标志 */
static bool s_bAdcInitialized = false;

/* 是否有 DMA 模式实例 */
static bool s_bHasDmaInstance = false;

/* DMA 实例 ID 映射：ADC 实例 ID → DMA 实例 ID（仅 DMA 模式有效） */
static int8_t s_a8AdcIdToDmaId[ADC_MAX_INSTANCES];

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

static void Adc_InitConfig(void);
static void Adc_SetPinAnalogMode(uint8_t u8Port, uint8_t u8Pin);
static void Adc_HardTriggerConfig(void);
static void Adc_IrqConfig(void);
static void Adc_HardTriggerStart(void);

static void Timer0_Config(uint32_t u32IntervalUs);

static void ADC1_SeqA_IrqCallback(void);

static uint16_t Adc_CalcVoltage(uint16_t u16AdcValue);
static void Adc_ProcessInterruptChannels(void);
static char Adc_GetPortLetter(uint8_t u8Port);
static uint8_t Adc_GetPinNumber(uint16_t u16Pin);

/*******************************************************************************
 * ADC 辅助函数
 ******************************************************************************/

/**
 * @brief  Get pin number from GPIO pin macro
 */
static uint8_t Adc_GetPinNumber(uint16_t u16Pin)
{
    switch (u16Pin) {
        case GPIO_PIN_00: return 0;
        case GPIO_PIN_01: return 1;
        case GPIO_PIN_02: return 2;
        case GPIO_PIN_03: return 3;
        case GPIO_PIN_04: return 4;
        case GPIO_PIN_05: return 5;
        case GPIO_PIN_06: return 6;
        case GPIO_PIN_07: return 7;
        case GPIO_PIN_08: return 8;
        case GPIO_PIN_09: return 9;
        case GPIO_PIN_10: return 10;
        case GPIO_PIN_11: return 11;
        case GPIO_PIN_12: return 12;
        case GPIO_PIN_13: return 13;
        case GPIO_PIN_14: return 14;
        case GPIO_PIN_15: return 15;
        default: return 0;
    }
}

/**
 * @brief  Get port letter for display
 */
static char Adc_GetPortLetter(uint8_t u8Port)
{
    switch (u8Port) {
        case GPIO_PORT_A: return 'A';
        case GPIO_PORT_B: return 'B';
        case GPIO_PORT_C: return 'C';
        case GPIO_PORT_D: return 'D';
        case GPIO_PORT_E: return 'E';
        case GPIO_PORT_H: return 'H';
        default: return '?';
    }
}

/**
 * @brief  Set a single ADC pin to analog mode
 */
static void Adc_SetPinAnalogMode(uint8_t u8Port, uint8_t u8Pin)
{
    stc_gpio_init_t stcGpioInit;
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
	LL_PERIPH_WE(LL_PERIPH_GPIO);
    (void)GPIO_Init(u8Port, u8Pin, &stcGpioInit);
	LL_PERIPH_WP(LL_PERIPH_GPIO);
}

/**
 * @brief  Initialize ADC hardware
 */
static void Adc_InitConfig(void)
{
    stc_adc_init_t stcAdcInit;

    /* 1. Enable ADC peripheral clock */
    FCG_Fcg3PeriphClockCmd(ADC_PERIPH_CLK, ENABLE);

    /* 2. Modify the default value depends on the application */
    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode = ADC_MD_SEQA_SINGLESHOT;  /* 序列A单次转换模式 */

    /* 3. Initialize ADC */
    (void)ADC_Init(ADC_UNIT, &stcAdcInit);

    /* 4. Configure all ADC pins and enable channels */
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        stc_adc_instance_t *pstcInst = &s_astcAdcInstances[i];
        
        /* 设置引脚为模拟模式 */
        Adc_SetPinAnalogMode(pstcInst->u8Port, pstcInst->u8Pin);
        ADC_Adp_DEBUG("ADC CH%d pin initialized (P%c%d)\r\n", 
               pstcInst->u8Channel,
               Adc_GetPortLetter(pstcInst->u8Port),
               Adc_GetPinNumber(pstcInst->u8Pin));
        
        /* 使能 ADC 通道 */
        ADC_ChCmd(ADC_UNIT, ADC_SEQ_A, pstcInst->u8Channel, ENABLE);
        ADC_Adp_DEBUG("ADC SEQ_A CH%d enabled (%s mode)\r\n", 
               pstcInst->u8Channel,
               (pstcInst->enMode == ADC_MODE_INTERRUPT) ? "Interrupt" : "DMA");
    }
    
    ADC_Adp_DEBUG("ADC initialized: %d channels in SEQ_A\r\n", s_u8AdcInstanceCount);
}

/**
 * @brief  Timer0 configuration for ADC trigger
 * @param  u32IntervalUs 触发间隔(us)，例如 1000 = 1ms
 */
static void Timer0_Config(uint32_t u32IntervalUs)
{
    stc_tmr0_init_t stcTmr0Init;
    
    /* Enable Timer0 peripheral clock */
    FCG_Fcg2PeriphClockCmd(TMR0_PERIPH_CLK, ENABLE);
    
    /* 计算定时器时钟频率 */
    uint32_t u32TimerClk = CLK_GetBusClockFreq(CLK_BUS_PCLK1) / (1UL << (TMR0_CLK_DIV >> TMR0_BCONR_CKDIVA_POS));
    
    /* 计算频率: freq = 1 / interval */
    uint32_t u32Freq = 1000000UL / u32IntervalUs;  /* 转换为 Hz */
    uint16_t u16CompareValue = (uint16_t)(u32TimerClk / u32Freq) - 1;
    
    /* Timer0 structure initialize */
    (void)TMR0_StructInit(&stcTmr0Init);
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV;
    stcTmr0Init.u32Func = TMR0_FUNC_CMP;
    stcTmr0Init.u16CompareValue = u16CompareValue;
    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_INTERN_CLK;
    
    /* Initialize Timer0 */
    (void)TMR0_Init(TMR0_UNIT, TMR0_CH, &stcTmr0Init);
    
    ADC_Adp_DEBUG("Timer0 configured for ADC trigger, interval=%lu us, compare=%u\r\n", 
           u32IntervalUs, u16CompareValue);
}

/**
 * @brief  ADC hard trigger configuration
 */
static void Adc_HardTriggerConfig(void)
{
    /* 配置 ADC 硬件触发 */
    ADC_TriggerConfig(ADC_UNIT, ADC_SEQ_A, ADC_SEQA_HARDTRIG);
    ADC_TriggerCmd(ADC_UNIT, ADC_SEQ_A, ENABLE);
    
    ADC_Adp_DEBUG("SEQ_A: Timer0 -> ADC (mixed interrupt/DMA modes)\r\n");
}

/**
 * @brief  ADC interrupt configuration
 */
static void Adc_IrqConfig(void)
{
    stc_irq_signin_config_t stcIrq;
    uint8_t u8AdcIntEn = 0U;

    /* 检查是否有中断模式的实例 */
    uint8_t u8HasInterruptMode = 0U;
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        if (s_astcAdcInstances[i].enMode == ADC_MODE_INTERRUPT) {
            u8HasInterruptMode = 1U;
            break;
        }
    }

    /* 如果有中断模式实例，使能 ADC 中断 */
    if (u8HasInterruptMode) {
        stcIrq.enIntSrc    = ADC_SEQA_INT_SRC;
        stcIrq.enIRQn      = ADC_SEQA_INT_IRQn;
        stcIrq.pfnCallback = &ADC1_SeqA_IrqCallback;
        (void)INTC_IrqSignIn(&stcIrq);
        NVIC_ClearPendingIRQ(stcIrq.enIRQn);
        NVIC_SetPriority(stcIrq.enIRQn, ADC_SEQA_INT_PRIO);
        NVIC_EnableIRQ(stcIrq.enIRQn);
        u8AdcIntEn |= ADC_INT_EOCA;
        ADC_Adp_DEBUG("SEQ_A interrupt enabled for interrupt-mode channels\r\n");
    } else {
        ADC_IntCmd(ADC_UNIT, ADC_INT_EOCA, DISABLE);
        ADC_Adp_DEBUG("SEQ_A interrupt disabled (no interrupt-mode channels)\r\n");
    }
    
    /* 使能中断 */
    if (u8AdcIntEn != 0U) {
        ADC_IntCmd(ADC_UNIT, u8AdcIntEn, ENABLE);
    }
}

/**
 * @brief  Process all interrupt mode channels in ADC interrupt
 */
static void Adc_ProcessInterruptChannels(void)
{
    uint16_t u16AdcValue;
    
    /* 遍历所有实例，处理中断模式的通道 */
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        stc_adc_instance_t *pstcInst = &s_astcAdcInstances[i];
        
        if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
            /* 读取 ADC 值 */
            u16AdcValue = ADC_GetValue(ADC_UNIT, pstcInst->u8Channel);
            
            /* 更新实例数据 */
            pstcInst->u16LatestValue = u16AdcValue;
            pstcInst->u8ValueUpdated = 1U;
            pstcInst->u32SampleCount++;
            
            /* 如果注册了回调函数，调用它 */
            if (pstcInst->pfnCallback != NULL) {
                pstcInst->pfnCallback(u16AdcValue);
            }
        }
    }
}

/**
 * @brief  ADC1 Sequence A interrupt callback
 */
static void ADC1_SeqA_IrqCallback(void)
{
    /* 清除中断标志 */
    ADC_ClearStatus(ADC_UNIT, ADC_FLAG_EOCA);
    
    /* 处理所有中断模式的通道 */
    Adc_ProcessInterruptChannels();
}

/**
 * @brief  Start hardware triggers
 */
static void Adc_HardTriggerStart(void)
{
    TMR0_Start(TMR0_UNIT, TMR0_CH);
    ADC_Adp_DEBUG("Timer0 started for SEQ_A\r\n");
}

/*******************************************************************************
 * 电压计算函数
 ******************************************************************************/

static uint16_t Adc_CalcVoltage(uint16_t u16AdcValue)
{
    return (uint16_t)((((float32_t)(u16AdcValue) * ADC_VREF) / ((float32_t)ADC_ACCURACY)) * 1000.F);
}

/*******************************************************************************
 * API 函数 - 对外接口
 ******************************************************************************/

/**
 * @brief  Create an ADC instance
 * @param  pstcConfig  ADC 配置结构体
 * @return ADC 实例 ID (0-7)，失败返回 0xFF
 */
uint8_t Adc_Create(stc_adc_config_t *pstcConfig)
{
    if (s_u8AdcInstanceCount >= ADC_MAX_INSTANCES) {
        ADC_Adp_DEBUG("ADC instance full! Max %d\r\n", ADC_MAX_INSTANCES);
        return 0xFF;
    }
    
    /* 检查通道是否已存在 */
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        if (s_astcAdcInstances[i].u8Channel == pstcConfig->u8Channel) {
            ADC_Adp_DEBUG("ADC CH%d already exists! Skip\r\n", pstcConfig->u8Channel);
            return 0xFF;
        }
    }
    
    uint8_t u8Id = s_u8AdcInstanceCount;
    stc_adc_instance_t *pstcInst = &s_astcAdcInstances[u8Id];
    
    /* 初始化实例 */
    memset(pstcInst, 0, sizeof(stc_adc_instance_t));
    pstcInst->u8Id = u8Id;
    pstcInst->u8Channel = pstcConfig->u8Channel;
    pstcInst->enMode = pstcConfig->enMode;
    pstcInst->u8Port = pstcConfig->stcPin.u8Port;
    pstcInst->u8Pin = pstcConfig->stcPin.u8Pin;
    pstcInst->pfnCallback = pstcConfig->pfnCallback;
    
    /* 初始化 DMA ID 映射为无效 */
    s_a8AdcIdToDmaId[u8Id] = -1;
    
    if (pstcConfig->enMode == ADC_MODE_DMA) {
        pstcInst->u16DmaBufferSize = pstcConfig->stcDmaConfig.u16BufferSize;
        pstcInst->u8DmaChannel = pstcConfig->stcDmaConfig.u8DmaChannel;
        s_bHasDmaInstance = true;
    }
    
    s_u8AdcInstanceCount++;
    
    ADC_Adp_DEBUG("ADC instance created: ID=%d, CH=%d, Mode=%s, Pin=P%c%d\r\n",
           u8Id, pstcInst->u8Channel,
           (pstcInst->enMode == ADC_MODE_INTERRUPT) ? "Interrupt" : "DMA",
           Adc_GetPortLetter(pstcInst->u8Port),
           Adc_GetPinNumber(pstcInst->u8Pin));
    
    return u8Id;
}

/**
 * @brief  Initialize ADC driver (call after creating all instances)
 * @note   如果存在 DMA 模式实例，会自动创建并初始化对应的 DMA 通道
 */
void Adc_Init(void)
{
    if (s_bAdcInitialized) {
        ADC_Adp_DEBUG("ADC driver already initialized\r\n");
        return;
    }
    
    if (s_u8AdcInstanceCount == 0) {
        ADC_Adp_DEBUG("No ADC instance created! Call Adc_Create first\r\n");
        return;
    }
    
    /* 1. 初始化 ADC 硬件 */
    Adc_InitConfig();
    
    /* 2. 如果有 DMA 模式实例，通过 Dma 层创建并初始化 DMA 通道 */
    if (s_bHasDmaInstance) {
        for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
            stc_adc_instance_t *pstcInst = &s_astcAdcInstances[i];
            
            if (pstcInst->enMode == ADC_MODE_DMA) {
                /* 构造 DMA 配置 */
                stc_dma_config_t stcDmaConfig;
                memset(&stcDmaConfig, 0, sizeof(stc_dma_config_t));
                
                stcDmaConfig.u8DmaUnit      = 1;  /* 使用 DMA1 */
                stcDmaConfig.u8Channel      = pstcInst->u8DmaChannel;
                stcDmaConfig.enDir          = DMA_DIR_PERIPH_TO_MEM;
                stcDmaConfig.enTransMode    = DMA_TRANS_MODE_REPEAT;
                stcDmaConfig.u32SrcAddr     = (uint32_t)((uint32_t)&ADC_UNIT->DR0 + (pstcInst->u8Channel * 2U));
                stcDmaConfig.u32DestAddr    = 0;  /* 由 Dma 层分配缓冲区后设置 */
                stcDmaConfig.u32DataWidth   = DMA_DATAWIDTH_16BIT;
                stcDmaConfig.u16BlockSize   = pstcInst->u16DmaBufferSize;
                stcDmaConfig.u16TransCount  = 0;  /* 无限传输 */
                stcDmaConfig.u32SrcAddrInc  = DMA_SRC_ADDR_FIX;
                stcDmaConfig.u32DestAddrInc = DMA_DEST_ADDR_INC;
                stcDmaConfig.u8EnableInt    = 1;
                stcDmaConfig.u8IntPriority  = DMA_DEFAULT_INT_PRIO;
                stcDmaConfig.pfnCallback    = NULL;  /* DMA 完成回调由 Dma 层管理 */
                
                /* 创建 DMA 实例 */
                uint8_t u8DmaId = Dma_Create(&stcDmaConfig);
                if (u8DmaId != 0xFF) {
                    s_a8AdcIdToDmaId[i] = (int8_t)u8DmaId;
                    ADC_Adp_DEBUG("ADC ID%d -> DMA ID%d (DMA1 CH%d)\r\n",
                           i, u8DmaId, pstcInst->u8DmaChannel);
                } else {
                    ADC_Adp_DEBUG("Failed to create DMA for ADC ID%d (CH%d)\r\n",
                           i, pstcInst->u8Channel);
                }
            }
        }
        
        /* 初始化所有 DMA 通道 */
        Dma_Init();
        
        /* 启动所有 DMA 通道 */
        Dma_StartAll();
    }
    
    /* 3. 配置定时器触发 */
    Timer0_Config(1000);
    
    /* 4. 配置 ADC 硬件触发 */
    Adc_HardTriggerConfig();
    
    /* 5. 配置 ADC 中断 */
    Adc_IrqConfig();
    
    s_bAdcInitialized = true;
    ADC_Adp_DEBUG("ADC driver initialized with %d instance(s)\r\n", s_u8AdcInstanceCount);
}

/**
 * @brief  Deinitialize ADC driver
 */
void Adc_DeInit(void)
{
    Adc_Stop();
    
    /* 反初始化 DMA */
    if (s_bHasDmaInstance) {
        Dma_DeInit();
    }
    
    s_u8AdcInstanceCount = 0;
    s_bHasDmaInstance = false;
    s_bAdcInitialized = false;
    ADC_Adp_DEBUG("ADC driver deinitialized\r\n");
}

/**
 * @brief  Start ADC conversions
 */
void Adc_Start(void)
{
    Adc_HardTriggerStart();
}

/**
 * @brief  Stop ADC conversions
 */
void Adc_Stop(void)
{
    TMR0_Stop(TMR0_UNIT, TMR0_CH);
    ADC_Adp_DEBUG("ADC stopped\r\n");
}

/**
 * @brief  Process ADC data - call this in main loop
 * @param  u32PrintIntervalMs 打印间隔(ms)，0 表示不打印
 */
void Adc_ProcessData(uint32_t u32PrintIntervalMs)
{
    static uint32_t s_u32LastPrintTick = 0;
    uint32_t u32CurrentTick = (uint32_t)tickTimer_GetCount();
    bool bPrintNow = false;
    
    if ((u32PrintIntervalMs > 0) && ((u32CurrentTick - s_u32LastPrintTick) >= u32PrintIntervalMs)) {
        bPrintNow = true;
        s_u32LastPrintTick = u32CurrentTick;
    }
    
    /* 清除中断模式实例的数据更新标志 */
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        if (s_astcAdcInstances[i].u8ValueUpdated != 0U) {
            s_astcAdcInstances[i].u8ValueUpdated = 0U;
        }
    }
    
    /* 打印调试信息 */
    if (bPrintNow) {
        for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
            stc_adc_instance_t *pstcInst = &s_astcAdcInstances[i];
            
            if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
                if(ADC_RTT == 1) {
                    ADC_Adp_DEBUG("ADC ID%d CH%d [P%c%d] samples=%lu, Latest=%4u ADC (%3u mV)\r\n",
                           pstcInst->u8Id, pstcInst->u8Channel,
                           Adc_GetPortLetter(pstcInst->u8Port), 
                           Adc_GetPinNumber(pstcInst->u8Pin),
                           pstcInst->u32SampleCount,
                           pstcInst->u16LatestValue, 
                           Adc_CalcVoltage(pstcInst->u16LatestValue));
                }
            }
            else if (pstcInst->enMode == ADC_MODE_DMA) {
                /* 通过 Dma 层获取数据 */
                int8_t s8DmaId = s_a8AdcIdToDmaId[i];
                if (s8DmaId >= 0) {
                    uint16_t u16Latest = Dma_GetLatestValue((uint8_t)s8DmaId);
                    uint16_t u16Avg = Dma_GetAverageValue((uint8_t)s8DmaId);
                    uint32_t u32Count = Dma_GetTransferCount((uint8_t)s8DmaId);
                    
                    if(ADC_RTT == 1) {
                        ADC_Adp_DEBUG("ADC ID%d CH%d [P%c%d] samples=%lu, Latest=%4u ADC (%3u mV), Avg=%4u ADC (%3u mV)\r\n",
                               pstcInst->u8Id, pstcInst->u8Channel,
                               Adc_GetPortLetter(pstcInst->u8Port),
                               Adc_GetPinNumber(pstcInst->u8Pin),
                               u32Count,
                               u16Latest, Adc_CalcVoltage(u16Latest),
                               u16Avg, Adc_CalcVoltage(u16Avg));
                    }
                }
            }
        }
    }
}

/**
 * @brief  Get latest ADC value for an ADC instance
 * @param  u8AdcId ADC 实例 ID
 * @return ADC 原始值
 */
uint16_t Adc_GetLatestValue(uint8_t u8AdcId)
{
    if (u8AdcId >= s_u8AdcInstanceCount) {
        return 0;
    }
    
    stc_adc_instance_t *pstcInst = &s_astcAdcInstances[u8AdcId];
    
    if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
        return pstcInst->u16LatestValue;
    } else if (pstcInst->enMode == ADC_MODE_DMA) {
        int8_t s8DmaId = s_a8AdcIdToDmaId[u8AdcId];
        if (s8DmaId >= 0) {
            return Dma_GetLatestValue((uint8_t)s8DmaId);
        }
    }
    return 0;
}

/**
 * @brief  Get average ADC value for an ADC instance
 * @param  u8AdcId ADC 实例 ID
 * @return ADC 平均值（DMA 模式下为缓冲区均值，中断模式下为最新值）
 */
uint16_t Adc_GetAverageValue(uint8_t u8AdcId)
{
    if (u8AdcId >= s_u8AdcInstanceCount) {
        return 0;
    }
    
    stc_adc_instance_t *pstcInst = &s_astcAdcInstances[u8AdcId];
    
    if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
        return pstcInst->u16LatestValue;
    } else if (pstcInst->enMode == ADC_MODE_DMA) {
        int8_t s8DmaId = s_a8AdcIdToDmaId[u8AdcId];
        if (s8DmaId >= 0) {
            return Dma_GetAverageValue((uint8_t)s8DmaId);
        }
    }
    return 0;
}

/**
 * @brief  Get sample count for an ADC instance
 * @param  u8AdcId ADC 实例 ID
 * @return 采样次数
 */
uint32_t Adc_GetSampleCount(uint8_t u8AdcId)
{
    if (u8AdcId >= s_u8AdcInstanceCount) {
        return 0;
    }
    
    stc_adc_instance_t *pstcInst = &s_astcAdcInstances[u8AdcId];
    
    if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
        return pstcInst->u32SampleCount;
    } else if (pstcInst->enMode == ADC_MODE_DMA) {
        int8_t s8DmaId = s_a8AdcIdToDmaId[u8AdcId];
        if (s8DmaId >= 0) {
            return Dma_GetTransferCount((uint8_t)s8DmaId);
        }
    }
    return 0;
}

/**
 * @brief  Find ADC instance ID by channel number
 * @param  u8Channel ADC 通道号
 * @return ADC 实例 ID，未找到返回 -1
 */
int8_t Adc_FindIdByChannel(uint8_t u8Channel)
{
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        if (s_astcAdcInstances[i].u8Channel == u8Channel) {
            return (int8_t)i;
        }
    }
    return -1;
}

/**
 * @brief  Re-enable ADC interrupt (for adding interrupt-mode channels after DMA channels)
 * @note   This function is called when a new interrupt-mode ADC instance is created
 *         after Adc_Init() has already been called. It ensures the ADC EOCA interrupt
 *         is properly enabled so that interrupt-mode channels can receive data.
 */
void Adc_EnableInterrupt(void)
{
    stc_irq_signin_config_t stcIrq;

    /* Register interrupt callback */
    stcIrq.enIntSrc    = ADC_SEQA_INT_SRC;
    stcIrq.enIRQn      = ADC_SEQA_INT_IRQn;
    stcIrq.pfnCallback = &ADC1_SeqA_IrqCallback;
    (void)INTC_IrqSignIn(&stcIrq);

    /* Configure NVIC */
    NVIC_ClearPendingIRQ(stcIrq.enIRQn);
    NVIC_SetPriority(stcIrq.enIRQn, ADC_SEQA_INT_PRIO);
    NVIC_EnableIRQ(stcIrq.enIRQn);

    /* Enable ADC EOCA interrupt */
    ADC_IntCmd(ADC_UNIT, ADC_INT_EOCA, ENABLE);

    ADC_Adp_DEBUG("ADC interrupt re-enabled (EOCA)\r\n");
}

#ifdef DEBUG
void Adc_PrintDebugInfo(void)
{
    ADC_Adp_DEBUG("=== ADC Driver Debug Info ===\r\n");
    ADC_Adp_DEBUG("Initialized: %s\r\n", s_bAdcInitialized ? "Yes" : "No");
    ADC_Adp_DEBUG("Instance count: %d\r\n", s_u8AdcInstanceCount);
    
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        stc_adc_instance_t *pstcInst = &s_astcAdcInstances[i];
        if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
            ADC_Adp_DEBUG("  ID%d: CH%d, Interrupt mode, Pin=P%c%d, samples=%lu\r\n",
                   pstcInst->u8Id, pstcInst->u8Channel,
                   Adc_GetPortLetter(pstcInst->u8Port),
                   Adc_GetPinNumber(pstcInst->u8Pin),
                   pstcInst->u32SampleCount);
        } else {
            int8_t s8DmaId = s_a8AdcIdToDmaId[i];
            ADC_Adp_DEBUG("  ID%d: CH%d, DMA mode (DMA ID=%d), Pin=P%c%d, samples=%lu\r\n",
                   pstcInst->u8Id, pstcInst->u8Channel,
                   s8DmaId,
                   Adc_GetPortLetter(pstcInst->u8Port),
                   Adc_GetPinNumber(pstcInst->u8Pin),
                   pstcInst->u32SampleCount);
        }
    }
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
