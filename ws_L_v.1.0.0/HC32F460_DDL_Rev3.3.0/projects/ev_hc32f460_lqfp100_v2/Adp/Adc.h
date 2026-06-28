/**
 *******************************************************************************
 * @file  Adc.h
 * @brief ADC Driver for HC32F460 - Generic framework with multiple instances
 *******************************************************************************
 */

#ifndef __ADC_H__
#define __ADC_H__

#include "main.h"
#include "Hardware.h"
#include "TickTimer.h"

#define ADC_RTT 1

#ifdef DEBUG_ADC_Adp
    #define ADC_Adp_DEBUG(fmt, ...)    MAIN_D("[ADC_DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define ADC_Adp_DEBUG(fmt, ...)    ((void)0)
#endif

#ifdef ADC_OUTPUT_Adp
    #define ADC_Adp_OUT(fmt, ...)      MAIN_D("[ADC_OUT] " fmt, ##__VA_ARGS__)
#else
    #define ADC_Adp_OUT(fmt, ...)      ((void)0)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/* DMA缓冲区大小定义 */
#define ADC_DMA_BUFFER_SIZE              (8U)  /* 每次触发采集8次 */

/* ADC硬件配置宏 */
#define ADC_UNIT                        (CM_ADC1)
#define ADC_PERIPH_CLK                  (FCG3_PERIPH_ADC1)

/* ===== 序列A触发配置（第一级：TMR0 → ADC）===== */
#define ADC_SEQA_HARDTRIG               (ADC_HARDTRIG_EVT0)        /* 使用事件触发 */
#define ADC_SEQA_AOS_TRIG_SEL           (AOS_ADC1_0)               /* ADC1的事件输入 */
#define ADC_SEQA_TRIG_EVT               (EVT_SRC_TMR0_1_CMP_B)     /* 定时器比较事件 */

/* ===== DMA触发配置（第二级：ADC → DMA）===== */
#define DMA_UNIT                        (CM_DMA1)
#define DMA_PERIPH_CLK                  (FCG0_PERIPH_DMA1)
#define DMA_DATA_WIDTH                  (DMA_DATAWIDTH_16BIT)      /* 16位数据 */
#define DMA_TRANS_CNT                   (0U)                        /* 0:无限传输 */
#define DMA_INT_PRIO                    (DDL_IRQ_PRIO_03)

/* Timer0 for sequence A */
#define TMR0_UNIT                       (CM_TMR0_1)
#define TMR0_CH                         (TMR0_CH_B)
#define TMR0_PERIPH_CLK                 (FCG2_PERIPH_TMR0_1)
#define TMR0_CLK_DIV                    (TMR0_CLK_DIV256)

/* ADC序列A中断配置 */
#define ADC_SEQA_INT_PRIO               (DDL_IRQ_PRIO_04)
#define ADC_SEQA_INT_SRC                (INT_SRC_ADC1_EOCA)
#define ADC_SEQA_INT_IRQn               (INT116_IRQn)

/* ADC参考电压 */
#define ADC_VREF                        (3.3F)
#define ADC_ACCURACY                    (1UL << 12U)
#define ADC_CAL_VOL(adcVal)             (uint16_t)((((float32_t)(adcVal) * ADC_VREF) / ((float32_t)ADC_ACCURACY)) * 1000.F)

/* 最大ADC实例数量 */
#define ADC_MAX_INSTANCES               (8U)

/* 最大ADC通道号 */
#define ADC_MAX_CHANNEL                 (32U)

/* 最大DMA通道数 */
#define ADC_MAX_DMA_CH                  (8U)

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/**
 * @brief  ADC通道处理模式定义
 */
typedef enum {
    ADC_MODE_INTERRUPT = 0,    /* 中断模式 */
    ADC_MODE_DMA = 1,          /* DMA模式 */
} en_adc_mode_t;

/**
 * @brief  ADC引脚映射结构体
 */
typedef struct {
    uint8_t u8Port;      /* GPIO端口 (如 GPIO_PORT_A) */
    uint8_t u8Pin;       /* GPIO引脚 (如 GPIO_PIN_01) */
} stc_adc_pin_t;

/**
 * @brief  ADC通道配置结构体 (用户创建时填写)
 */
typedef struct {
    uint8_t u8Channel;              /* ADC通道号 (如 ADC_CH1, ADC_CH4 等) */
    en_adc_mode_t enMode;           /* 处理模式 */
    stc_adc_pin_t stcPin;           /* 引脚配置 */
    void (*pfnCallback)(uint16_t u16AdcValue);  /* 中断模式回调函数 */
    /* DMA相关配置 - 只有模式为DMA时才有效 */
    struct {
        uint16_t u16BufferSize;       /* DMA缓冲区大小 */
        uint8_t u8DmaChannel;         /* 使用的DMA通道 */
    } stcDmaConfig;
} stc_adc_config_t;


typedef struct {
    uint8_t u8Id;                    /* ADC实例ID (0-7) */
    uint8_t u8Channel;              /* ADC通道号 */
    en_adc_mode_t enMode;           /* 处理模式 */
    uint8_t u8Port;                 /* GPIO端口 */
    uint8_t u8Pin;                  /* GPIO引脚 */
    void (*pfnCallback)(uint16_t u16AdcValue);  /* 中断模式回调函数 */
    /* DMA相关 */
    uint16_t *pu16DmaBuffer;         /* DMA缓冲区指针 */
    uint16_t u16DmaBufferSize;       /* DMA缓冲区大小 */
    uint8_t u8DmaChannel;            /* 使用的DMA通道 */
    /* 运行状态 */
    uint32_t u32SampleCount;         /* 采样计数 */
    uint16_t u16LatestValue;         /* 最新值 (中断模式) */
    uint8_t u8ValueUpdated;          /* 数据更新标志 */
} stc_adc_instance_t;


/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/

/* ADC 实例管理 */
uint8_t Adc_Create(stc_adc_config_t *pstcConfig);
void Adc_Init(void);
void Adc_DeInit(void);

/* ADC 控制函数 */
void Adc_Start(void);
void Adc_Stop(void);
void Adc_ProcessData(uint32_t u32PrintIntervalMs);

/* ADC 数据获取函数 */
uint16_t Adc_GetLatestValue(uint8_t u8AdcId);
uint16_t Adc_GetAverageValue(uint8_t u8AdcId);
uint32_t Adc_GetSampleCount(uint8_t u8AdcId);

/* 根据通道号查找ADC ID */
int8_t Adc_FindIdByChannel(uint8_t u8Channel);

/* 重新使能ADC中断（用于在已有DMA通道后添加中断模式通道） */
void Adc_EnableInterrupt(void);

/* 调试信息 */
#ifdef DEBUG
void Adc_PrintDebugInfo(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
