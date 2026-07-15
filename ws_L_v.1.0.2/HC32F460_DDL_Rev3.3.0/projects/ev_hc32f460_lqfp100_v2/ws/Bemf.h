/**
 *******************************************************************************
 * @file  Bemf.h
 * @brief BEMF (Back Electromotive Force) detection for BLDC sensorless control
 *        DMA-based 4-channel ADC sampling synchronized with TMR4 PWM
 *
 *        通道分配:
 *          PA0 = ADC1_CH0 = M_BEMF  (中性点)
 *          PA1 = ADC1_CH1 = U_BEMF  (U相反电动势)
 *          PA2 = ADC1_CH2 = V_BEMF  (V相反电动势)
 *          PA3 = ADC1_CH3 = W_BEMF  (W相反电动势)
 *
 *        数据流:
 *          TMR4_3 UDF (PWM中点) -> AOS -> ADC1_SEQ_A 触发扫描
 *          ADC1 EOCA -> AOS -> DMA1_CH0/1/2/3 各自搬运 DR0~DR3
 *          DMA block complete -> 更新数据 -> 可选回调
 *******************************************************************************
 */

#ifndef __BEMF_H__
#define __BEMF_H__

#include "main.h"
#include "Hardware.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/* 调试开关 (取消注释以启用) */
/* #define DEBUG_BEMF_Adp */          /* Bemf.c 内部函数级调试打印 */
/* #define BEMF_PERIODIC_DBG */      /* main.c 每500ms BEMF 数据打印 */

#ifdef DEBUG_BEMF_Adp
    #define BEMF_Adp_DEBUG(fmt, ...)    MAIN_D("[BEMF_DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define BEMF_Adp_DEBUG(fmt, ...)    ((void)0)
#endif

/* ===== BEMF 通道定义 ===== */
#define BEMF_CH_M                       (ADC_CH0)   /* PA0: 中性点 (M_BEMF) */
#define BEMF_CH_U                       (ADC_CH1)   /* PA1: U相 (U_BEMF) */
#define BEMF_CH_V                       (ADC_CH2)   /* PA2: V相 (V_BEMF) */
#define BEMF_CH_W                       (ADC_CH3)   /* PA3: W相 (W_BEMF) */
#define BEMF_CHANNEL_COUNT              (4U)        /* 总通道数 */

/* ===== BEMF 引脚定义 ===== */
#define BEMF_M_PORT                     (GPIO_PORT_A)
#define BEMF_M_PIN                      (GPIO_PIN_00)
#define BEMF_U_PORT                     (GPIO_PORT_A)
#define BEMF_U_PIN                      (GPIO_PIN_01)
#define BEMF_V_PORT                     (GPIO_PORT_A)
#define BEMF_V_PIN                      (GPIO_PIN_02)
#define BEMF_W_PORT                     (GPIO_PORT_A)
#define BEMF_W_PIN                      (GPIO_PIN_03)

/* ===== ADC 硬件配置 (复用 Adc.h 中定义的硬件宏) ===== */
#define BEMF_ADC_UNIT                   (CM_ADC1)
#define BEMF_ADC_PERIPH_CLK             (FCG3_PERIPH_ADC1)
#define BEMF_ADC_SEQA_HARDTRIG          (ADC_HARDTRIG_EVT0)

/* ===== DMA 配置 ===== */
#define BEMF_DMA_UNIT                   (DMA_UNIT_1)            /* 使用 DMA1 */
#define BEMF_DMA_CH_M                   (0U)                    /* DMA1 CH0 -> M_BEMF */
#define BEMF_DMA_CH_U                   (1U)                    /* DMA1 CH1 -> U_BEMF */
#define BEMF_DMA_CH_V                   (2U)                    /* DMA1 CH2 -> V_BEMF */
#define BEMF_DMA_CH_W                   (3U)                    /* DMA1 CH3 -> W_BEMF */
#define BEMF_DMA_BUFFER_SIZE            (8U)                    /* 每通道缓冲大小 */
#define BEMF_DMA_DATA_WIDTH             (DMA_DATAWIDTH_16BIT)   /* 16位数据宽度 */
#define BEMF_DMA_INT_PRIO               (DDL_IRQ_PRIO_03)       /* DMA 中断优先级 */

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/**
 * @brief  BEMF 单次采样数据 (4通道同步)
 */
typedef struct {
    uint16_t u16M;          /* 中性点电压 (raw ADC) */
    uint16_t u16U;          /* U相电压 (raw ADC) */
    uint16_t u16V;          /* V相电压 (raw ADC) */
    uint16_t u16W;          /* W相电压 (raw ADC) */
} stc_bemf_sample_t;

/**
 * @brief  BEMF 反电动势计算结果
 * @note   i16BemfX = u16PhaseX - u16Neutral (带符号的差值)
 *         正值表示相反电动势高于中性点，负值表示低于中性点
 */
typedef struct {
    stc_bemf_sample_t stcLatest;    /* 最新采样原始值 */
    int16_t  i16BemfU;              /* U相 - 中性点 (signed ADC差值) */
    int16_t  i16BemfV;              /* V相 - 中性点 (signed ADC差值) */
    int16_t  i16BemfW;              /* W相 - 中性点 (signed ADC差值) */
    uint32_t u32SampleCount;        /* 累计采样次数 (每次 = 4通道各1个采样) */
    uint8_t  u8NewData;             /* 新数据标志 (读取后清零) */
} stc_bemf_data_t;

/**
 * @brief  BEMF 数据就绪回调函数类型
 */
typedef void (*bemf_callback_t)(const stc_bemf_data_t *pData);

/*******************************************************************************
 * Global variables for JScope (extern - defined in Bemf.c)
 ******************************************************************************/

/* 原始 ADC 值 (0-4095) */
extern volatile uint16_t g_bemf_m_raw;
extern volatile uint16_t g_bemf_u_raw;
extern volatile uint16_t g_bemf_v_raw;
extern volatile uint16_t g_bemf_w_raw;

/* BEMF 电压 (mV, 相对中性点, 带符号) */
extern volatile int16_t  g_bemf_u_mv;
extern volatile int16_t  g_bemf_v_mv;
extern volatile int16_t  g_bemf_w_mv;

/* 中性点电压 (mV) */
extern volatile uint16_t g_bemf_m_mv;

/* 累计采样计数 */
extern volatile uint32_t g_bemf_sample_cnt;

/* 模块运行状态 (0=停止, 1=运行中) */
extern volatile uint8_t  g_bemf_running;

/* BEMF 波形数据: 当前浮空相 vs 中性点 ADC 差值 (signed raw)
 * JScope / VOFA+ 直接读取此变量即可得到梯形波 */
extern volatile int16_t  g_bemf_wave_data;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/

/* BEMF 初始化和反初始化 */
void Bemf_Init(void);
void Bemf_DeInit(void);

/* BEMF 数据获取 (观察者模式) */
void Bemf_GetData(stc_bemf_data_t *pData);

/* 单通道原始值获取 (0=M, 1=U, 2=V, 3=W) */
uint16_t Bemf_GetRawValue(uint8_t u8Channel);

/* 反电动势电压获取 (mV, 相对中性点) */
int16_t Bemf_GetBemfVoltage(uint8_t u8Phase);  /* 0=U, 1=V, 2=W */

/* 浮空相选择 (六步换相) */
uint8_t  Bemf_GetFloatingChannel(uint8_t u8Step);       /* 返回浮空相 ADC 通道号 (0=M, 1=U, 2=V, 3=W) */
uint16_t Bemf_GetFloatingPhaseRaw(uint8_t u8Step);      /* 返回浮空相原始 ADC 值 */
int16_t  Bemf_GetFloatingPhaseBemf(uint8_t u8Step);      /* 返回浮空相反电动势 (mV) */

/* 更新 BEMF 波形数据 (主循环调用) */
void Bemf_UpdateWaveData(uint8_t u8Step);                /* 根据当前换相步更新 g_bemf_wave_data */

/* 注册数据就绪回调 (NULL 取消注册) */
void Bemf_RegisterCallback(bemf_callback_t pfnCallback);

/* 调试信息打印 */
#ifdef DEBUG
void Bemf_PrintDebugInfo(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __BEMF_H__ */
