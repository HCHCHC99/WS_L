/**
 *******************************************************************************
 * @file  Bemf.c
 * @brief BEMF detection - DMA-based 4-channel ADC sampling
 *        TMR4_3 PWM 同步触发，4通道 DMA 并行搬运，独立 buffer 存储
 *******************************************************************************
 */

#include "Bemf.h"
#include "Dma.h"
#include "Aos.h"
#include "rtt_log.h"
#include "hc32_ll_tmr4.h"
#include <string.h>

/* Current commutation step from Hall sensor ISR (extern, defined in hall_sensor_3ch.c) */
extern volatile uint8_t g_scope_step;

/*******************************************************************************
 * Global variables for JScope monitor (accessible via debug probe)
 ******************************************************************************/

/* 原始 ADC 值 (0-4095, 12bit) */
volatile uint16_t g_bemf_m_raw  = 0;    /* 中性点 (PA0/CH0) */
volatile uint16_t g_bemf_u_raw  = 0;    /* U相 (PA1/CH1) */
volatile uint16_t g_bemf_v_raw  = 0;    /* V相 (PA2/CH2) */
volatile uint16_t g_bemf_w_raw  = 0;    /* W相 (PA3/CH3) */

/* 反电动势电压 (mV, 相对中性点, 带符号) */
volatile int16_t  g_bemf_u_mv   = 0;    /* U相 BEMF */
volatile int16_t  g_bemf_v_mv   = 0;    /* V相 BEMF */
volatile int16_t  g_bemf_w_mv   = 0;    /* W相 BEMF */

/* 中性点电压 (mV) */
volatile uint16_t g_bemf_m_mv   = 0;

/* 累计采样次数 */
volatile uint32_t g_bemf_sample_cnt = 0;

/* BEMF 模块运行状态 (0=未初始化, 1=运行中) */
volatile uint8_t  g_bemf_running = 0;

/* BEMF 波形数据: 当前浮空相 vs 中性点的 ADC 差值 (signed raw)
 * 由 Bemf_UpdateWaveData() 在主循环中更新
 * JScope / VOFA+ 直接监控此变量即可得到梯形波 */
volatile int16_t  g_bemf_wave_data = 0;

/*******************************************************************************
 * Local variables ('static')
 ******************************************************************************/

/* DMA 实例 ID (由 Dma_Create 返回) */
static uint8_t s_au8DmaId[4] = { 0xFF, 0xFF, 0xFF, 0xFF };

/* 最新 BEMF 数据 */
static stc_bemf_data_t s_stcBemfData;

/* 初始化标志 */
static bool s_bBemfInitialized = false;

/* 用户回调 */
static bemf_callback_t s_pfnUserCallback = NULL;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

static void Bemf_SetPinAnalogMode(uint8_t u8Port, uint8_t u8Pin);
static void Bemf_AdcConfig(void);
static void Bemf_DmaConfig(void);
static void Bemf_TriggerConfig(void);
static void Bemf_DataCallback(void);

static char Bemf_GetPortLetter(uint8_t u8Port);
static uint8_t Bemf_GetPinNumber(uint16_t u16Pin);

/*******************************************************************************
 * 辅助函数
 ******************************************************************************/

/**
 * @brief  Get pin number from GPIO pin macro
 */
static uint8_t Bemf_GetPinNumber(uint16_t u16Pin)
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
static char Bemf_GetPortLetter(uint8_t u8Port)
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
 * @brief  Set a single GPIO pin to analog mode
 */
static void Bemf_SetPinAnalogMode(uint8_t u8Port, uint8_t u8Pin)
{
    stc_gpio_init_t stcGpioInit;
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    (void)GPIO_Init(u8Port, u8Pin, &stcGpioInit);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
}

/**
 * @brief  计算电压值 (mV)
 */
static uint16_t Bemf_CalcVoltage(uint16_t u16AdcValue)
{
    return (uint16_t)((((float32_t)(u16AdcValue) * ADC_VREF) / ((float32_t)ADC_ACCURACY)) * 1000.F);
}

/*******************************************************************************
 * ADC 配置
 ******************************************************************************/

/**
 * @brief  Configure ADC1 for BEMF 4-channel scan
 *         SEQ_A single-shot scan: CH0, CH1, CH2, CH3
 */
static void Bemf_AdcConfig(void)
{
    stc_adc_init_t stcAdcInit;

    /* 1. Enable ADC1 peripheral clock */
    FCG_Fcg3PeriphClockCmd(BEMF_ADC_PERIPH_CLK, ENABLE);

    /* 2. ADC base configuration: SEQ_A + SEQ_B both single-shot (SEQ_B used by current sensing) */
    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode = ADC_MD_SEQA_SEQB_SINGLESHOT;

    /* 3. Initialize ADC1 */
    (void)ADC_Init(BEMF_ADC_UNIT, &stcAdcInit);

    /* 4. Configure pins and enable channels */
    struct {
        uint8_t u8Port;
        uint8_t u8Pin;
        uint8_t u8Channel;
        const char *pszName;
    } astcChannels[4] = {
        { BEMF_M_PORT, BEMF_M_PIN, BEMF_CH_M, "M_BEMF" },
        { BEMF_U_PORT, BEMF_U_PIN, BEMF_CH_U, "U_BEMF" },
        { BEMF_V_PORT, BEMF_V_PIN, BEMF_CH_V, "V_BEMF" },
        { BEMF_W_PORT, BEMF_W_PIN, BEMF_CH_W, "W_BEMF" },
    };

    for (uint8_t i = 0; i < 4; i++) {
        /* Set pin to analog mode */
        Bemf_SetPinAnalogMode(astcChannels[i].u8Port, astcChannels[i].u8Pin);

        /* Enable ADC channel in SEQ_A */
        ADC_ChCmd(BEMF_ADC_UNIT, ADC_SEQ_A, astcChannels[i].u8Channel, ENABLE);

        BEMF_Adp_DEBUG("BEMF %s configured: P%c%d (ADC1_CH%d)\r\n",
               astcChannels[i].pszName,
               Bemf_GetPortLetter(astcChannels[i].u8Port),
               Bemf_GetPinNumber(astcChannels[i].u8Pin),
               astcChannels[i].u8Channel);
    }

    BEMF_Adp_DEBUG("ADC1 SEQ_A configured for 4-channel BEMF scan\r\n");
}

/*******************************************************************************
 * DMA 配置
 ******************************************************************************/

/**
 * @brief  DMA block complete callback (DMA1 CH0 触发)
 *         当 4 个 DMA 通道同时完成一个 block 传输时,
 *         从各自的 buffer 读取最新值并更新 BEMF 数据结构
 */
static void Bemf_DataCallback(void)
{
    /* Ensure all 4 DMA channels are valid */
    if (s_au8DmaId[0] == 0xFF) {
        return;
    }

    /* Read averaged values from each DMA buffer (8-tap moving average,
     * -3dB @ 6.25kHz, suppresses PWM switching noise at 100kHz) */
    uint16_t u16M = Dma_GetAverageValue(s_au8DmaId[0]);
    uint16_t u16U = Dma_GetAverageValue(s_au8DmaId[1]);
    uint16_t u16V = Dma_GetAverageValue(s_au8DmaId[2]);
    uint16_t u16W = Dma_GetAverageValue(s_au8DmaId[3]);

    /* Update internal data structure */
    s_stcBemfData.stcLatest.u16M = u16M;
    s_stcBemfData.stcLatest.u16U = u16U;
    s_stcBemfData.stcLatest.u16V = u16V;
    s_stcBemfData.stcLatest.u16W = u16W;

    /* Calculate BEMF voltages (signed difference vs neutral) */
    s_stcBemfData.i16BemfU = (int16_t)u16U - (int16_t)u16M;
    s_stcBemfData.i16BemfV = (int16_t)u16V - (int16_t)u16M;
    s_stcBemfData.i16BemfW = (int16_t)u16W - (int16_t)u16M;

    s_stcBemfData.u32SampleCount++;
    s_stcBemfData.u8NewData = 1U;

    /* ---- Update JScope global variables ---- */
    g_bemf_m_raw = u16M;
    g_bemf_u_raw = u16U;
    g_bemf_v_raw = u16V;
    g_bemf_w_raw = u16W;

    g_bemf_u_mv = (int16_t)(((int32_t)s_stcBemfData.i16BemfU * 3300) / 4096);
    g_bemf_v_mv = (int16_t)(((int32_t)s_stcBemfData.i16BemfV * 3300) / 4096);
    g_bemf_w_mv = (int16_t)(((int32_t)s_stcBemfData.i16BemfW * 3300) / 4096);
    g_bemf_m_mv = (uint16_t)(((uint32_t)u16M * 3300) / 4096);

    g_bemf_sample_cnt = s_stcBemfData.u32SampleCount;

    /* ---- Update BEMF waveform data: floating phase - neutral (signed raw ADC) ---- */
    if (g_scope_step < 6U) {
        switch (Bemf_GetFloatingChannel(g_scope_step)) {
            case 1: g_bemf_wave_data = (int16_t)u16U - (int16_t)u16M; break;
            case 2: g_bemf_wave_data = (int16_t)u16V - (int16_t)u16M; break;
            case 3: g_bemf_wave_data = (int16_t)u16W - (int16_t)u16M; break;
            default: g_bemf_wave_data = 0; break;
        }
    } else {
        g_bemf_wave_data = 0;
    }

    /* Invoke user callback if registered */
    if (s_pfnUserCallback != NULL) {
        s_pfnUserCallback(&s_stcBemfData);
    }
}

/**
 * @brief  Create 4 DMA channels for BEMF data acquisition
 *         CH0 -> ADC1 DR0 (M_BEMF)
 *         CH1 -> ADC1 DR1 (U_BEMF)
 *         CH2 -> ADC1 DR2 (V_BEMF)
 *         CH3 -> ADC1 DR3 (W_BEMF)
 */
static void Bemf_DmaConfig(void)
{
    stc_dma_config_t stcDmaConfig;

    /* Channel descriptor table */
    struct {
        uint8_t  u8DmaCh;       /* DMA 通道号 */
        uint8_t  u8AdcCh;       /* ADC 通道号 (用于计算 DR 偏移) */
        const char *pszName;
    } astcDmaChannels[4] = {
        { BEMF_DMA_CH_M, BEMF_CH_M, "M_BEMF" },
        { BEMF_DMA_CH_U, BEMF_CH_U, "U_BEMF" },
        { BEMF_DMA_CH_V, BEMF_CH_V, "V_BEMF" },
        { BEMF_DMA_CH_W, BEMF_CH_W, "W_BEMF" },
    };

    for (uint8_t i = 0; i < 4; i++) {
        memset(&stcDmaConfig, 0, sizeof(stc_dma_config_t));

        stcDmaConfig.u8DmaUnit      = BEMF_DMA_UNIT;
        stcDmaConfig.u8Channel      = astcDmaChannels[i].u8DmaCh;
        stcDmaConfig.enDir          = DMA_DIR_PERIPH_TO_MEM;
        stcDmaConfig.enTransMode    = DMA_TRANS_MODE_REPEAT;

        /* Source: ADC1 DRn register (DR0 + channel*2, 16-bit per channel) */
        stcDmaConfig.u32SrcAddr     = (uint32_t)((uint32_t)&BEMF_ADC_UNIT->DR0 +
                                                 (astcDmaChannels[i].u8AdcCh * 2U));
        stcDmaConfig.u32DestAddr    = 0;  /* Dma_Init 会用 malloc 分配的 buffer 地址替换 */
        stcDmaConfig.u32DataWidth   = BEMF_DMA_DATA_WIDTH;
        stcDmaConfig.u16BlockSize   = BEMF_DMA_BUFFER_SIZE;
        stcDmaConfig.u16TransCount  = 0;  /* 无限传输 (repeat mode) */
        stcDmaConfig.u32SrcAddrInc  = DMA_SRC_ADDR_FIX;    /* 外设地址固定 */
        stcDmaConfig.u32DestAddrInc = DMA_DEST_ADDR_INC;   /* 内存地址递增 */
        stcDmaConfig.u8EnableInt    = 1;                    /* 使能 BTC 中断 */
        stcDmaConfig.u8IntPriority  = BEMF_DMA_INT_PRIO;

        /* 只在 CH0 上注册回调 (4 个通道同时完成 block, CH0 回调统一处理) */
        stcDmaConfig.pfnCallback    = (i == 0) ? Bemf_DataCallback : NULL;

        /* 创建 DMA 实例 */
        s_au8DmaId[i] = Dma_Create(&stcDmaConfig);
        if (s_au8DmaId[i] != 0xFF) {
            BEMF_Adp_DEBUG("DMA1 CH%d created (ID=%d) for %s, src=ADC1_DR%d\r\n",
                   astcDmaChannels[i].u8DmaCh, s_au8DmaId[i],
                   astcDmaChannels[i].pszName, astcDmaChannels[i].u8AdcCh);
        } else {
            BEMF_Adp_DEBUG("ERROR: Failed to create DMA1 CH%d for %s!\r\n",
                   astcDmaChannels[i].u8DmaCh, astcDmaChannels[i].pszName);
        }
    }
}

/*******************************************************************************
 * TMR4 EVT 配置 (软件比较事件 - SCMP0)
 ******************************************************************************/

/**
 * @brief  Configure TMR4_3 EVT to fire SCMP0 at every PWM period peak
 * @note   TMR4_3 is center-aligned (triangle wave). Counter goes 0 -> period -> 0.
 *         峰值在 counter == period 时。SCMP0 事件经 AOS 路由到 ADC1 触发采样。
 *
 *         EVT channel 0 (UH) 的 PWM 功能仍正常工作，EVT 子模块独立运行。
 */
static void Bemf_Tmr4EvtConfig(void)
{
    stc_tmr4_evt_init_t stcTmr4Evt;

    /* Read current PWM period from TMR4 hardware register */
    uint16_t u16Period = TMR4_GetPeriodValue(CM_TMR4_3);
    if (u16Period == 0U) {
        u16Period = 1U;
    }

    /* Initialize EVT on channel 0 (映射到 SCMP0 事件输出)
     * 三角波计数器: 0 -> u16Period -> 0, peak = counter == u16Period
     * compareValue = u16Period 保证每个 peak 都触发一次 SCMP0 事件 */
    (void)TMR4_EVT_StructInit(&stcTmr4Evt);
    stcTmr4Evt.u16Mode         = TMR4_EVT_MD_CMP;               /* 比较模式 */
    stcTmr4Evt.u16CompareValue = u16Period;                     /* 匹配 peak 时的计数值 */
    stcTmr4Evt.u16MatchCond    = TMR4_EVT_MATCH_CNT_PEAK;       /* 计数器达到peak时匹配 */
    /* u16OutputEvent 保持 StructInit 默认值 TMR4_EVT_OUTPUT_EVT0 (0) = SCMP0 */

    (void)TMR4_EVT_Init(CM_TMR4_3, TMR4_EVT_CH_UH, &stcTmr4Evt);

    BEMF_Adp_DEBUG("TMR4_3 EVT configured: CH=UH, mode=CMP, cmpVal=%u, match=PEAK, output=SCMP0 (EVT0)\r\n",
           u16Period);
}

/*******************************************************************************
 * ADC 触发配置
 ******************************************************************************/

/**
 * @brief  Configure ADC hardware trigger (TMR4_3 SCMP0 via AOS routing)
 *         AOS_InitForBemf() 已经建立了 TMR4_3_SCMP0 -> ADC1 的路由
 */
static void Bemf_TriggerConfig(void)
{
    /* ADC1 SEQ_A 使用硬件触发 (EVT0, 由 AOS 路由到 TMR4_3 SCMP0) */
    ADC_TriggerConfig(BEMF_ADC_UNIT, ADC_SEQ_A, BEMF_ADC_SEQA_HARDTRIG);
    ADC_TriggerCmd(BEMF_ADC_UNIT, ADC_SEQ_A, ENABLE);

    BEMF_Adp_DEBUG("ADC1 SEQ_A hardware trigger enabled (via AOS: TMR4_3_SCMP0)\r\n");
}

/*******************************************************************************
 * API 实现 - 生命周期
 ******************************************************************************/

/**
 * @brief  Initialize BEMF detection module
 * @note   调用前需确保:
 *         1. TMR4 PWM 已配置并启动 (TMR4_PWM_Config + TMR4_PWM_StartOutput)
 *         2. 在 Hardware_Init() 之后调用, 或在调用前确保时钟系统已初始化
 *
 *         初始化顺序:
 *         1. 配置 AOS 路由 (TMR4_3_UDF -> ADC1, ADC1_EOCA -> DMA1_CH0~3)
 *         2. 配置 ADC1 (引脚 + SEQ_A 扫描)
 *         3. 创建 DMA 通道
 *         4. 初始化 DMA (分配 buffer + 硬件配置)
 *         5. 配置 ADC 硬件触发
 *         6. 启动所有 DMA 通道
 */
void Bemf_Init(void)
{
    if (s_bBemfInitialized) {
        BEMF_Adp_DEBUG("BEMF already initialized\r\n");
        return;
    }

    /* Reset data structure */
    memset(&s_stcBemfData, 0, sizeof(s_stcBemfData));
    for (uint8_t i = 0; i < 4; i++) {
        s_au8DmaId[i] = 0xFF;
    }

    /* 1. Configure AOS routing (TMR4_3 -> ADC1 -> DMA1) */
    AOS_InitForBemf();

    /* 2. Configure ADC1: pins + SEQ_A channels */
    Bemf_AdcConfig();

    /* 3. Create 4 DMA channel instances */
    Bemf_DmaConfig();

    /* 4. Initialize DMA (allocate buffers, configure hardware, setup interrupts) */
    Dma_Init();

    /* 5. Configure TMR4 EVT (SCMP0 fires at PWM peak -> ADC trigger) */
    Bemf_Tmr4EvtConfig();

    /* 6. Configure ADC hardware trigger */
    Bemf_TriggerConfig();

    /* 7. Start all DMA channels (waiting for ADC EOCA trigger) */
    Dma_StartAll();

    s_bBemfInitialized = true;
    g_bemf_running = 1;
    BEMF_Adp_DEBUG("BEMF initialized: 4 channels, DMA buffer=%d, trigger=TMR4_3_SCMP0@PEAK\r\n",
           BEMF_DMA_BUFFER_SIZE);
}

/**
 * @brief  Deinitialize BEMF module
 */
void Bemf_DeInit(void)
{
    s_bBemfInitialized = false;
    s_pfnUserCallback = NULL;
    g_bemf_running = 0;

    /* Stop and deinit DMA */
    Dma_DeInit();

    /* Clear data */
    memset(&s_stcBemfData, 0, sizeof(s_stcBemfData));
    for (uint8_t i = 0; i < 4; i++) {
        s_au8DmaId[i] = 0xFF;
    }

    BEMF_Adp_DEBUG("BEMF deinitialized\r\n");
}

/*******************************************************************************
 * API 实现 - 数据获取 (观察者模式)
 ******************************************************************************/

/**
 * @brief  Get latest BEMF data (4-channel synchronized)
 * @param  pData  输出数据结构指针
 * @note   读取后自动清除 u8NewData 标志
 */
void Bemf_GetData(stc_bemf_data_t *pData)
{
    if (pData == NULL) {
        return;
    }

    /* Atomic copy (DMA ISR may update simultaneously) */
    memcpy(pData, &s_stcBemfData, sizeof(stc_bemf_data_t));

    /* Clear new data flag */
    s_stcBemfData.u8NewData = 0U;
}

/**
 * @brief  Get raw ADC value for a single channel
 * @param  u8Channel  0=M, 1=U, 2=V, 3=W
 * @return Raw ADC value (0-4095), 0 if invalid
 */
uint16_t Bemf_GetRawValue(uint8_t u8Channel)
{
    if (u8Channel >= 4 || s_au8DmaId[u8Channel] == 0xFF) {
        return 0;
    }
    return Dma_GetLatestValue(s_au8DmaId[u8Channel]);
}

/**
 * @brief  Get BEMF voltage relative to neutral point
 * @param  u8Phase  0=U, 1=V, 2=W
 * @return Signed voltage in mV (relative to neutral)
 */
int16_t Bemf_GetBemfVoltage(uint8_t u8Phase)
{
    if (u8Phase >= 3) {
        return 0;
    }

    /* Get latest raw values */
    uint16_t u16Neutral = Bemf_GetRawValue(0);
    uint16_t u16Phase   = Bemf_GetRawValue(u8Phase + 1);  /* +1: skip M, get U/V/W */

    /* Calculate signed difference and convert to mV */
    int16_t i16Diff = (int16_t)u16Phase - (int16_t)u16Neutral;
    return (int16_t)(((int32_t)i16Diff * (int32_t)ADC_VREF * 1000) / (int32_t)ADC_ACCURACY);
}

/*******************************************************************************
 * API 实现 - 浮空相选择
 ******************************************************************************/

/**
 * @brief  Get the floating phase ADC channel for a given commutation step
 * @param  u8Step  Commutation step (0–5), per dev_commutation.c
 * @return ADC channel index (0=M, 1=U, 2=V, 3=W), returns 0 if invalid
 * @note   Six-step mapping (matches dev_commutation.c state table):
 *         Step 0 (UH+VL): W floating → channel 3
 *         Step 1 (UH+WL): V floating → channel 2
 *         Step 2 (VH+WL): U floating → channel 1
 *         Step 3 (VH+UL): W floating → channel 3
 *         Step 4 (WH+UL): V floating → channel 2
 *         Step 5 (WH+VL): U floating → channel 1
 */
uint8_t Bemf_GetFloatingChannel(uint8_t u8Step)
{
    static const uint8_t s_au8Map[6] = {
        3,  /* Step 0 (UH+VL): W floating */
        2,  /* Step 1 (UH+WL): V floating */
        1,  /* Step 2 (VH+WL): U floating */
        3,  /* Step 3 (VH+UL): W floating */
        2,  /* Step 4 (WH+UL): V floating */
        1,  /* Step 5 (WH+VL): U floating */
    };
    return (u8Step < 6U) ? s_au8Map[u8Step] : 0U;
}

/**
 * @brief  Get the raw ADC value of the floating phase for a given step
 * @param  u8Step  Commutation step (0–5)
 * @return Raw ADC value (0–4095), 0 if invalid
 */
uint16_t Bemf_GetFloatingPhaseRaw(uint8_t u8Step)
{
    uint8_t u8Ch = Bemf_GetFloatingChannel(u8Step);
    if (u8Ch == 0U) {
        return 0;
    }
    return Dma_GetAverageValue(s_au8DmaId[u8Ch]);
}

/**
 * @brief  Get BEMF voltage of the floating phase for a given step (mV, relative to neutral)
 * @param  u8Step  Commutation step (0–5)
 * @return Signed BEMF voltage in mV
 */
int16_t Bemf_GetFloatingPhaseBemf(uint8_t u8Step)
{
    uint8_t u8Ch = Bemf_GetFloatingChannel(u8Step);
    if (u8Ch == 0U || u8Ch >= 4U) {
        return 0;
    }

    uint16_t u16Neutral = Dma_GetAverageValue(s_au8DmaId[0]);
    uint16_t u16Phase   = Dma_GetAverageValue(s_au8DmaId[u8Ch]);

    int16_t i16Diff = (int16_t)u16Phase - (int16_t)u16Neutral;
    return (int16_t)(((int32_t)i16Diff * (int32_t)ADC_VREF * 1000) / (int32_t)ADC_ACCURACY);
}

/**
 * @brief  Update g_bemf_wave_data from the floating phase for the given step
 * @param  u8Step  Current commutation step (0–5, from Hall sensor)
 * @note   Call from main loop. Uses DMA buffer average (8-tap MA filter).
 *         g_bemf_wave_data = (floating_phase_raw - neutral_raw) as signed ADC diff.
 *         JScope / VOFA+ directly plot this variable for the BEMF trapezoid.
 */
void Bemf_UpdateWaveData(uint8_t u8Step)
{
    if (u8Step >= 6U || s_au8DmaId[0] == 0xFF) {
        g_bemf_wave_data = 0;
        return;
    }

    uint8_t u8FloatingCh = Bemf_GetFloatingChannel(u8Step);
    if (u8FloatingCh == 0U || u8FloatingCh >= 4U) {
        g_bemf_wave_data = 0;
        return;
    }

    uint16_t u16Neutral = Dma_GetAverageValue(s_au8DmaId[0]);
    uint16_t u16Phase   = Dma_GetAverageValue(s_au8DmaId[u8FloatingCh]);

    g_bemf_wave_data = (int16_t)u16Phase - (int16_t)u16Neutral;
}

/*******************************************************************************
 * API 实现 - 回调
 ******************************************************************************/

/**
 * @brief  Register callback for new BEMF data notification
 * @param  pfnCallback  回调函数 (NULL 取消注册)
 * @note   回调在 DMA ISR 上下文中执行，请保持简短
 */
void Bemf_RegisterCallback(bemf_callback_t pfnCallback)
{
    s_pfnUserCallback = pfnCallback;
    BEMF_Adp_DEBUG("BEMF callback %s\r\n", (pfnCallback != NULL) ? "registered" : "unregistered");
}

/*******************************************************************************
 * API 实现 - 调试
 ******************************************************************************/

#ifdef DEBUG
void Bemf_PrintDebugInfo(void)
{
    BEMF_Adp_DEBUG("=== BEMF Debug Info ===\r\n");
    BEMF_Adp_DEBUG("Initialized: %s\r\n", s_bBemfInitialized ? "Yes" : "No");
    BEMF_Adp_DEBUG("Samples: %lu\r\n", s_stcBemfData.u32SampleCount);

    BEMF_Adp_DEBUG("Latest raw: M=%u, U=%u, V=%u, W=%u\r\n",
           s_stcBemfData.stcLatest.u16M,
           s_stcBemfData.stcLatest.u16U,
           s_stcBemfData.stcLatest.u16V,
           s_stcBemfData.stcLatest.u16W);

    BEMF_Adp_DEBUG("BEMF (raw diff): U=%d, V=%d, W=%d\r\n",
           s_stcBemfData.i16BemfU,
           s_stcBemfData.i16BemfV,
           s_stcBemfData.i16BemfW);

    BEMF_Adp_DEBUG("BEMF (mV): U=%d, V=%d, W=%d\r\n",
           Bemf_GetBemfVoltage(0),
           Bemf_GetBemfVoltage(1),
           Bemf_GetBemfVoltage(2));

    BEMF_Adp_DEBUG("DMA IDs: M=%d, U=%d, V=%d, W=%d\r\n",
           s_au8DmaId[0], s_au8DmaId[1], s_au8DmaId[2], s_au8DmaId[3]);

    BEMF_Adp_DEBUG("Callback: %s\r\n", (s_pfnUserCallback != NULL) ? "Yes" : "No");
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
