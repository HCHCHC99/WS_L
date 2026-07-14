/**
 *******************************************************************************
 * @file  I.c
 * @brief Current sensing module — ADC1 SEQ_B interrupt-mode 3-phase sampling
 *        Triggered at PWM peak via TMR4_3 SCMP2 → AOS_ADC1_1 → ADC1_SEQ_B
 *******************************************************************************
 */

#include "I.h"
#include "rtt_log.h"
#include "TickTimer.h"
#include "tmr4_pwm.h"
#include <string.h>

/*******************************************************************************
 * Global variables for JScope monitor
 ******************************************************************************/

/* Raw ADC values (0-4095) */
volatile uint16_t g_i_iu_raw  = 0;
volatile uint16_t g_i_iv_raw  = 0;
volatile uint16_t g_i_iw_raw  = 0;

/* Current in mA (signed) */
volatile int16_t  g_i_iu_ma   = 0;
volatile int16_t  g_i_iv_ma   = 0;
volatile int16_t  g_i_iw_ma   = 0;

/* EMA-filtered current (Q8 fixed-point: value = actual_mA × 256) */
volatile int32_t  g_i_iu_filt = 0;
volatile int32_t  g_i_iv_filt = 0;
volatile int32_t  g_i_iw_filt = 0;

/* Display-friendly: mA + 10000, always positive for J-Scope */
volatile uint16_t g_i_iu_disp = 10000;
volatile uint16_t g_i_iv_disp = 10000;
volatile uint16_t g_i_iw_disp = 10000;

/* 2nd-order Butterworth IIR (fc=200Hz, fs=50kHz, -40dB/dec)
 * Designed in MATLAB: [b,a] = butter(2, 200/25000)
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2] */
#define BIQUAD_B0  0.0001551484f
#define BIQUAD_B1  0.0003102968f
#define BIQUAD_B2  0.0001551484f
#define BIQUAD_A1  (-1.9644605802f)   /* -a1 in diff eq = +1.96446*y[n-1] */
#define BIQUAD_A2  0.9650811739f       /* -a2 in diff eq = -0.96508*y[n-2] */

/* Biquad state: x[n-1], x[n-2], y[n-1], y[n-2] per phase */
static float s_fX1U = 0.0f, s_fX2U = 0.0f, s_fY1U = 0.0f, s_fY2U = 0.0f;
static float s_fX1V = 0.0f, s_fX2V = 0.0f, s_fY1V = 0.0f, s_fY2V = 0.0f;
static float s_fX1W = 0.0f, s_fX2W = 0.0f, s_fY1W = 0.0f, s_fY2W = 0.0f;
static bool  s_bBiquadInit = false;

/* Three-phase sum (should be ~0 mA / ~6144 raw) */
volatile int32_t  g_i_uvw_ma  = 0;
volatile int32_t  g_i_uvw_raw  = 0;

/* Sample count */
volatile uint32_t g_i_sample_cnt = 0;

/* Module running state */
volatile uint8_t  g_i_running = 0;

/* Calibration state and zero references */
volatile uint8_t  g_i_calib_state  = 0;   /* 0=idle, 1=in_progress, 2=done */
volatile uint16_t g_i_calib_zero_u = 2048;
volatile uint16_t g_i_calib_zero_v = 2048;
volatile uint16_t g_i_calib_zero_w = 2048;

/*******************************************************************************
 * Local variables ('static')
 ******************************************************************************/

/* Latest current data (ISR-updated) */
static stc_i_data_t s_stcIData;

/* Initialization flag */
static bool s_bIInitialized = false;

/* User callback */
static i_callback_t s_pfnUserCallback = NULL;

/* Calibration accumulators (ISR writes, I_Calibrate reads after blocking) */
static volatile int32_t s_i32CalibSumU = 0;
static volatile int32_t s_i32CalibSumV = 0;
static volatile int32_t s_i32CalibSumW = 0;
static volatile int32_t s_i32CalibCnt  = 0;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

static void I_SetPinAnalogMode(uint8_t u8Port, uint8_t u8Pin);
static void I_AdcConfig(void);
static void I_TriggerConfig(void);
static void I_IrqConfig(void);

static char I_GetPortLetter(uint8_t u8Port);
static uint8_t I_GetPinNumber(uint16_t u16Pin);

/*******************************************************************************
 * Helper functions
 ******************************************************************************/

static uint8_t I_GetPinNumber(uint16_t u16Pin)
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

static char I_GetPortLetter(uint8_t u8Port)
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
static void I_SetPinAnalogMode(uint8_t u8Port, uint8_t u8Pin)
{
    stc_gpio_init_t stcGpioInit;
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    (void)GPIO_Init(u8Port, u8Pin, &stcGpioInit);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
}

/*******************************************************************************
 * ADC2 configuration
 ******************************************************************************/

/**
 * @brief  Configure ADC1 SEQ_B for 3-channel current scan
 *         SEQ_B single-shot: CH5(IU), CH6(IV), CH7(IW)
 * @note   ADC1 is already initialized by BEMF (SEQ_A). We only add SEQ_B channels.
 *         ADC_Init must NOT be called again — it would reset SEQ_A config.
 */
static void I_AdcConfig(void)
{
    /* Enable ADC1 peripheral clock (idempotent if BEMF already enabled it) */
    FCG_Fcg3PeriphClockCmd(I_ADC_PERIPH_CLK, ENABLE);

    /* Configure pins and enable channels in SEQ_B */
    struct {
        uint8_t u8Port;
        uint8_t u8Pin;
        uint8_t u8Channel;
        const char *pszName;
    } astcChannels[3] = {
        { I_U_PORT, I_U_PIN, I_CH_U, "IU" },
        { I_V_PORT, I_V_PIN, I_CH_V, "IV" },
        { I_W_PORT, I_W_PIN, I_CH_W, "IW" },
    };

    for (uint8_t i = 0; i < 3; i++) {
        /* Set pin to analog mode */
        I_SetPinAnalogMode(astcChannels[i].u8Port, astcChannels[i].u8Pin);

        /* Enable ADC channel in SEQ_B */
        ADC_ChCmd(I_ADC_UNIT, I_ADC_SEQ, astcChannels[i].u8Channel, ENABLE);

        I_DEBUG("%s configured: P%c%d (ADC1_CH%d, SEQ_B)\r\n",
               astcChannels[i].pszName,
               I_GetPortLetter(astcChannels[i].u8Port),
               I_GetPinNumber(astcChannels[i].u8Pin),
               astcChannels[i].u8Channel);
    }

    I_DEBUG("ADC1 SEQ_B configured for 3-channel current scan\r\n");
}

/*******************************************************************************
 * ADC1 SEQ_B hardware trigger configuration
 ******************************************************************************/

/**
 * @brief  Configure ADC1 SEQ_B hardware trigger
 *         Shares BEMF's SCMP0 via EVT0 (AOS_ADC1_0), both sequences fire at PWM peak.
 */
static void I_TriggerConfig(void)
{
    /* SEQ_B uses EVT0, same as BEMF's SEQ_A — both triggered by SCMP0 at PWM peak */
    ADC_TriggerConfig(I_ADC_UNIT, I_ADC_SEQ, I_ADC_HARDTRIG);
    ADC_TriggerCmd(I_ADC_UNIT, I_ADC_SEQ, ENABLE);

    MAIN_D("[I] SEQ_B trigger: EVT0 (shared with BEMF SCMP0)\r\n");
}

/*******************************************************************************
 * Interrupt configuration & ISR
 ******************************************************************************/

/**
 * @brief  ADC1 EOCB interrupt callback
 *         Reads 3 current channels from DR5/DR6/DR7, converts to mA.
 */
static void I_IrqCallback(void)
{
    /* Clear SEQ_B end-of-conversion flag */
    ADC_ClearStatus(I_ADC_UNIT, ADC_FLAG_EOCB);

    /* Read ADC1 DR5(PA5/IU), DR6(PA6/IV), DR7(PA7/IW) */
    uint16_t u16IU = ADC_GetValue(I_ADC_UNIT, I_CH_U);
    uint16_t u16IV = ADC_GetValue(I_ADC_UNIT, I_CH_V);
    uint16_t u16IW = ADC_GetValue(I_ADC_UNIT, I_CH_W);

    /* DEBUG: print first 3 ISR entries unconditionally */
    {
        static uint8_t s_u8FirstPrints = 3;
        if (s_u8FirstPrints > 0) {
            s_u8FirstPrints--;
            MAIN_D("[I] ISR fired! cnt=%lu DR5=%u DR6=%u DR7=%u\r\n",
                   s_stcIData.u32SampleCount, u16IU, u16IV, u16IW);
        }
    }

    /* During calibration: accumulate raw values, skip mA conversion */
    if (g_i_calib_state == 1) {
        s_i32CalibSumU += (int32_t)u16IU;
        s_i32CalibSumV += (int32_t)u16IV;
        s_i32CalibSumW += (int32_t)u16IW;
        s_i32CalibCnt++;
    }

    /* Apply calibrated zero reference, then convert to mA */
    uint16_t u16ZeroU = (g_i_calib_state == 2) ? g_i_calib_zero_u : I_ADC_ZERO;
    uint16_t u16ZeroV = (g_i_calib_state == 2) ? g_i_calib_zero_v : I_ADC_ZERO;
    uint16_t u16ZeroW = (g_i_calib_state == 2) ? g_i_calib_zero_w : I_ADC_ZERO;

    int16_t i16IU_mA = I_ADC_TO_MA_REF(u16IU, u16ZeroU);
    int16_t i16IV_mA = I_ADC_TO_MA_REF(u16IV, u16ZeroV);
    int16_t i16IW_mA = I_ADC_TO_MA_REF(u16IW, u16ZeroW);

    /* 2nd-order Butterworth IIR (fc=200Hz, fs=50kHz, -40dB/dec) */
    float fIU, fIV, fIW;
    if (!s_bBiquadInit) {
        /* Seed states with first sample (fast settling, no ramp-up) */
        s_fX1U = s_fX2U = s_fY1U = s_fY2U = (float)i16IU_mA;
        s_fX1V = s_fX2V = s_fY1V = s_fY2V = (float)i16IV_mA;
        s_fX1W = s_fX2W = s_fY1W = s_fY2W = (float)i16IW_mA;
        s_bBiquadInit = true;
        fIU = (float)i16IU_mA;
        fIV = (float)i16IV_mA;
        fIW = (float)i16IW_mA;
    } else {
        fIU = BIQUAD_B0 * (float)i16IU_mA + BIQUAD_B1 * s_fX1U + BIQUAD_B2 * s_fX2U
              - BIQUAD_A1 * s_fY1U - BIQUAD_A2 * s_fY2U;
        fIV = BIQUAD_B0 * (float)i16IV_mA + BIQUAD_B1 * s_fX1V + BIQUAD_B2 * s_fX2V
              - BIQUAD_A1 * s_fY1V - BIQUAD_A2 * s_fY2V;
        fIW = BIQUAD_B0 * (float)i16IW_mA + BIQUAD_B1 * s_fX1W + BIQUAD_B2 * s_fX2W
              - BIQUAD_A1 * s_fY1W - BIQUAD_A2 * s_fY2W;
        /* Shift input history */
        s_fX2U = s_fX1U; s_fX1U = (float)i16IU_mA;
        s_fX2V = s_fX1V; s_fX1V = (float)i16IV_mA;
        s_fX2W = s_fX1W; s_fX1W = (float)i16IW_mA;
        /* Shift output history */
        s_fY2U = s_fY1U; s_fY1U = fIU;
        s_fY2V = s_fY1V; s_fY1V = fIV;
        s_fY2W = s_fY1W; s_fY1W = fIW;
    }
    int16_t i16IU_fmA = (int16_t)fIU;
    int16_t i16IV_fmA = (int16_t)fIV;
    int16_t i16IW_fmA = (int16_t)fIW;

    /* Update internal data structure */
    s_stcIData.u16IU     = u16IU;
    s_stcIData.u16IV     = u16IV;
    s_stcIData.u16IW     = u16IW;
    s_stcIData.i16IU_mA  = i16IU_mA;
    s_stcIData.i16IV_mA  = i16IV_mA;
    s_stcIData.i16IW_mA  = i16IW_mA;
    s_stcIData.u32SampleCount++;
    s_stcIData.u8NewData = 1U;

    /* Update JScope global variables */
    g_i_iu_raw  = u16IU;
    g_i_iv_raw  = u16IV;
    g_i_iw_raw  = u16IW;
    g_i_iu_ma   = i16IU_mA;
    g_i_iv_ma   = i16IV_mA;
    g_i_iw_ma   = i16IW_mA;
    g_i_iu_filt = (int32_t)((float)i16IU_fmA * 256.0f);  /* Q8: ×256 for J-Scope */
    g_i_iv_filt = (int32_t)((float)i16IV_fmA * 256.0f);
    g_i_iw_filt = (int32_t)((float)i16IW_fmA * 256.0f);
    g_i_iu_disp = (uint16_t)((int32_t)i16IU_fmA * 10 + 10000);
    g_i_iv_disp = (uint16_t)((int32_t)i16IV_fmA * 10 + 10000);
    g_i_iw_disp = (uint16_t)((int32_t)i16IW_fmA * 10 + 10000);
    g_i_uvw_raw = (int32_t)u16IU + (int32_t)u16IV + (int32_t)u16IW;
    g_i_uvw_ma  = (int32_t)i16IU_mA + (int32_t)i16IV_mA + (int32_t)i16IW_mA;
    g_i_sample_cnt = s_stcIData.u32SampleCount;

    /* Invoke user callback if registered */
    if (s_pfnUserCallback != NULL) {
        s_pfnUserCallback(&s_stcIData);
    }
}

/**
 * @brief  Register ADC1 EOCB interrupt via INTC sign-in
 */
static void I_IrqConfig(void)
{
    stc_irq_signin_config_t stcIrq;

    stcIrq.enIntSrc    = I_ADC_INT_SRC;
    stcIrq.enIRQn      = I_ADC_IRQn;
    stcIrq.pfnCallback = &I_IrqCallback;

    if (LL_OK != INTC_IrqSignIn(&stcIrq)) {
        MAIN_D("[I] ERROR: INTC_IrqSignIn failed for ADC1 EOCB!\r\n");
        return;
    }

    NVIC_ClearPendingIRQ(stcIrq.enIRQn);
    NVIC_SetPriority(stcIrq.enIRQn, I_ADC_INT_PRIO);
    NVIC_EnableIRQ(stcIrq.enIRQn);

    /* Enable ADC1 EOCB interrupt */
    ADC_IntCmd(I_ADC_UNIT, ADC_INT_EOCB, ENABLE);

    MAIN_D("[I] ADC1 EOCB ISR registered: INT_SRC=%u, IRQn=%d, prio=%d\r\n",
           (unsigned)I_ADC_INT_SRC, (int)I_ADC_IRQn, (int)I_ADC_INT_PRIO);
}

/*******************************************************************************
 * API — Lifecycle
 ******************************************************************************/

/**
 * @brief  Initialize current sensing module
 * @note   Call order requirements (same as BEMF):
 *         1. TMR4 PWM must be configured and running (TMR4_PWM_Config + StartOutput)
 *         2. Call after Hardware_Init() + CommRunner_Init()
 */
void I_Init(void)
{
    MAIN_D("[I] Init entry\r\n");

    if (s_bIInitialized) {
        I_DEBUG("Already initialized\r\n");
        return;
    }

    /* Reset data structure */
    memset(&s_stcIData, 0, sizeof(s_stcIData));

    /* 1. Configure ADC1 SEQ_B: pins + CH5/6/7 (SEQ_A & mode already set by BEMF) */
    I_AdcConfig();

    /* 2. SEQ_B trigger = EVT0 (same SCMP0 as BEMF, shared via AOS_ADC1_0) */
    I_TriggerConfig();

    /* 3. Register EOCB interrupt */
    I_IrqConfig();

    s_bIInitialized = true;
    g_i_running = 1;
    MAIN_D("[I] Init done: ADC1_SEQ_B shares SCMP0 trigger with BEMF, ISR=INT116_EOCB\r\n");
}

/*******************************************************************************
 * I_Calibrate — Blocking zero-offset calibration (500ms)
 ******************************************************************************/

/**
 * @brief  Blocking zero-offset calibration.
 *         Samples all 3 current channels for 500ms at ~50kHz, computes
 *         per-phase average as the zero reference, and stores the offsets.
 * @note   Must be called AFTER I_Init and BEFORE motor starts.
 *         Blocks for 500ms using tickTimer_DelayMs.
 */
void I_Calibrate(void)
{
    MAIN_D("[I] Calibration started (500ms blocking)...\r\n");

    /* Force all PWM channels OFF — motor truly floating, zero current */
    TMR4_PWM_SetChannelMode(TMR4_CHANNEL_U, TMR4_MODE_OFF, 0.0f);
    TMR4_PWM_SetChannelMode(TMR4_CHANNEL_V, TMR4_MODE_OFF, 0.0f);
    TMR4_PWM_SetChannelMode(TMR4_CHANNEL_W, TMR4_MODE_OFF, 0.0f);
    MAIN_D("[I] All PWM channels forced OFF for calibration\r\n");

    /* Reset accumulators */
    s_i32CalibSumU = 0;
    s_i32CalibSumV = 0;
    s_i32CalibSumW = 0;
    s_i32CalibCnt  = 0;

    /* Arm calibration mode — ISR starts accumulating */
    g_i_calib_state = 1;

    /* Block 500ms. ISR fires ~25000 times during this period. */
    tickTimer_DelayMs(500);

    /* Stop calibration */
    g_i_calib_state = 2;

    /* Compute per-phase average as zero reference */
    if (s_i32CalibCnt > 0) {
        g_i_calib_zero_u = (uint16_t)(s_i32CalibSumU / s_i32CalibCnt);
        g_i_calib_zero_v = (uint16_t)(s_i32CalibSumV / s_i32CalibCnt);
        g_i_calib_zero_w = (uint16_t)(s_i32CalibSumW / s_i32CalibCnt);
    }

    MAIN_D("[I] Calibration done: %ld samples, zero_ref U=%u V=%u W=%u\r\n",
           s_i32CalibCnt, g_i_calib_zero_u, g_i_calib_zero_v, g_i_calib_zero_w);
}

/**
 * @brief  Deinitialize current sensing module
 */
void I_DeInit(void)
{
    s_bIInitialized = false;
    s_pfnUserCallback = NULL;
    g_i_running = 0;

    /* Disable ADC1 EOCB interrupt */
    ADC_IntCmd(I_ADC_UNIT, ADC_INT_EOCB, DISABLE);
    NVIC_DisableIRQ(I_ADC_IRQn);
    NVIC_ClearPendingIRQ(I_ADC_IRQn);

    /* Disable ADC1 SEQ_B trigger */
    ADC_TriggerCmd(I_ADC_UNIT, I_ADC_SEQ, DISABLE);

    /* Clear data */
    memset(&s_stcIData, 0, sizeof(s_stcIData));

    I_DEBUG("Deinitialized\r\n");
}

/*******************************************************************************
 * API — Data access
 ******************************************************************************/

/**
 * @brief  Get latest 3-phase current data
 * @param  pData  Output data structure
 * @note   Reads current values directly from ADC2 DR registers, not cached.
 */
void I_GetData(stc_i_data_t *pData)
{
    if (pData == NULL) {
        return;
    }

    /* Read current values from ADC2 data registers */
    pData->u16IU = ADC_GetValue(I_ADC_UNIT, I_CH_U);
    pData->u16IV = ADC_GetValue(I_ADC_UNIT, I_CH_V);
    pData->u16IW = ADC_GetValue(I_ADC_UNIT, I_CH_W);
    uint16_t u16Z;
    u16Z = (g_i_calib_state == 2) ? g_i_calib_zero_u : I_ADC_ZERO;
    pData->i16IU_mA = I_ADC_TO_MA_REF(pData->u16IU, u16Z);
    u16Z = (g_i_calib_state == 2) ? g_i_calib_zero_v : I_ADC_ZERO;
    pData->i16IV_mA = I_ADC_TO_MA_REF(pData->u16IV, u16Z);
    u16Z = (g_i_calib_state == 2) ? g_i_calib_zero_w : I_ADC_ZERO;
    pData->i16IW_mA = I_ADC_TO_MA_REF(pData->u16IW, u16Z);

    /* Copy sample count and clear flag */
    pData->u32SampleCount = s_stcIData.u32SampleCount;
    pData->u8NewData = s_stcIData.u8NewData;
    s_stcIData.u8NewData = 0U;
}

/**
 * @brief  Get raw ADC value for a single current channel
 * @param  u8Phase  0=U, 1=V, 2=W
 * @return Raw ADC value (0-4095), 0 if invalid
 */
uint16_t I_GetRawValue(uint8_t u8Phase)
{
    uint8_t u8Channel;
    switch (u8Phase) {
        case 0: u8Channel = I_CH_U; break;
        case 1: u8Channel = I_CH_V; break;
        case 2: u8Channel = I_CH_W; break;
        default: return 0;
    }
    return ADC_GetValue(I_ADC_UNIT, u8Channel);
}

/**
 * @brief  Get current in mA for a single phase
 * @param  u8Phase  0=U, 1=V, 2=W
 * @return Current in mA (signed), 0 if invalid
 */
int16_t I_GetCurrentMA(uint8_t u8Phase)
{
    uint16_t u16Raw = I_GetRawValue(u8Phase);
    if (u16Raw == 0) {
        return 0;
    }
    uint16_t u16Zero;
    if (g_i_calib_state == 2) {
        switch (u8Phase) {
            case 0: u16Zero = g_i_calib_zero_u; break;
            case 1: u16Zero = g_i_calib_zero_v; break;
            case 2: u16Zero = g_i_calib_zero_w; break;
            default: return 0;
        }
    } else {
        u16Zero = I_ADC_ZERO;
    }
    return I_ADC_TO_MA_REF(u16Raw, u16Zero);
}

/*******************************************************************************
 * API — Callback
 ******************************************************************************/

/**
 * @brief  Register callback for new current data notification
 * @param  pfnCallback  Callback function (NULL to unregister)
 * @note   Callback runs in ADC2 ISR context — keep it short.
 */
void I_RegisterCallback(i_callback_t pfnCallback)
{
    s_pfnUserCallback = pfnCallback;
    I_DEBUG("Callback %s\r\n", (pfnCallback != NULL) ? "registered" : "unregistered");
}

/*******************************************************************************
 * API — Debug
 ******************************************************************************/

#ifdef DEBUG
void I_PrintDebugInfo(void)
{
    I_DEBUG("=== Current Module Debug Info ===\r\n");
    I_DEBUG("Initialized: %s\r\n", s_bIInitialized ? "Yes" : "No");
    I_DEBUG("Samples: %lu\r\n", s_stcIData.u32SampleCount);
    I_DEBUG("Latest raw:  IU=%u, IV=%u, IW=%u\r\n",
           s_stcIData.u16IU, s_stcIData.u16IV, s_stcIData.u16IW);
    I_DEBUG("Latest mA:   IU=%d, IV=%d, IW=%d\r\n",
           s_stcIData.i16IU_mA, s_stcIData.i16IV_mA, s_stcIData.i16IW_mA);
    I_DEBUG("Zero ref: %u (1650mV)\r\n", I_ADC_ZERO);
    I_DEBUG("Scale: 1 ADC count = %d.%d mA\r\n", I_MA_PER_ADC >> I_MA_SHIFT,
           (int)(((I_MA_PER_ADC & 0xFF) * 100) >> I_MA_SHIFT));
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
