/**
 *******************************************************************************
 * @file  I.h
 * @brief Current sensing module — 3-phase current via ADC1 SEQ_B interrupt mode
 *
 *        PA5 = ADC1_CH5 = IU  (current sensor U)
 *        PA6 = ADC1_CH6 = IV  (current sensor V)
 *        PA7 = ADC1_CH7 = IW  (current sensor W)
 *
 *        Sensor formula: VOUT = 1650 + IP(A) × 132 (mV)
 *          ±10A range, 3.3V / 12-bit ADC, zero = 2048 raw
 *
 *        Trigger chain:
 *          TMR4_3 SCMP2 (PWM peak) → AOS_ADC1_1 → ADC1_SEQ_B → EOCB ISR
 *
 *        ADC1 layout:
 *          SEQ_A (CH0-CH3): BEMF, TMR4_3 SCMP0 → AOS_ADC1_0 → DMA
 *          SEQ_B (CH5-CH7): Current, TMR4_3 SCMP2 → AOS_ADC1_1 → ISR
 *******************************************************************************
 */

#ifndef __I_H__
#define __I_H__

#include "main.h"
#include "Hardware.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/* Debug switch */
#ifdef DEBUG_I_WS
    #define I_DEBUG(fmt, ...)    MAIN_D("[I] " fmt, ##__VA_ARGS__)
#else
    #define I_DEBUG(fmt, ...)    ((void)0)
#endif

/* ===== Current channel definitions ===== */
#define I_CH_U                          (ADC_CH5)   /* PA5/ADC1_CH5: IU */
#define I_CH_V                          (ADC_CH6)   /* PA6/ADC1_CH6: IV */
#define I_CH_W                          (ADC_CH7)   /* PA7/ADC1_CH7: IW */
#define I_CHANNEL_COUNT                 (3U)

/* ===== Pin definitions ===== */
#define I_U_PORT                        (GPIO_PORT_A)
#define I_U_PIN                         (GPIO_PIN_05)
#define I_V_PORT                        (GPIO_PORT_A)
#define I_V_PIN                         (GPIO_PIN_06)
#define I_W_PORT                        (GPIO_PORT_A)
#define I_W_PIN                         (GPIO_PIN_07)

/* ===== ADC1 hardware configuration ===== */
#define I_ADC_UNIT                      (CM_ADC1)
#define I_ADC_PERIPH_CLK                (FCG3_PERIPH_ADC1)
#define I_ADC_SEQ                       (ADC_SEQ_B)

/* ===== Trigger: share BEMF's SCMP0 via EVT0 ===== */
#define I_ADC_HARDTRIG                  (ADC_HARDTRIG_EVT0)

/* ===== Interrupt configuration ===== */
#define I_ADC_INT_SRC                   (INT_SRC_ADC1_EOCB)
#define I_ADC_IRQn                      (INT116_IRQn)
#define I_ADC_INT_PRIO                  (DDL_IRQ_PRIO_03)

/* ===== Current conversion constants ===== */
#define I_ADC_ZERO                      (2048)      /* ADC raw at 0A (1650mV @ 3.3V/12bit) */
#define I_MA_PER_ADC                    (1563)      /* Fixed-point slope: 3300*1000/(4095*132) ≈ 6.105 mA/count, ×256 for shift */
#define I_MA_SHIFT                      (8U)        /* Right-shift after multiply */

/* Integer conversion: I_mA = (raw - zero_ref) * 1563 >> 8.  Error < 0.01%. */
#define I_ADC_TO_MA_REF(raw, zero)  ((int16_t)(((int32_t)((int32_t)(raw) - (int32_t)(zero)) * (int32_t)I_MA_PER_ADC) >> I_MA_SHIFT))

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/**
 * @brief  Single current sample (3-phase synchronized)
 */
typedef struct {
    uint16_t u16IU;         /* IU raw ADC */
    uint16_t u16IV;         /* IV raw ADC */
    uint16_t u16IW;         /* IW raw ADC */
    int16_t  i16IU_mA;      /* IU current (mA, signed) */
    int16_t  i16IV_mA;      /* IV current (mA, signed) */
    int16_t  i16IW_mA;      /* IW current (mA, signed) */
    uint32_t u32SampleCount;
    uint8_t  u8NewData;
} stc_i_data_t;

/**
 * @brief  Current data ready callback (called from ADC2 ISR context)
 */
typedef void (*i_callback_t)(const stc_i_data_t *pData);

/*******************************************************************************
 * Global variables for JScope (extern - defined in I.c)
 ******************************************************************************/

/* Raw ADC values (0-4095) */
extern volatile uint16_t g_i_iu_raw;
extern volatile uint16_t g_i_iv_raw;
extern volatile uint16_t g_i_iw_raw;

/* Current values (mA, signed) */
extern volatile int16_t  g_i_iu_ma;
extern volatile int16_t  g_i_iv_ma;
extern volatile int16_t  g_i_iw_ma;

/* EMA-filtered current (mA × 256, Q8 fixed-point, J-Scope: value/256 = mA) */
extern volatile int32_t  g_i_iu_filt;
extern volatile int32_t  g_i_iv_filt;
extern volatile int32_t  g_i_iw_filt;

/* Display-friendly: mA + 10000 offset (always positive, J-Scope safe). Subtract 10000 for real value. */
extern volatile uint16_t g_i_iu_disp;
extern volatile uint16_t g_i_iv_disp;
extern volatile uint16_t g_i_iw_disp;

/* Sum of three phases (should be ~0 mA / ~6144 raw) */
extern volatile int32_t  g_i_uvw_ma;
extern volatile int32_t  g_i_uvw_raw;

/* Sample count */
extern volatile uint32_t g_i_sample_cnt;

/* Module running state (0=stopped, 1=running) */
extern volatile uint8_t  g_i_running;

/* Calibration status (0=idle, 1=in progress, 2=done) */
extern volatile uint8_t  g_i_calib_state;

/* Calibrated zero references (per-phase ADC raw, after 500ms averaging) */
extern volatile uint16_t g_i_calib_zero_u;
extern volatile uint16_t g_i_calib_zero_v;
extern volatile uint16_t g_i_calib_zero_w;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/

/* Lifecycle */
void I_Init(void);
void I_DeInit(void);

/* Zero-offset calibration: blocks 500ms, samples all 3 phases, stores offsets */
void I_Calibrate(void);

/* Data access */
void     I_GetData(stc_i_data_t *pData);
uint16_t I_GetRawValue(uint8_t u8Phase);    /* 0=U, 1=V, 2=W */
int16_t  I_GetCurrentMA(uint8_t u8Phase);   /* 0=U, 1=V, 2=W, returns mA */

/* Callback (called in ISR context, keep short) */
void I_RegisterCallback(i_callback_t pfnCallback);

#ifdef DEBUG
void I_PrintDebugInfo(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __I_H__ */
