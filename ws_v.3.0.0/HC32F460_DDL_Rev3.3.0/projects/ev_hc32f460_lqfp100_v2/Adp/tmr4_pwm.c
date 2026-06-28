#include "tmr4_pwm.h"
#include "hc32_ll_tmr4.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_clk.h"
#include "hc32_ll_utility.h"
#include "rtt_log.h"
#include <stdbool.h>

/*=============================================================================
 * Shadow register control (1=ON, 0=OFF)
 *   ON:  writes go to shadow → transferred at counter PEAK → 3-ch sync, no glitch
 *   OFF: writes take effect immediately (IMMED)
 *=============================================================================*/
#define TMR4_SHADOW_CPSR_ENABLE   1U   /* Counter period    (0=immed, 1=PEAK) */
#define TMR4_SHADOW_OCCR_ENABLE   1U   /* Compare value     (0=immed, 1=PEAK) */
#define TMR4_SHADOW_OCMR_ENABLE   1U   /* Compare mode      (0=immed, 1=PEAK) */

/*=============================================================================
 * Half-bridge pair descriptor (ROM table, channel enum used as index)
 *=============================================================================*/
typedef struct {
    uint32_t oc_ch_h;       /* High-side OC channel */
    uint32_t oc_ch_l;       /* Low-side OC channel */
    uint32_t pwm_ch;        /* PWM channel */
    uint16_t pin_h;         /* High-side GPIO pin */
    uint16_t pin_l;         /* Low-side GPIO pin */
} pwm_pair_desc_t;

static const pwm_pair_desc_t s_pairs[3] = {
    /* CH_U: PB9(OUH) + PB8(OUL) */
    { TMR4_OC_CH_UH, TMR4_OC_CH_UL, TMR4_CHANNEL_U,
      GPIO_PIN_09,   GPIO_PIN_08 },
    /* CH_V: PB7(OVH) + PB6(OVL) */
    { TMR4_OC_CH_VH, TMR4_OC_CH_VL, TMR4_CHANNEL_V,
      GPIO_PIN_07,   GPIO_PIN_06 },
    /* CH_W: PB5(OWH) + PB4(OWL) */
    { TMR4_OC_CH_WH, TMR4_OC_CH_WL, TMR4_CHANNEL_W,
      GPIO_PIN_05,   GPIO_PIN_04 },
};

/*=============================================================================
 * Module internal state
 *=============================================================================*/
static bool               s_bConfigured = false;
static uint16_t           s_u16Period   = 0U;
static tmr4_output_type_t s_channel_type[3] = {
    TMR4_OUTPUT_SYNC,
    TMR4_OUTPUT_SYNC,
    TMR4_OUTPUT_SYNC,
};
static tmr4_channel_mode_t s_ch_op_mode[3] = {
    TMR4_MODE_OFF,
    TMR4_MODE_OFF,
    TMR4_MODE_OFF,
};
/*=============================================================================
 * Apply shadow register settings to one OC channel
 *=============================================================================*/
static void Shadow_ApplyOC(uint32_t oc_ch)
{
    uint16_t cond = TMR4_OC_BUF_COND_PEAK;
    uint16_t immed = TMR4_OC_BUF_COND_IMMED;

    TMR4_OC_SetCompareBufCond(CM_TMR4_3, oc_ch,
        TMR4_OC_BUF_CMP_VALUE,
        TMR4_SHADOW_OCCR_ENABLE ? cond : immed);
    TMR4_OC_SetCompareBufCond(CM_TMR4_3, oc_ch,
        TMR4_OC_BUF_CMP_MD,
        TMR4_SHADOW_OCMR_ENABLE ? cond : immed);
}

/*=============================================================================
 * Dead-time conversion (nanoseconds -> timer ticks)
 *=============================================================================*/
static uint16_t DeadTimeNsToTicks(uint16_t dead_time_ns, uint32_t timer_hz)
{
    uint16_t u16Ticks;
    u16Ticks = (uint16_t)((uint64_t)dead_time_ns * timer_hz / 1000000000ULL);
    /* TMR4 hardware requires at least 1 tick for complementary dead-time to function */
    if (u16Ticks == 0U) {
        u16Ticks = 1U;
    }
    return u16Ticks;
}

/*=============================================================================
 * OC compare mode helpers (avoid 3x duplication of giant bitfield structs)
 *=============================================================================*/

/* SYNC mode: high-side channel compare mode (center-aligned PWM) */
static void OCMode_SyncHigh(CM_TMR4_TypeDef *TMR4x, uint32_t ch)
{
    un_tmr4_oc_ocmrh_t m;
    m.OCMRx = 0U;
    m.OCMRx_f.OCFDCH = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFPKH = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFUCH = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFZRH = TMR4_OC_OCF_SET;
    m.OCMRx_f.OPDCH  = TMR4_OC_HIGH;
    m.OCMRx_f.OPPKH  = TMR4_OC_HIGH;
    m.OCMRx_f.OPUCH  = TMR4_OC_LOW;
    m.OCMRx_f.OPZRH  = TMR4_OC_HOLD;
    m.OCMRx_f.OPNPKH = TMR4_OC_LOW;
    m.OCMRx_f.OPNZRH = TMR4_OC_HOLD;
    TMR4_OC_SetHighChCompareMode(TMR4x, ch, m);
}

/* SYNC mode: low-side channel compare mode (with EOP extension, synced to high-side) */
static void OCMode_SyncLow(CM_TMR4_TypeDef *TMR4x, uint32_t ch)
{
    un_tmr4_oc_ocmrl_t m;
    m.OCMRx = 0U;
    m.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OPDCL   = TMR4_OC_HIGH;
    m.OCMRx_f.OPPKL   = TMR4_OC_HIGH;
    m.OCMRx_f.OPUCL   = TMR4_OC_LOW;
    m.OCMRx_f.OPZRL   = TMR4_OC_HOLD;
    m.OCMRx_f.OPNPKL  = TMR4_OC_LOW;
    m.OCMRx_f.OPNZRL  = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPDCL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPPKL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPUCL  = TMR4_OC_LOW;
    m.OCMRx_f.EOPZRL  = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNPKL = TMR4_OC_LOW;
    m.OCMRx_f.EOPNZRL = TMR4_OC_HOLD;
    TMR4_OC_SetLowChCompareMode(TMR4x, ch, m);
}

/* COMPLEMENTARY mode: low-side channel compare mode (dead-timer auto-generates high-side) */
static void OCMode_CompLow(CM_TMR4_TypeDef *TMR4x, uint32_t ch)
{
    un_tmr4_oc_ocmrl_t m;
    m.OCMRx = 0U;
    m.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OPDCL   = TMR4_OC_INVT;
    m.OCMRx_f.OPPKL   = TMR4_OC_INVT;
    m.OCMRx_f.OPUCL   = TMR4_OC_INVT;
    m.OCMRx_f.OPZRL   = TMR4_OC_INVT;
    m.OCMRx_f.OPNPKL  = TMR4_OC_HOLD;
    m.OCMRx_f.OPNZRL  = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPDCL  = TMR4_OC_INVT;
    m.OCMRx_f.EOPPKL  = TMR4_OC_INVT;
    m.OCMRx_f.EOPUCL  = TMR4_OC_INVT;
    m.OCMRx_f.EOPZRL  = TMR4_OC_INVT;
    m.OCMRx_f.EOPNPKL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNZRL = TMR4_OC_HOLD;
    TMR4_OC_SetLowChCompareMode(TMR4x, ch, m);
}

/*=============================================================================
 * TMR4_PWM_Config - Initialize 3-channel PWM (GPIO + counter + OC + PWM)
 *=============================================================================*/
void TMR4_PWM_Config(const tmr4_pwm_config_t *pConfig)
{
    stc_tmr4_init_t     stcTmr4Init;
    stc_tmr4_oc_init_t  stcTmr4OcInit;
    stc_tmr4_pwm_init_t stcTmr4PwmInit;
    uint32_t u32TimerClock;
    uint16_t u16Polarity;
    int i;

    if (pConfig == NULL) {
        return;
    }

    s_channel_type[0] = pConfig->output_type_u;
    s_channel_type[1] = pConfig->output_type_v;
    s_channel_type[2] = pConfig->output_type_w;

    u32TimerClock = CLK_GetBusClockFreq(CLK_BUS_PCLK1);
    /* Triangle wave: one PWM cycle = 2 * period ticks */
    s_u16Period   = (uint16_t)(u32TimerClock / (pConfig->freq_hz * 2U));

    MAIN_D("[TMR4] clk=%lu Hz period=%u ticks [U:%s V:%s W:%s]",
           u32TimerClock, s_u16Period,
           (pConfig->output_type_u == TMR4_OUTPUT_COMPLEMENTARY) ? "COMP" : "SYNC",
           (pConfig->output_type_v == TMR4_OUTPUT_COMPLEMENTARY) ? "COMP" : "SYNC",
           (pConfig->output_type_w == TMR4_OUTPUT_COMPLEMENTARY) ? "COMP" : "SYNC");

    /* Polarity mapping */
    if (pConfig->active_high) {
        u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
    } else {
        u16Polarity = TMR4_PWM_OXH_INVT_OXL_INVT;
    }

    /* --- Enable TMR4_3 peripheral clock --- */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_3, ENABLE);

    /* --- GPIO: all 6 pins set to Func2 --- */
    LL_PERIPH_WE(LL_PERIPH_GPIO);

    /* Release PB4(NJTRST) from JTAG, keep SWD(PA13/PA14/PA15) */
    GPIO_SetDebugPort((uint8_t)(GPIO_PIN_DEBUG_JTAG & ~GPIO_PIN_DEBUG_SWD), DISABLE);

    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_2);  /* TIM4_3_OUH */
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_08, GPIO_FUNC_2);  /* TIM4_3_OUL */
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_07, GPIO_FUNC_2);  /* TIM4_3_OVH */
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_06, GPIO_FUNC_2);  /* TIM4_3_OVL */
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_05, GPIO_FUNC_2);  /* TIM4_3_OWH */
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_04, GPIO_FUNC_2);  /* TIM4_3_OWL */
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    /* --- Counter (shared by all 3 channels) --- */
    TMR4_StructInit(&stcTmr4Init);
    stcTmr4Init.u16ClockDiv    = TMR4_CLK_DIV1;
    stcTmr4Init.u16CountMode   = TMR4_MD_TRIANGLE;
    stcTmr4Init.u16PeriodValue = s_u16Period - 1U;
    TMR4_Init(CM_TMR4_3, &stcTmr4Init);
    TMR4_PeriodBufCmd(CM_TMR4_3, TMR4_SHADOW_CPSR_ENABLE ? ENABLE : DISABLE);

    /* --- OC channels & PWM channels (loop U/V/W) --- */
    TMR4_OC_StructInit(&stcTmr4OcInit);

    for (i = 0; i < 3; i++) {
        const pwm_pair_desc_t *pair = &s_pairs[i];

        /***** OC init *****/
        if (s_channel_type[i] == TMR4_OUTPUT_COMPLEMENTARY) {
            /* Complementary: low-side only, high-side auto-generated by dead-timer */
            TMR4_OC_Init(CM_TMR4_3, pair->oc_ch_l, &stcTmr4OcInit);
            OCMode_CompLow(CM_TMR4_3, pair->oc_ch_l);
            TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_l, ENABLE);
            Shadow_ApplyOC(pair->oc_ch_l);
        } else {
            /* SYNC: both high and low sides, initial compare = 50% */
            stcTmr4OcInit.u16CompareValue = stcTmr4Init.u16PeriodValue / 2U;
            stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_PEAK;

            TMR4_OC_Init(CM_TMR4_3, pair->oc_ch_h, &stcTmr4OcInit);
            TMR4_OC_Init(CM_TMR4_3, pair->oc_ch_l, &stcTmr4OcInit);

            OCMode_SyncHigh(CM_TMR4_3, pair->oc_ch_h);
            OCMode_SyncLow(CM_TMR4_3, pair->oc_ch_l);

            TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_h, ENABLE);
            TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_l, ENABLE);

            Shadow_ApplyOC(pair->oc_ch_h);
            Shadow_ApplyOC(pair->oc_ch_l);
        }

        /***** PWM init *****/
        TMR4_PWM_StructInit(&stcTmr4PwmInit);
        stcTmr4PwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        if (s_channel_type[i] == TMR4_OUTPUT_COMPLEMENTARY) {
            stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_DEAD_TMR;
        } else {
            stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_THROUGH;
        }
        TMR4_PWM_Init(CM_TMR4_3, pair->pwm_ch, &stcTmr4PwmInit);

        /* Polarity */
        TMR4_PWM_SetPolarity(CM_TMR4_3, pair->pwm_ch, u16Polarity);

        /* Dead-time (complementary mode only) */
        if (s_channel_type[i] == TMR4_OUTPUT_COMPLEMENTARY) {
            uint16_t u16DeadTicks = DeadTimeNsToTicks(pConfig->dead_time_ns, u32TimerClock);
            TMR4_PWM_SetDeadTimeValue(CM_TMR4_3, pair->pwm_ch,
                                      TMR4_PWM_PDAR_IDX, u16DeadTicks);
            TMR4_PWM_SetDeadTimeValue(CM_TMR4_3, pair->pwm_ch,
                                      TMR4_PWM_PDBR_IDX, u16DeadTicks);
        }
    }

    s_bConfigured = true;

    MAIN_D("[TMR4] All 3 channels configured: period=%u polar=%s",
           s_u16Period,
           pConfig->active_high ? "HIGH" : "LOW");
}

/*=============================================================================
 * TMR4_PWM_StartOutput - Start the counter (all channels pre-configured)
 *=============================================================================*/
void TMR4_PWM_StartOutput(void)
{
    if (!s_bConfigured) {
        return;
    }
    TMR4_Start(CM_TMR4_3);
}

/*=============================================================================
 * TMR4_PWM_StopOutput - Stop counter + disable all channels
 *=============================================================================*/
void TMR4_PWM_StopOutput(void)
{
    int i;

    if (!s_bConfigured) {
        return;
    }

    TMR4_Stop(CM_TMR4_3);

    for (i = 0; i < 3; i++) {
        const pwm_pair_desc_t *pair = &s_pairs[i];

        if (s_channel_type[i] == TMR4_OUTPUT_SYNC) {
            TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_h, DISABLE);
        }
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_l, DISABLE);
    }
}

/*=============================================================================
 * TMR4_PWM_EmergencyStop - Stop counter, clear count, zero duty, disable all
 *=============================================================================*/
void TMR4_PWM_EmergencyStop(void)
{
    int i;

    if (!s_bConfigured) {
        return;
    }

    TMR4_Stop(CM_TMR4_3);
    TMR4_ClearCountValue(CM_TMR4_3);

    for (i = 0; i < 3; i++) {
        const pwm_pair_desc_t *pair = &s_pairs[i];

        if (s_channel_type[i] == TMR4_OUTPUT_SYNC) {
            TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_h, DISABLE);
            TMR4_OC_SetCompareValue(CM_TMR4_3, pair->oc_ch_h, 0U);
        }
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_l, DISABLE);
        TMR4_OC_SetCompareValue(CM_TMR4_3, pair->oc_ch_l, 0U);
    }
}

/*=============================================================================
 * TMR4_PWM_SetDuty - Set duty cycle for a specific channel
 *   COMPLEMENTARY mode: write low-side OCCR only
 *   SYNC mode: write both high and low side with same value
 *=============================================================================*/
void TMR4_PWM_SetDuty(tmr4_pwm_channel_t channel, uint16_t u16Duty)
{
    uint16_t u16Compare;
    const pwm_pair_desc_t *pair;

    if (!s_bConfigured || channel > TMR4_CHANNEL_W) {
        return;
    }

    if (u16Duty > TMR4_PWM_DUTY_MAX) {
        u16Duty = TMR4_PWM_DUTY_MAX;
    }

    u16Compare = (uint16_t)(((uint32_t)s_u16Period * u16Duty) /
                            TMR4_PWM_DUTY_MAX);

    pair = &s_pairs[channel];

    switch (s_ch_op_mode[channel]) {
    case TMR4_MODE_HIGH_SIDE:
        TMR4_OC_SetCompareValue(CM_TMR4_3, pair->oc_ch_h, u16Compare);
        break;
    case TMR4_MODE_LOW_SIDE:
        /* Fixed 100%, no update */
        break;
    case TMR4_MODE_OFF:
    default:
        break;
    }
}

/*=============================================================================
 * TMR4_PWM_ChannelCmd - Enable or disable a specific channel pair
 *   COMPLEMENTARY mode: control low-side OC only
 *   SYNC mode: control both high and low side OC
 *=============================================================================*/
void TMR4_PWM_ChannelCmd(tmr4_pwm_channel_t channel, bool enable)
{
    const pwm_pair_desc_t *pair;

    if (!s_bConfigured || channel > TMR4_CHANNEL_W) {
        return;
    }

    pair = &s_pairs[channel];

    switch (s_ch_op_mode[channel]) {
    case TMR4_MODE_HIGH_SIDE:
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_h, enable ? ENABLE : DISABLE);
        /* L stays disabled (forced LOW) */
        break;
    case TMR4_MODE_LOW_SIDE:
        /* H stays disabled (forced LOW) */
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_l, enable ? ENABLE : DISABLE);
        break;
    case TMR4_MODE_OFF:
    default:
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_h, enable ? ENABLE : DISABLE);
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_l, enable ? ENABLE : DISABLE);
        break;
    }
}

/*=============================================================================
 * TMR4_PWM_PinCmd - Enable or disable a single pin (high or low side)
 *   COMPLEMENTARY mode: high_side control is a no-op (hardware auto-generated)
 *=============================================================================*/
void TMR4_PWM_PinCmd(tmr4_pwm_channel_t channel, bool high_side, bool enable)
{
    const pwm_pair_desc_t *pair;
    uint32_t oc_ch;

    if (!s_bConfigured || channel > TMR4_CHANNEL_W) {
        return;
    }

    /* In COMPLEMENTARY mode, the high side is auto-generated by the dead-timer */
    if (s_channel_type[channel] == TMR4_OUTPUT_COMPLEMENTARY && high_side) {
        return;
    }

    pair = &s_pairs[channel];
    oc_ch = high_side ? pair->oc_ch_h : pair->oc_ch_l;

    TMR4_OC_Cmd(CM_TMR4_3, oc_ch, enable ? ENABLE : DISABLE);
}

/*=============================================================================
 * TMR4_PWM_PinSetDuty - Set duty cycle for a single pin
 *=============================================================================*/
void TMR4_PWM_PinSetDuty(tmr4_pwm_channel_t channel, bool high_side, uint16_t u16Duty)
{
    const pwm_pair_desc_t *pair;
    uint32_t oc_ch;
    uint16_t u16Compare;

    if (!s_bConfigured || channel > TMR4_CHANNEL_W) {
        return;
    }

    if (u16Duty > TMR4_PWM_DUTY_MAX) {
        u16Duty = TMR4_PWM_DUTY_MAX;
    }

    u16Compare = (uint16_t)(((uint32_t)s_u16Period * u16Duty) / TMR4_PWM_DUTY_MAX);

    pair = &s_pairs[channel];
    oc_ch = high_side ? pair->oc_ch_h : pair->oc_ch_l;

    TMR4_OC_SetCompareValue(CM_TMR4_3, oc_ch, u16Compare);
}

/*=============================================================================
 * TMR4_PWM_PinSetInvalidLevel - Set output level when OC is disabled
 *   level_high=false → INVD_LOW (default), true → INVD_HIGH
 *=============================================================================*/
void TMR4_PWM_PinSetInvalidLevel(tmr4_pwm_channel_t channel, bool high_side, bool level_high)
{
    const pwm_pair_desc_t *pair;
    uint32_t oc_ch;

    if (!s_bConfigured || channel > TMR4_CHANNEL_W) {
        return;
    }

    pair = &s_pairs[channel];
    oc_ch = high_side ? pair->oc_ch_h : pair->oc_ch_l;

    TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, oc_ch,
        level_high ? TMR4_OC_INVD_HIGH : TMR4_OC_INVD_LOW);
}

/*=============================================================================
 * TMR4_PWM_SetChannelMode - Switch channel between SYNC and COMPLEMENTARY
 *=============================================================================*/
static uint16_t DutyFloatToCompare(float duty_pct)
{
    if (duty_pct < 0.0f) duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;
    return (uint16_t)((float)s_u16Period * duty_pct / 100.0f);
}

/*=============================================================================
 * TMR4_PWM_SetChannelMode - Switch channel for 6288T-MNS pre-driver
 *
 * OFF:       H & L both disabled, invalid level LOW → HIN=L, LIN=L → both OFF
 * HIGH_SIDE: H enabled (SYNC, PWM duty), L disabled (LOW) → HIN=PWM, LIN=L
 *            PWM ON: HIN=H,LIN=L → high-side ON. PWM OFF: HIN=L,LIN=L → both OFF.
 * LOW_SIDE:  H disabled (LOW), L enabled (SYNC, 100%) → HIN=L, LIN=H → low-side ON
 *=============================================================================*/
void TMR4_PWM_SetChannelMode(tmr4_pwm_channel_t channel, tmr4_channel_mode_t mode, float duty_pct)
{
    const pwm_pair_desc_t *pair;
    stc_tmr4_pwm_init_t stcPwmInit;
    stc_tmr4_oc_init_t  stcOcInit;
    uint16_t u16Compare;

    if (!s_bConfigured || channel > TMR4_CHANNEL_W) {
        return;
    }

    u16Compare = DutyFloatToCompare(duty_pct);
    pair = &s_pairs[channel];

    /* All 6288T-MNS modes use THROUGH (no dead-time needed) */
    s_channel_type[channel] = TMR4_OUTPUT_SYNC;
    s_ch_op_mode[channel]   = mode;

    switch (mode) {

    case TMR4_MODE_OFF:
        /* H=L=LOW → HIN=L, LIN=L → both OFF (6288T-MNS) */
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_h, DISABLE);
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_l, DISABLE);
        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->oc_ch_h, TMR4_OC_INVD_LOW);
        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->oc_ch_l, TMR4_OC_INVD_LOW);

        TMR4_PWM_StructInit(&stcPwmInit);
        stcPwmInit.u16Mode     = TMR4_PWM_MD_THROUGH;
        stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        stcPwmInit.u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
        TMR4_PWM_Init(CM_TMR4_3, pair->pwm_ch, &stcPwmInit);
        break;

    case TMR4_MODE_HIGH_SIDE:
        /* H=PWM, L=LOW → HIN=PWM, LIN=L → high-side FET chopping (6288T-MNS)
         *   PWM ON:  HIN=H, LIN=L → high-side ON
         *   PWM OFF: HIN=L, LIN=L → both OFF (body-diode freewheeling) */

        /* H channel: SYNC mode OC with PWM duty */
        TMR4_OC_StructInit(&stcOcInit);
        stcOcInit.u16CompareValue        = s_u16Period / 2U;
        stcOcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_PEAK;
        TMR4_OC_Init(CM_TMR4_3, pair->oc_ch_h, &stcOcInit);
        OCMode_SyncHigh(CM_TMR4_3, pair->oc_ch_h);
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_h, ENABLE);
        TMR4_OC_SetCompareValue(CM_TMR4_3, pair->oc_ch_h, u16Compare);
        Shadow_ApplyOC(pair->oc_ch_h);

        /* L channel: disabled, forced LOW */
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_l, DISABLE);
        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->oc_ch_l, TMR4_OC_INVD_LOW);

        TMR4_PWM_StructInit(&stcPwmInit);
        stcPwmInit.u16Mode     = TMR4_PWM_MD_THROUGH;
        stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        stcPwmInit.u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
        TMR4_PWM_Init(CM_TMR4_3, pair->pwm_ch, &stcPwmInit);
        break;

    case TMR4_MODE_LOW_SIDE:
        /* H=LOW, L=HIGH → HIN=L, LIN=H → low-side FET ON continuous (6288T-MNS) */

        /* H channel: disabled, forced LOW */
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_h, DISABLE);
        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->oc_ch_h, TMR4_OC_INVD_LOW);

        /* L channel: 100% duty = always HIGH */
        TMR4_OC_StructInit(&stcOcInit);
        stcOcInit.u16CompareValue        = s_u16Period / 2U;
        stcOcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_PEAK;
        TMR4_OC_Init(CM_TMR4_3, pair->oc_ch_l, &stcOcInit);
        OCMode_SyncLow(CM_TMR4_3, pair->oc_ch_l);
        TMR4_OC_Cmd(CM_TMR4_3, pair->oc_ch_l, ENABLE);
        TMR4_OC_SetCompareValue(CM_TMR4_3, pair->oc_ch_l, s_u16Period - 1U);  /* 100% duty */
        Shadow_ApplyOC(pair->oc_ch_l);

        TMR4_PWM_StructInit(&stcPwmInit);
        stcPwmInit.u16Mode     = TMR4_PWM_MD_THROUGH;
        stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        stcPwmInit.u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
        TMR4_PWM_Init(CM_TMR4_3, pair->pwm_ch, &stcPwmInit);
        break;
    }
}

/*=============================================================================
 * TMR4_PWM_SetFrequency - Update PWM frequency (Hz)
 *   Recalculates period. All duty values must be re-applied after calling this.
 *=============================================================================*/
void TMR4_PWM_SetFrequency(uint16_t freq_hz)
{
    uint32_t u32TimerClock;

    if (!s_bConfigured || freq_hz == 0U) {
        return;
    }

    u32TimerClock = CLK_GetBusClockFreq(CLK_BUS_PCLK1);
    s_u16Period = (uint16_t)(u32TimerClock / (freq_hz * 2U));
    TMR4_SetPeriodValue(CM_TMR4_3, s_u16Period - 1U);
}

/*=============================================================================
 * TMR4_PWM_SetDutyFloat - Set duty as float percentage (0.0f ~ 100.0f)
 *=============================================================================*/
void TMR4_PWM_SetDutyFloat(tmr4_pwm_channel_t channel, float duty_pct)
{
    uint16_t u16Compare;
    const pwm_pair_desc_t *pair;

    if (!s_bConfigured || channel > TMR4_CHANNEL_W) {
        return;
    }

    u16Compare = DutyFloatToCompare(duty_pct);
    pair = &s_pairs[channel];

    switch (s_ch_op_mode[channel]) {
    case TMR4_MODE_HIGH_SIDE:
        /* Only H channel has PWM; update its compare value */
        TMR4_OC_SetCompareValue(CM_TMR4_3, pair->oc_ch_h, u16Compare);
        break;
    case TMR4_MODE_LOW_SIDE:
        /* L channel fixed at 100%; no duty update needed */
        break;
    case TMR4_MODE_OFF:
    default:
        /* Both channels disabled; no duty update needed */
        break;
    }
}
