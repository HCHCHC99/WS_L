#include "dev_commutation.h"
#include "tmr4_pwm.h"

/*=============================================================================
 * Commutation state table
 *
 * Each row = { U_mode, U_duty_flag,  V_mode, V_duty_flag,  W_mode, W_duty_flag }
 *
 * mode:  M_OFF = both LOW (OFF),  M_HIGH = H=PWM/L=LOW (high-side),
 *        M_LOW = H=LOW/L=HIGH (low-side)
 * duty:  D_PWM = PWM duty (from parameter, only meaningful for M_HIGH),
 *        D_MIN = LOW_SIDE 100% (only meaningful for M_LOW),
 *        D_OFF = don't care (only meaningful for M_OFF)
 *=============================================================================*/
enum { M_OFF = 0, M_HIGH = 1, M_LOW = 2 };
enum { D_PWM = 0, D_MIN = 1, D_OFF = 2 };

static const uint8_t s_states[6][6] = {
    /*         U_mode  U_duty  V_mode  V_duty  W_mode  W_duty */
    /* UH_VL */ { M_HIGH, D_PWM, M_LOW, D_MIN, M_OFF, D_OFF },
    /* UH_WL */ { M_HIGH, D_PWM, M_OFF, D_OFF, M_LOW, D_MIN },
    /* VH_WL */ { M_OFF, D_OFF, M_HIGH, D_PWM, M_LOW, D_MIN },
    /* VH_UL */ { M_LOW, D_MIN, M_HIGH, D_PWM, M_OFF, D_OFF },
    /* WH_UL */ { M_LOW, D_MIN, M_OFF, D_OFF, M_HIGH, D_PWM },
    /* WH_VL */ { M_OFF, D_OFF, M_LOW, D_MIN, M_HIGH, D_PWM },
};

/* Step metadata for debug display */
static const struct {
    const char *high;   /* "UH"/"VH"/"WH" */
    const char *low;    /* "UL"/"VL"/"WL" */
    uint16_t    angle;  /* field angle in degrees */
} s_step_meta[6] = {
    {"UH", "VL", 330},  /* Step 0: UH+VL */
    {"UH", "WL",  30},  /* Step 1: UH+WL */
    {"VH", "WL",  90},  /* Step 2: VH+WL */
    {"VH", "UL", 150},  /* Step 3: VH+UL */
    {"WH", "UL", 210},  /* Step 4: WH+UL */
    {"WH", "VL", 270},  /* Step 5: WH+VL */
};

const char* Commutation_GetHighPhase(uint8_t step)
{
    if (step > 5) return "??";
    return s_step_meta[step].high;
}

const char* Commutation_GetLowPhase(uint8_t step)
{
    if (step > 5) return "??";
    return s_step_meta[step].low;
}

uint16_t Commutation_GetFieldAngle(uint8_t step)
{
    if (step > 5) return 0;
    return s_step_meta[step].angle;
}

/* Per-channel change detection */
static uint16_t s_last_freq     = 0U;
static uint8_t  s_last_ch_mode[3] = {0xFFU, 0xFFU, 0xFFU};
static float    s_last_ch_duty[3] = {0.0f, 0.0f, 0.0f};

/*=============================================================================
 * Commutation_Init - All 3 phases to OFF (both pins LOW → 6288T-MNS both FETs OFF)
 *=============================================================================*/
void Commutation_Init(void)
{
    int ch;
    for (ch = 0; ch < 3; ch++) {
        s_last_ch_mode[ch] = 0xFFU;  /* force per-channel reconfigure */
    }
    for (ch = 0; ch < 3; ch++) {
        TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch,
                                TMR4_MODE_OFF, COMM_DUTY_OFF_F);
    }
}

/*=============================================================================
 * Commutation_Step - Optimized: per-channel mode tracking.
 *   Only calls SetChannelMode (full reinit) when channel MODE changes.
 *   When mode is unchanged and only duty differs, calls SetDutyFloat (OCCR only).
 *   When nothing changed for a channel, skips entirely.
 *=============================================================================*/
void Commutation_Step(uint8_t state, uint16_t freq_hz, float duty_pct)
{
    int ch;

    if (state > 5U) {
        return;
    }

    /* Clamp duty to 2%~98% */
    if (duty_pct < COMM_DUTY_MIN_F) {
        duty_pct = COMM_DUTY_MIN_F;
    }
    if (duty_pct > COMM_DUTY_MAX_F) {
        duty_pct = COMM_DUTY_MAX_F;
    }

    /* Update frequency if changed */
    if (freq_hz != s_last_freq) {
        TMR4_PWM_SetFrequency(freq_hz);
        s_last_freq = freq_hz;
        /* Frequency change invalidates per-channel mode cache */
        for (ch = 0; ch < 3; ch++) {
            s_last_ch_mode[ch] = 0xFFU;
        }
    }

    /* Per-channel: only reconfigure what actually changed */
    for (ch = 0; ch < 3; ch++) {
        uint8_t new_mode = s_states[state][ch * 2U];
        uint8_t dflag    = s_states[state][ch * 2U + 1U];
        float   new_duty;

        if (dflag == D_PWM) {
            new_duty = duty_pct;         /* M_HIGH: H pin PWM duty */
        } else if (dflag == D_MIN) {
            new_duty = COMM_DUTY_MIN_F;  /* M_LOW: duty ignored (L fixed 100%) */
        } else {
            new_duty = COMM_DUTY_OFF_F;  /* M_OFF: duty ignored (both LOW) */
        }

        if (new_mode != s_last_ch_mode[ch]) {
            /* Mode changed: OFF / HIGH_SIDE / LOW_SIDE -> full channel reinit */
            tmr4_channel_mode_t tmr4_mode;
            if (new_mode == M_HIGH) {
                tmr4_mode = TMR4_MODE_HIGH_SIDE;
            } else if (new_mode == M_LOW) {
                tmr4_mode = TMR4_MODE_LOW_SIDE;
            } else {
                tmr4_mode = TMR4_MODE_OFF;
            }
            TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch, tmr4_mode, new_duty);
            s_last_ch_mode[ch] = new_mode;
            s_last_ch_duty[ch] = new_duty;
        } else if (dflag == D_PWM && new_duty != s_last_ch_duty[ch]) {
            /* Same mode, active PWM channel, duty changed � OCCR only (fast) */
            TMR4_PWM_SetDutyFloat((tmr4_pwm_channel_t)ch, new_duty);
            s_last_ch_duty[ch] = new_duty;
        }
        /* else: mode unchanged, duty unchanged, or non-PWM channel � skip */
    }

    (void)state;
}

/*=============================================================================
 * Commutation_Stop - All phases to OFF (both pins LOW → 6288T-MNS both FETs OFF)
 *=============================================================================*/
void Commutation_Stop(void)
{
    int ch;
    for (ch = 0; ch < 3; ch++) {
        TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch,
                                TMR4_MODE_OFF, COMM_DUTY_OFF_F);
    }
}
