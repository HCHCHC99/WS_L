/**
 *******************************************************************************
 * @file  cur_loop.c
 * @brief 25kHz current PI loop for six-step BLDC (learning)
 *
 *        Mounted on ADC1 EOCB ISR (25kHz, 1:1 with PWM) via I_RegisterCallback.
 *        Runs the PI on EVERY ADC sample (current loop frequency == PWM frequency).
 *        Feedback: active high-side phase current (fixed state table, selected by
 *        g_scope_step), smoothed by a 5-tap sliding average (~200us window).
 *        dt: measured from Timer6 microsecond timestamp.
 *        Output: Commutation_SetActiveDuty() -> OCCR (takes effect at next PWM peak).
 *
 *        Active only in COMM_RUNNER_CURLOOP_FW phase 1 (CommRunner_CurLoopActive()).
 *******************************************************************************
 */

#include "cur_loop.h"
#include "I.h"
#include "hall_sensor_3ch.h"
#include "dev_commutation.h"
#include "dev_comm_runner.h"
#include "timer6_timebase.h"
#include "TickTimer.h"
#include "rtt_log.h"

#define CURLOOP_WIN_SIZE     5u    /* 5-tap sliding average at 25kHz (~200us) */
#define CURLOOP_DT_FIRST_US  40u    /* assumed 25kHz period for the very first call */
#define CURLOOP_DUTY_RATE    1.0f  /* max duty change per control cycle (%) */

/* Keil Watch: current setpoint (mA) */
volatile float g_i_ref_ma = 800.0f;

/* J-Scope observability */
volatile float g_scope_i_ref  = 0.0f;
volatile float g_scope_i_fb   = 0.0f;
volatile float g_scope_i_duty = 0.0f;
volatile float g_scope_i_err  = 0.0f;
volatile float g_scope_i_ol   = 0.0f;   /* open-loop phase current estimate (mA) */

/* Current-loop PI config (Keil Watch tunable) */
pid_config_t g_cur_pid_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = 0.2f,     /* % duty per mA error (full authority: 500mA err -> 100%) */
    .ki           = 0.2f,     /* % duty per (mA*s) */
    .kd           = 0.0f,
    .output_min   = 2.0f,
    .output_max   = 98.0f,
    .integral_max = 500.0f,   /* mA*s */
    .i_term_max   = 20.0f,    /* I contribution clamped to +/-20% */
    .update_ms    = 0,        /* no throttle: run on every ADC sample (25kHz) */
};

static pid_state_t s_pid;
static int16_t  s_win[CURLOOP_WIN_SIZE];
static uint8_t  s_win_idx = 0;
static uint8_t  s_win_cnt = 0;
static int32_t  s_win_sum = 0;
static uint64_t s_last_us = 0;
static uint8_t  s_inited  = 0;
static float    s_ol_current_ma = 0.0f;   /* EMA of active-phase current during open-loop ramp */
static float    s_last_duty      = 80.0f; /* last applied duty (rate-limiter state) */

static int16_t curloop_feedback(const stc_i_data_t *pData)
{
    uint8_t ch = Commutation_GetPwmChannel(g_scope_step);
    switch (ch) {
        case 0:  return pData->i16IU_mA;
        case 1:  return pData->i16IV_mA;
        case 2:  return pData->i16IW_mA;
        default: return 0;
    }
}

static void curloop_win_reset(void)
{
    s_win_idx = 0;
    s_win_cnt = 0;
    s_win_sum = 0;
}

static void curloop_isr(const stc_i_data_t *pData)
{
    if (CommRunner_GetMode() != COMM_RUNNER_CURLOOP_FW) {
        /* mode not current-loop: reset state so restart starts clean */
        s_last_us = 0;
        curloop_win_reset();
        return;
    }

    if (!CommRunner_CurLoopActive()) {
        /* phase 0 (timed open loop): estimate running current for display only */
        float fb_inst = (float)curloop_feedback(pData);
        s_ol_current_ma += (fb_inst - s_ol_current_ma) * 0.02f;   /* EMA, tau~1ms */
        g_scope_i_ol = s_ol_current_ma;
        s_last_us = 0;
        curloop_win_reset();
        return;
    }

    if (s_last_us == 0) {
        /* fresh activation: bumpless handoff - start duty from the open-loop duty.
         * Ref stays at the user-set g_i_ref_ma (open-loop current capture is
         * unreliable because g_scope_step is not the active commutation step
         * during the timed ramp). The duty rate limiter smooths the takeover. */
        s_last_duty  = CommRunner_GetDuty();
        PID_Reset(&s_pid);
        curloop_win_reset();
    }

    /* Sliding 5-tap average of the active-phase current (every ADC sample) */
    {
        int16_t fb_inst = curloop_feedback(pData);
        if (s_win_cnt < CURLOOP_WIN_SIZE) {
            s_win_sum += fb_inst;
        } else {
            s_win_sum += fb_inst - s_win[s_win_idx];
        }
        s_win[s_win_idx] = fb_inst;
        s_win_idx = (s_win_idx + 1u) % CURLOOP_WIN_SIZE;
        if (s_win_cnt < CURLOOP_WIN_SIZE) {
            s_win_cnt++;
        }
    }
    float fb = (float)s_win_sum / (float)s_win_cnt;

    Timer6_Timebase_UpdateTimestamp();
    uint64_t now = Timer6_Timebase_GetTimestamp();
    uint32_t dt_us;
    if (s_last_us == 0) {
        dt_us = CURLOOP_DT_FIRST_US;
    } else {
        dt_us = (uint32_t)(now - s_last_us);
    }
    s_last_us = now;

    float duty = PID_UpdateUs(&s_pid, g_i_ref_ma, fb, dt_us);

    /* rate-limit duty: no 2%<->98% slam at handoff or during tuning */
    if (duty > s_last_duty + CURLOOP_DUTY_RATE) duty = s_last_duty + CURLOOP_DUTY_RATE;
    if (duty < s_last_duty - CURLOOP_DUTY_RATE) duty = s_last_duty - CURLOOP_DUTY_RATE;
    s_last_duty = duty;

    Commutation_SetActiveDuty(g_scope_step, duty);
    /* Keep runner s_duty in sync so the next Hall edge re-applies the same duty */
    CommRunner_SetDuty(duty);

    g_scope_i_ref  = g_i_ref_ma;
    g_scope_i_fb   = fb;
    g_scope_i_duty = duty;
    g_scope_i_err  = g_i_ref_ma - fb;

    {
        static uint32_t s_last_dbg = 0;
        uint32_t now_ms = (uint32_t)tickTimer_GetCount();
        if ((now_ms - s_last_dbg) >= 500u) {
            s_last_dbg = now_ms;
            MAIN_D("[CURLOOP] ref=%d fb=%d err=%d duty=%d%% i=%d dt=%luus step=%u",
                   (int)g_i_ref_ma, (int)fb, (int)(g_i_ref_ma - fb),
                   (int)(duty * 10) / 10, (int)s_pid.i_term,
                   (unsigned long)dt_us, (unsigned)g_scope_step);
        }
    }
}

void CurLoop_Init(void)
{
    if (s_inited) {
        return;
    }
    PID_Init(&s_pid, &g_cur_pid_cfg);
    I_RegisterCallback(curloop_isr);
    s_inited = 1;
}

void CurLoop_SetRef(float ma)
{
    g_i_ref_ma = ma;
}

float CurLoop_GetRef(void)
{
    return g_i_ref_ma;
}
