/**
 *******************************************************************************
 * @file  cur_loop.c
 * @brief 10kHz current PI loop for six-step BLDC (learning)
 *
 *        Mounted on ADC1 EOCB ISR (100kHz) via I_RegisterCallback, decimated
 *        by CURLOOP_DECIMATION (10 -> 10kHz control rate).
 *        Feedback: active high-side phase current (from fixed state table,
 *        selected by g_scope_step), window-averaged over the decimation window.
 *        dt: measured from Timer6 microsecond timestamp.
 *        Output: Commutation_SetActiveDuty() -> OCCR (takes effect at next PWM peak).
 *
 *        Active only in COMM_RUNNER_CURLOOP_FW mode while hall FSM is RUNNING.
 *******************************************************************************
 */

#include "cur_loop.h"
#include "I.h"
#include "hall_sensor_3ch.h"
#include "dev_commutation.h"
#include "dev_comm_runner.h"
#include "timer6_timebase.h"

#define CURLOOP_DECIMATION   10u   /* 100kHz / 10 = 10kHz */
#define CURLOOP_DT_FIRST_US  100u  /* assumed 10kHz period for the very first call */

/* Keil Watch: current setpoint (mA) */
volatile float g_i_ref_ma = 500.0f;

/* J-Scope observability */
volatile float g_scope_i_ref  = 0.0f;
volatile float g_scope_i_fb   = 0.0f;
volatile float g_scope_i_duty = 0.0f;
volatile float g_scope_i_err  = 0.0f;

/* Current-loop PI config (Keil Watch tunable) */
pid_config_t g_cur_pid_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = 0.05f,    /* % duty per mA error */
    .ki           = 0.005f,   /* % duty per (mA*s) */
    .kd           = 0.0f,
    .output_min   = 2.0f,
    .output_max   = 98.0f,
    .integral_max = 50.0f,
    .i_term_max   = 10.0f,    /* I contribution clamped to +/-10% */
    .update_ms    = 0,        /* no throttle: run at every decimated call */
};

static pid_state_t s_pid;
static int32_t  s_sum_ma  = 0;
static uint8_t  s_cnt     = 0;
static uint64_t s_last_us = 0;
static uint8_t  s_inited  = 0;

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

static void curloop_isr(const stc_i_data_t *pData)
{
    if (CommRunner_GetMode() != COMM_RUNNER_CURLOOP_FW ||
        !CommRunner_IsRunning()) {
        /* not active: reset window so restart starts clean */
        s_last_us = 0;
        s_cnt     = 0;
        s_sum_ma  = 0;
        return;
    }

    s_sum_ma += curloop_feedback(pData);
    s_cnt++;

    if (s_cnt >= CURLOOP_DECIMATION) {
        s_cnt = 0;
        float fb = (float)s_sum_ma / (float)CURLOOP_DECIMATION;
        s_sum_ma = 0;

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
        Commutation_SetActiveDuty(g_scope_step, duty);

        g_scope_i_ref  = g_i_ref_ma;
        g_scope_i_fb   = fb;
        g_scope_i_duty = duty;
        g_scope_i_err  = g_i_ref_ma - fb;
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
