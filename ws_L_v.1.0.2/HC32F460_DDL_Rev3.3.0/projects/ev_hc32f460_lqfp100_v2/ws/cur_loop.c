/**
 *******************************************************************************
 * @file  cur_loop.c
 * @brief Current PI loop, 1:1 with PWM (frequency: MOTOR_PWM_FREQ_HZ)
 *
 *        Mounted on ADC1 EOCB ISR (1:1 with PWM) via I_RegisterCallback.
 *        Runs the PI on EVERY ADC sample (current loop frequency == PWM frequency).
 *        Feedback: active high-side phase current (fixed state table, selected by
 *        g_scope_step), smoothed by a 5-tap sliding average (window = 5 / PWM freq).
 *        dt: measured from Timer6 microsecond timestamp.
 *        Output: Commutation_SetActiveDuty() -> OCCR (takes effect at next PWM peak).
 *
 *        Active only in COMM_RUNNER_CURLOOP_FW / COMM_RUNNER_CASCADE_FW phase 1 (CommRunner_CurLoopActive()).
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
#include "motor_config.h"

/* Feedback window: keep a ~400us time-domain average across frequencies.
 * WIN_SIZE scales with MOTOR_PWM_FREQ_HZ (50k->20, 25k->10, 20k->8, 10k->4). */
#if   MOTOR_PWM_FREQ_HZ >= 50000u
  #define CURLOOP_WIN_SIZE 20u
#elif MOTOR_PWM_FREQ_HZ >= 25000u
  #define CURLOOP_WIN_SIZE 10u
#elif MOTOR_PWM_FREQ_HZ >= 20000u
  #define CURLOOP_WIN_SIZE 8u
#else
  #define CURLOOP_WIN_SIZE 4u
#endif
/* Skip this many ADC samples after a commutation edge: keep the window free of
 * transient / wrong-phase samples (the first PWM cycle after an edge still uses
 * the old PWM state, so the active-phase selection is stale). */
#define CURLOOP_BLANK_SKIP   3u
#define CURLOOP_DUTY_RATE    20.0f  /* max duty change per control cycle (%) */
#define CURLOOP_REF_RAMP_MS  1000u  /* soft-start: ramp ref to g_i_ref_ma over 1s */

/* Keil Watch: current setpoint (mA) */
volatile float g_i_ref_ma = 150.0f;

/* Cascade external current reference (written by the chain/speed loop) */
volatile float g_cur_ref_ext_ma = 0.0f;

/* J-Scope observability */
volatile float g_scope_i_ref  = 0.0f;
volatile float g_scope_i_fb   = 0.0f;
volatile float g_scope_i_duty = 0.0f;
volatile float g_scope_i_err  = 0.0f;
volatile float g_scope_i_ol   = 0.0f;   /* open-loop phase current estimate (mA) */
volatile uint32_t g_scope_i_dt_us = 0;    /* last current-loop dt (us) */

/* Feedback smoothing (Keil Watch tunable): 1.0 = no extra smoothing (window
 * only), 0.1 = heavy 1st-order low-pass on the windowed feedback. */
volatile float g_cur_fb_alpha = 1.0f;

/* Debug print interval (ms) for the [CURLOOP] log; 0 = off (Keil Watch tunable) */
volatile uint32_t g_cur_dbg_ms = 20;

/* Current-loop PI config (Keil Watch tunable) */
pid_config_t g_cur_pid_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = 0.1f,     /* % duty per mA error (lowered: feedback window +200us delay) */
    .ki           = 0.1f,     /* % duty per (mA*s) */
    .kd           = 0.0f,
    .output_min   = 2.0f,
    .output_max   = 98.0f,
    .integral_max = 500.0f,   /* mA*s */
    .i_term_max   = 20.0f,    /* I contribution clamped to +/-20% */
    .update_ms    = 0,        /* no throttle: run on every ADC sample (PWM rate) */
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
static uint8_t  s_last_step     = 0xFFu; /* last g_scope_step seen (edge blanking) */
static uint8_t  s_active        = 0;    /* current-loop activation latch */
static uint8_t  s_blank_skip    = 0;    /* commutation blanking: remaining skipped samples */
static float    s_fb_smooth     = 0.0f; /* 1st-order smoothed feedback */
static uint8_t  s_fb_smooth_init= 0;    /* smoothing primed from first full-window sample */
static volatile uint8_t  s_ref_ramp_active  = 0;
static float    s_ref_start        = 0.0f;
static volatile uint64_t s_ref_ramp_start_us = 0;
/* Cascade handshake (main loop -> ISR):
 *  engage: write g_cur_ref_ext_ma first, then SetExternalRef(true);
 *  exit:   SetExternalRef(false) first, then switch g_i_ref_ma.
 * g_cur_ref_ext_ma is a 32-bit float: single-instruction read/write on
 * Cortex-M4, so hardware cannot tear it.
 * PID state is NOT reset on source switch; CURLOOP_DUTY_RATE constrains
 * the duty transition. */
static volatile uint8_t s_use_ext_ref = 0;   /* 1 = use g_cur_ref_ext_ma, no ramp */

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
#if !MOTOR_LOOP_CURRENT_ENABLE
    (void)pData;   /* current loop disabled by topology (speed-only) */
    return;
#endif
    if (CommRunner_GetMode() != COMM_RUNNER_CURLOOP_FW &&
        CommRunner_GetMode() != COMM_RUNNER_CASCADE_FW) {
        /* mode not current-loop: reset state so restart starts clean */
        s_active = 0;
        s_ref_ramp_active = 0;
        s_use_ext_ref = 0;   /* cascade flag must not persist across mode/stop */
        s_last_us = 0;
        curloop_win_reset();
        return;
    }

    if (!CommRunner_CurLoopActive()) {
        /* phase 0 (timed open loop): estimate running current for display only */
        float fb_inst = (float)curloop_feedback(pData);
        s_ol_current_ma += (fb_inst - s_ol_current_ma) * 0.02f;   /* EMA, tau~1ms */
        g_scope_i_ol = s_ol_current_ma;
        s_active = 0;
        s_ref_ramp_active = 0;
        /* s_use_ext_ref is intentionally NOT cleared here: a cascade preset
         * made before phase 0 must survive until closed-loop entry. */
        s_last_us = 0;
        curloop_win_reset();
        return;
    }

    if (!s_active) {
        /* fresh activation (latched once): bumpless handoff - start duty from
         * the open-loop duty. Ref stays at the user-set g_i_ref_ma. The duty
         * rate limiter smooths the takeover. */
        s_active     = 1;
        s_last_duty  = CommRunner_GetDuty();
        PID_Reset(&s_pid);
        curloop_win_reset();
        s_last_step  = g_scope_step;
        s_ref_ramp_active   = 1;
        s_ref_ramp_start_us = 0;   /* latched on first PID run using real fb */
        s_fb_smooth_init = 0;   /* prime smoothing from first full-window sample */
        s_blank_skip     = 0;
    }

    /* Commutation edge blanking: on a step change flush the window and skip a
     * few samples so feedback never mixes old/new phase currents (the first PWM
     * cycle after an edge still drives the old state -> wrong-phase samples). */
    if (g_scope_step != s_last_step) {
        curloop_win_reset();
        s_last_step = g_scope_step;
        s_blank_skip = CURLOOP_BLANK_SKIP;
    }
    if (s_blank_skip > 0u) {
        s_blank_skip--;
        return;   /* post-edge blanking: no window fill / no control this cycle */
    }

    /* Sliding window average of the active-phase current (every ADC sample) */
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
    if (s_win_cnt < CURLOOP_WIN_SIZE) {
        return;   /* window filling / post-edge blanking: no control this cycle */
    }
    float fb = (float)s_win_sum / (float)CURLOOP_WIN_SIZE;
    float fb_raw = fb;   /* pre-smoothing windowed average (diagnostic) */

    /* Optional 1st-order smoothing (g_cur_fb_alpha tunable in Keil Watch).
     * Primed on the first full-window sample so there is no start-up transient. */
    if (s_fb_smooth_init) {
        float alpha = g_cur_fb_alpha;
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        s_fb_smooth += alpha * (fb - s_fb_smooth);
        fb = s_fb_smooth;
    } else {
        s_fb_smooth = fb;
        s_fb_smooth_init = 1;
    }

    Timer6_Timebase_UpdateTimestamp();
    uint64_t now = Timer6_Timebase_GetTimestamp();
    uint32_t dt_us;
    if (s_last_us == 0) {
        /* First call: use the nominal period derived from the ACTUAL PWM config,
         * so the log matches the real frequency without manual edits. */
        uint32_t f = CommRunner_GetPwmFreqHz();
        dt_us = (f != 0u) ? (1000000u / f) : 40u;
    } else {
        dt_us = (uint32_t)(now - s_last_us);
    }
    s_last_us = now;
    g_scope_i_dt_us = dt_us;

    /* Current setpoint: cascade mode uses external ref (no ramp); else soft-start ramp */
    float ref;
    if (s_use_ext_ref) {
        ref = g_cur_ref_ext_ma;
    } else {
        ref = g_i_ref_ma;
        if (s_ref_ramp_active) {
            if (s_ref_ramp_start_us == 0) {
                s_ref_start = fb;              /* anchor at actual open-loop current */
                s_ref_ramp_start_us = now;
            }
            uint64_t ramp_el = now - s_ref_ramp_start_us;
            uint64_t ramp_tot = (uint64_t)CURLOOP_REF_RAMP_MS * 1000UL;
            float ratio = (ramp_el >= ramp_tot) ? 1.0f : ((float)ramp_el / (float)ramp_tot);
            ref = s_ref_start + (g_i_ref_ma - s_ref_start) * ratio;
            if (ratio >= 1.0f) {
                s_ref_ramp_active = 0;
            }
        }
    }
    if (ref < 0.0f) {
        ref = 0.0f;   /* never command negative current */
    }

    float duty = PID_UpdateUs(&s_pid, ref, fb, dt_us);

    /* rate-limit duty: no 2%<->98% slam at handoff or during tuning */
    if (duty > s_last_duty + CURLOOP_DUTY_RATE) duty = s_last_duty + CURLOOP_DUTY_RATE;
    if (duty < s_last_duty - CURLOOP_DUTY_RATE) duty = s_last_duty - CURLOOP_DUTY_RATE;
    s_last_duty = duty;

    Commutation_SetActiveDuty(g_scope_step, duty);
    /* Keep runner s_duty in sync so the next Hall edge re-applies the same duty */
    CommRunner_SetDuty(duty);

    g_scope_i_ref  = ref;
    g_scope_i_fb   = fb;
    g_scope_i_duty = duty;
    g_scope_i_err  = ref - fb;

    {
        static uint32_t s_last_dbg = 0;
        uint32_t now_ms = (uint32_t)tickTimer_GetCount();
        uint32_t dbg_ms = g_cur_dbg_ms;
        if (dbg_ms > 0u && (now_ms - s_last_dbg) >= dbg_ms) {
            s_last_dbg = now_ms;
            MAIN_D("[CURLOOP] tus=%lu dt=%lu step=%u ref=%d fb=%d raw=%d err=%d duty=%d%% i=%d kp=%d ki=%d al=%d",
                   (unsigned long)(now & 0xFFFFFFFFul),
                   (unsigned long)dt_us,
                   (unsigned)g_scope_step,
                   (int)ref, (int)fb, (int)fb_raw, (int)(ref - fb),
                   (int)(duty * 10) / 10, (int)s_pid.i_term,
                   (int)(g_cur_pid_cfg.kp * 1000.0f),
                   (int)(g_cur_pid_cfg.ki * 1000.0f),
                   (int)(g_cur_fb_alpha * 100.0f));
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
    s_use_ext_ref = 0;   /* never start with a stale cascade reference */
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

void CurLoop_SetExternalRef(bool enable)
{
    /* Source switch: drop any stale soft-start ramp so a later return to
     * g_i_ref_ma re-anchors from actual feedback. Clear s_ref_ramp_active
     * first (it is the ISR's guard for reading the 64-bit timestamp), then
     * the timestamp, and switch s_use_ext_ref last. */
    s_ref_ramp_active = 0;
    s_ref_ramp_start_us = 0;
    s_use_ext_ref = enable ? 1 : 0;
}
