/**
 *******************************************************************************
 * @file  foc.c
 * @brief FOC runner (open-loop mode 21 + current-loop mode 22).
 *
 *        ISR frequency assumption:
 *          I.c uses the INMOP-style double trigger (SCMP0 @ PEAK + SCMP2 @
 *          VALLEY) -> 10 kHz PWM produces 20 kHz EOCB ISR. Foc_Isr therefore
 *          runs at FOC_ISR_HZ = 20000 (motor_config.h).
 *
 *        Mode 21 (open loop): theta integrates g_foc_openloop_freq_hz each ISR
 *        and the voltage vector (g_foc_openloop_volt_v) is transformed by
 *        SVPWM into U/V/W duty, written through TMR4_PWM_SetDuty3Phase().
 *
 *        Mode 22 (current loop, I-F start):
 *          Foc_StartCurrentLoop() -> IF_START runs the current loop from the
 *          first tick with a synthetic angle (I-F start):
 *            - frequency ramps 0 -> g_foc_openloop_freq_hz
 *            - Iq ramps 0 -> g_foc_iq_ref_cmd_ma (FOC_IQ_REF_MA = 2000 mA)
 *            - voltage envelope ramps 0 -> g_foc_vmax_v
 *          Current is controlled from tick 1, so it never exceeds the Iq
 *          target (no open-loop 3A surge, no sensor clipping, no heat).
 *          When the encoder electrical angle tracks the synthetic angle
 *          (sync window), Foc_IfHandover() smoothly switches to the encoder
 *          angle and RUN continues with encoder-angle FOC:
 *            ia/ib/ic (A) -> Foc_Clarke -> Foc_Park(theta) -> id/iq
 *            -> vd = PI_id(0, id), vq = PI_iq(iq_ref, iq)   (dev_pid)
 *            -> Foc_InvPark(vd, vq, theta) -> Foc_Svpwm -> TMR4 duty
 *
 *        ISR constraint: short, no blocking, no prints, no malloc.
 *******************************************************************************
 */

#include "foc.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"
#include "dev_pid.h"
#include <math.h>
#include "SEGGER_RTT.h"
#include <stdio.h>

/*******************************************************************************
 * Global variables for JScope / Keil Watch
 ******************************************************************************/
volatile float   g_foc_theta_rad        = 0.0f;
volatile float   g_foc_du               = 50.0f;
volatile float   g_foc_dv               = 50.0f;
volatile float   g_foc_dw               = 50.0f;
volatile float   g_foc_valpha           = 0.0f;
volatile float   g_foc_vbeta            = 0.0f;
volatile uint8_t g_foc_active           = 0u;
volatile float   g_foc_openloop_freq_hz = FOC_OPENLOOP_FREQ_HZ;
volatile float   g_foc_openloop_volt_v  = FOC_OPENLOOP_VOLT_V;

/* Current-loop observables / tuning (all volatile, Keil Watch editable) */
volatile uint8_t g_foc_mode             = FOC_MODE_NONE;
volatile int32_t g_foc_cur_sign         = (int32_t)FOC_CUR_SIGN;
volatile int32_t g_foc_enc_dir         = (int32_t)FOC_ENC_DIR;  /* encoder direction, Watch tunable */
volatile int32_t g_motor_scope        = (int32_t)MOTOR_SCOPE_KEY;   /* MotorScope RTT 总开关：0=关 1=开（Watch 可改） */
volatile int32_t g_foc_pi_off_180      = (int32_t)FOC_PI_OFF_180;   /* 1 = +180deg control angle (flip torque), Watch tunable */
volatile float   g_foc_iq_ref_cmd_ma    = (float)FOC_IQ_REF_MA;
volatile float   g_foc_iq_ref_ma        = 0.0f;
volatile float   g_foc_id_ma            = 0.0f;
volatile float   g_foc_iq_ma            = 0.0f;
volatile float   g_foc_vd               = 0.0f;
volatile float   g_foc_vq               = 0.0f;
volatile uint8_t g_foc_align_state      = 0u;
volatile uint8_t g_foc_fault            = 0u;

/* Over-current limit (Watch tunable) + fault diagnostic */
volatile float g_foc_oc_limit_a    = (float)FOC_OC_LIMIT_A;
volatile float g_foc_fault_i_ma    = 0.0f;   /* |phase current| at OC trip (mA) */
/* Startup substage + OC trip diagnostics (latched in Foc_OverCurrent) */
volatile uint8_t g_foc_phase       = 0u;   /* 0=idle 1=hold 2=ramp/sync 3=run 4=align */
volatile uint8_t g_foc_fault_stage = 0u;   /* g_foc_phase at OC trip */
volatile int16_t g_foc_fault_iu_ma = 0;    /* phase currents at OC trip (mA) */
volatile int16_t g_foc_fault_iv_ma = 0;
volatile int16_t g_foc_fault_iw_ma = 0;

/* Voltage envelope / feedback filter (all Watch tunable) */
volatile float g_foc_vmax_v       = (float)FOC_VMAX_V;        /* current-loop max |v| (V) */
volatile float g_foc_vramp_v_s    = (float)FOC_VRAMP_V_S;     /* voltage envelope ramp (V/s) */
volatile float g_foc_cur_fb_alpha = FOC_CUR_FB_ALPHA;         /* EMA weight on id/iq (1.0=off) */
volatile float g_foc_iq_ramp_ma_s = (float)FOC_IQ_RAMP_MA_S;  /* Iq soft-start ramp (mA/s) */
volatile float g_foc_vlim_v       = 0.0f;                     /* current voltage envelope (V) */
/* RUN speed control (mode22 after handover): speed PI via g_foc_pid_spd_cfg.
 * RUN is a torque loop with no speed loop -> without it the rotor would
 * accelerate to the voltage limit ~1000rpm and trip OC. */
volatile float g_foc_run_target_rpm = -30.0f;
volatile float g_foc_iq_pi_ma        = 0.0f;   /* actual RUN Iq reference from speed PI (mA) */
volatile float g_foc_anchor_deg      = (float)FOC_RUN_ANCHOR_DEG;   /* extra angle (deg) added to phi_hold: sweep 0/90/180/270
                                                * with g_foc_run_iq_sign to find the correct RUN frame;
                                                * correct frame -> spd settles at -30, no reversal */
volatile float g_foc_run_iq_sign     = (float)FOC_RUN_IQ_SIGN;  /* RUN Iq sign: -1 because field-oriented RUN + cur_sign=-1
                                                * makes positive Iq produce POSITIVE-direction torque (motor
                                                * reversed); flip to +1 if direction comes out wrong */

/* I-F start observables */
volatile float   g_foc_if_freq_hz  = 0.0f;   /* current I-F electrical frequency (Hz) */
volatile float   g_foc_if_diff_rad = 0.0f;   /* encoder-elec angle - synthetic angle (rad) */
volatile float   g_foc_if_sweep_cHz = 0.0f;  /* live diff sweep rate (cHz), 100ms window */
volatile float   g_foc_if_hold_iq_ma = (float)FOC_IF_HOLD_IQ_MA; /* hold threshold (mA), Watch tunable */
volatile float   g_foc_if_sync_band_rad = FOC_IF_SYNC_BAND_RAD;   /* sync window band (rad), Watch tunable */
volatile uint32_t g_foc_if_sync_win_cnt = FOC_IF_SYNC_WIN_CNT;    /* sync window length (samples @20k), Watch tunable */
volatile uint32_t g_foc_if_sync_good_wins = FOC_IF_SYNC_GOOD_WINS;/* consecutive good windows required, Watch tunable */
volatile float   g_foc_if_lock_diff_rad = 0.0f;                   /* lock offset latched while aligned in hold (rad) */
volatile float   g_foc_if_rotor_rad   = 0.0f;   /* rotor electrical angle, folded [0,2PI) (rad) */
volatile float   g_foc_if_rel_diff_rad = 0.0f;  /* unwrapped diff relative to lock offset (rad) */
volatile float   g_foc_if_win_min_rad  = 0.0f;  /* current sync-window min of rel diff (rad) */
volatile float   g_foc_if_win_max_rad  = 0.0f;  /* current sync-window max of rel diff (rad) */
volatile uint32_t g_foc_if_win_cnt     = 0u;    /* samples collected in current window */
volatile uint32_t g_foc_if_good_cnt    = 0u;    /* consecutive good windows so far */
volatile uint8_t g_foc_if_sync     = 0u;     /* 1 = rotor synchronized, handed over to encoder */
/* I-F start events for main-loop printing (ISR only sets flag+payload) */
volatile uint8_t  g_foc_if_evt    = 0u;   /* 1=hold done 2=handover 3=timeout */
volatile int32_t  g_foc_if_evt_v1 = 0;
volatile int32_t  g_foc_if_evt_v2 = 0;
volatile int32_t  g_foc_if_evt_v3 = 0;
volatile int32_t  g_foc_if_evt_v4 = 0;
/* Align calibration (mode 23) */
volatile float   g_foc_align_volt_v = FOC_ALIGN_VOLT_V;        /* fixed align voltage (V), Watch tunable */
volatile int32_t g_foc_align_offset = 0;                        /* recorded encoder electrical-zero count */
/* Align calibration events for main-loop MAIN_D printing (ISR only sets flag+payload) */
volatile uint8_t  g_foc_align_evt    = 0u;   /* 1=start 2=beta done 3=locked 4=done 5=fault */
volatile int32_t  g_foc_align_evt_v1 = 0;
volatile int32_t  g_foc_align_evt_v2 = 0;
volatile int32_t  g_foc_align_evt_v3 = 0;

/* Current-loop PI configs (volatile, Keil Watch can tune kp/ki live).
 * INMOP-style position PI: Kp = FOC_PI_KP (V/A), Ki = FOC_PI_KI (per-second
 * integral gain), output clamped to +/-FOC_PI_UMAX_V (6 V on 12 V bus).
 * update_ms = 0 -> PID_UpdateUs runs on every 20 kHz ISR. */
pid_config_t g_foc_pid_id_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = FOC_PI_KP,
    .ki           = FOC_PI_KI,
    .kd           = 0.0f,
    .output_min   = -FOC_PI_UMAX_V,
    .output_max   =  FOC_PI_UMAX_V,
    .integral_max = 0.5f,     /* A*s, scaled for low-R motor */
    .i_term_max   = 0.0f,     /* disabled: bounded by integral_max + output clamp */
    .update_ms    = 0,        /* no throttle: every ISR */
};

pid_config_t g_foc_pid_iq_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = FOC_PI_KP,
    .ki           = FOC_PI_KI,
    .kd           = 0.0f,
    .output_min   = -FOC_PI_UMAX_V,
    .output_max   =  FOC_PI_UMAX_V,
    .integral_max = 0.5f,     /* A*s, scaled for low-R motor */
    .i_term_max   = 0.0f,     /* disabled: bounded by integral_max + output clamp */
    .update_ms    = 0,        /* no throttle: every ISR */
};

/* RUN speed PI: output = Iq reference (mA), error = actual_rpm - target_rpm,
 * so a rotor faster than target drives the output to 0 (fast brake). */
pid_config_t g_foc_pid_spd_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = 50.0f,    /* mA/rpm */
    .ki           = 10.0f,    /* mA/(rpm*s) */
    .kd           = 0.0f,
    .output_min   = 0.0f,     /* positive Iq only (torque direction fixed by pi_off) */
    .output_max   = (float)FOC_IQ_REF_MA,
    .integral_max = 50.0f,    /* rpm*s: limit windup */
    .i_term_max   = 0.0f,
    .update_ms    = 0,
};

/* Local state */
static uint8_t s_bInited = 0u;

/* Current-loop state machine (mirrored to g_foc_align_state for Watch) */
typedef enum {
    FOC_STATE_IDLE     = 0,
    FOC_STATE_IF_START = 1,   /* I-F current-controlled start (synthetic angle) */
    FOC_STATE_RUN      = 2,   /* encoder-angle FOC current loop */
    FOC_STATE_ALIGN    = 3,   /* standstill electrical alignment (mode 23) */
} foc_state_t;

static foc_state_t s_state        = FOC_STATE_IDLE;
static int32_t     s_align_offset = 0;
static float       s_vlim         = 0.0f;   /* current voltage envelope (V) */
static float       s_id_f         = 0.0f;   /* EMA-filtered d current (A) */
static float       s_iq_f         = 0.0f;   /* EMA-filtered q current (A) */

/* I-F sync detection (sliding window on angle difference) */
static float    s_if_diff_min   = 0.0f;
static float    s_if_diff_max   = 0.0f;
static uint32_t s_if_win_cnt    = 0u;
static uint32_t s_if_hold_tick  = 0u;
static uint8_t  s_if_hold_done  = 0u;
static float    s_if_last_diff = 0.0f;
static uint32_t s_if_wrap_cnt  = 0u;
static uint32_t s_if_sweep_last_wrap = 0u;
static uint32_t s_if_sweep_last_tick = 0u;
static float    s_if_diff_unwrapped = 0.0f;   /* continuous diff for sync band (no +/-pi wrap) */
static uint8_t  s_run_blend_cnt  = 0u;        /* handover angle blend progress */
static float    s_run_blend_from = 0.0f;      /* synthetic angle at handover (blend start) */
static uint8_t  s_if_diff_unwrapped_valid = 0u;
static uint32_t s_if_good_wins  = 0u;
static uint32_t s_if_tick       = 0u;
static int32_t  s_align_last_cnt   = 0;
static uint32_t s_align_stable_cnt = 0u;
static uint32_t s_align_hold_cnt   = 0u;
static uint8_t  s_align_phase      = 0u;   /* 0=beta(90deg), 1=alpha(0deg) */
static uint32_t s_align_phase_tick = 0u;

/* PI runtime states (bound to the Watch-tunable configs) */
static pid_state_t s_pid_id;
static pid_state_t s_pid_iq;
static pid_state_t s_pid_spd;

/* ISR period in us (FOC_ISR_HZ = 20000 -> 50 us) */
#define FOC_ISR_DT_US  (1000000u / FOC_ISR_HZ)
#define FOC_IF_HOLD_MAX_CNT ((uint32_t)FOC_IF_HOLD_MAX_MS * FOC_ISR_HZ / 1000u)
/* Handover angle blend: rotate the RUN control frame from the synthetic I-F
 * angle to the true rotor angle over this many ISRs (200 = 10 ms @20 kHz).
 * Avoids the current kick when the frame jumps by the I-F load angle. */
#define FOC_RUN_BLEND_CNT    200u

/* OC debounce: require N consecutive over-limit samples (N x 50us) before trip */
#define FOC_OC_DEBOUNCE_SAMPLES  4u
static uint16_t s_oc_cnt = 0u;

/*******************************************************************************
 * Local helpers
 ******************************************************************************/

/* Safe positive modulo: ((x % N) + N) % N */
static int32_t Foc_ModPos(int32_t x, int32_t n)
{
    return ((x % n) + n) % n;
}

/* Hard cap on the open-loop phase voltage (overheat protection). */
static void Foc_ClampOpenLoopVolt(void)
{
    if (g_foc_openloop_volt_v > FOC_OPENLOOP_VOLT_MAX) {
        g_foc_openloop_volt_v = FOC_OPENLOOP_VOLT_MAX;
    }
}

/* Over-current check: any phase |I| > g_foc_oc_limit_a (mA conversion). */
static uint8_t Foc_OverCurrent(const stc_i_data_t *pData)
{
    float iu, iv, iw, imax;

    if (pData == NULL) {
        return 0u;
    }

    iu  = (float)pData->i16IU_mA;
    iv  = (float)pData->i16IV_mA;
    iw  = (float)pData->i16IW_mA;

    /* max |phase current| (mA) */
    imax = (iu > iv) ? iu : iv;
    if (iw > imax) imax = iw;
    {
        float imin = (iu < iv) ? iu : iv;
        if (iw < imin) imin = iw;
        if (-imin > imax) imax = -imin;
    }

    if (imax > g_foc_oc_limit_a * 1000.0f) {
        if (++s_oc_cnt >= FOC_OC_DEBOUNCE_SAMPLES) {
            s_oc_cnt        = 0u;
            g_foc_fault_i_ma = imax;   /* diagnostic: current that tripped OC */
            g_foc_fault_stage = g_foc_phase;
            if (pData != NULL) {
                g_foc_fault_iu_ma = pData->i16IU_mA;
                g_foc_fault_iv_ma = pData->i16IV_mA;
                g_foc_fault_iw_ma = pData->i16IW_mA;
            }
            return 1u;
        }
    } else {
        s_oc_cnt = 0u;
    }
    return 0u;
}

/* Fault stop: latch fault code (1=OC, 2=start timeout), disable output. */
static void Foc_FaultStop(uint8_t u8Code)
{
    g_foc_fault       = u8Code;
    g_foc_active      = 0u;
    s_state           = FOC_STATE_IDLE;
    g_foc_align_state = 0u;
    s_oc_cnt          = 0u;
    TMR4_PWM_EmergencyStop();
    g_foc_du = 0.0f;
    g_foc_dv = 0.0f;
    g_foc_dw = 0.0f;
}

/* Electrical angle from the aligned encoder (RUN phase). */
static float Foc_CurLoopTheta(void)
{
    int32_t diff = Foc_ModPos(((int32_t)g_enc_count * (int32_t)g_foc_enc_dir) - s_align_offset,
                              (int32_t)ENCODER_CPR);

    return (float)diff * (FOC_MATH_2PI / (float)ENCODER_CPR)
         * (float)FOC_POLE_PAIRS;
}

/* dq currents for an arbitrary control angle theta (A). */
static void Foc_GetDq(const stc_i_data_t *pData, float theta, float *id, float *iq)
{
    float ia, ib, ic, ialpha, ibeta;
    float sign = (float)g_foc_cur_sign;

    if (pData != NULL) {
        ia = (float)pData->i16IU_mA * 0.001f * sign;
        ib = (float)pData->i16IV_mA * 0.001f * sign;
        ic = (float)pData->i16IW_mA * 0.001f * sign;
    } else {
        ia = ib = ic = 0.0f;
    }
    Foc_Clarke(ia, ib, ic, &ialpha, &ibeta);
    Foc_Park(ialpha, ibeta, theta, id, iq);
}

/* EMA on dq feedback (g_foc_cur_fb_alpha: 1.0 = off, lower = smoother). */
static void Foc_EmaFilter(float *id, float *iq)
{
    float alpha = g_foc_cur_fb_alpha;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    s_id_f += alpha * (*id - s_id_f);
    s_iq_f += alpha * (*iq - s_iq_f);
    *id = s_id_f;
    *iq = s_iq_f;
}

/* Voltage envelope: ramp allowed |v| from 0 up to g_foc_vmax_v, clamp mag. */
static void Foc_ApplyVoltageEnvelope(float *vd, float *vq)
{
    s_vlim += g_foc_vramp_v_s / (float)FOC_ISR_HZ;
    if (s_vlim > g_foc_vmax_v) {
        s_vlim = g_foc_vmax_v;
    }
    g_foc_vlim_v = s_vlim;
    {
        float v2    = (*vd) * (*vd) + (*vq) * (*vq);
        float vmax2 = s_vlim * s_vlim;
        if (v2 > vmax2) {
            float k = s_vlim / sqrtf(v2);
            *vd *= k;
            *vq *= k;
        }
    }
}

/*******************************************************************************
 * Foc_Init - math LUT + register ISR callback (no output)
 ******************************************************************************/
void Foc_Init(void)
{
    if (s_bInited) {
        return;
    }

    Foc_Math_Init();
    PID_Init(&s_pid_id, &g_foc_pid_id_cfg);
    PID_Init(&s_pid_iq, &g_foc_pid_iq_cfg);
    PID_Init(&s_pid_spd, &g_foc_pid_spd_cfg);
    I_RegisterFocCallback(Foc_Isr);
    s_bInited = 1u;
}

/*******************************************************************************
 * Foc_StartOpenLoop - mode 21: voltage open loop (unchanged)
 ******************************************************************************/
void Foc_StartOpenLoop(void)
{
    g_foc_theta_rad   = 0.0f;
    g_foc_fault       = 0u;
    g_foc_fault_i_ma  = 0.0f;
    s_oc_cnt          = 0u;
    if (g_foc_openloop_volt_v <= 0.0f) {
        g_foc_openloop_volt_v = FOC_OPENLOOP_VOLT_V;   /* keep Watch-tuned value; recover default if zeroed */
    }
    g_foc_mode        = FOC_MODE_OPENLOOP;
    g_foc_phase       = 0u;
    s_state           = FOC_STATE_IDLE;
    g_foc_align_state = 0u;

    /* Reconfigure all 3 channels to complementary (dead-timer) PWM. */
    TMR4_PWM_SetFocMode(FOC_DEADTIME_NS);

    /* Neutral 50% duty before enabling output */
    g_foc_du = 50.0f;
    g_foc_dv = 50.0f;
    g_foc_dw = 50.0f;
    TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);

    TMR4_PWM_StartOutput();
    g_foc_active = 1u;
}

/*******************************************************************************
 * Foc_StartCurrentLoop - mode 22: I-F current-controlled start
 ******************************************************************************/
void Foc_StartCurrentLoop(void)
{
    g_foc_fault       = 0u;
    g_foc_fault_i_ma  = 0.0f;
    s_oc_cnt          = 0u;
    g_foc_mode        = FOC_MODE_CURLOOP;
    g_foc_phase       = 1u;   /* hold */
    g_foc_theta_rad   = 0.0f;
    g_foc_iq_ref_ma   = 0.0f;      /* ramps to g_foc_iq_ref_cmd_ma (FOC_IQ_REF_MA = 3000 mA) */
    g_foc_id_ma       = 0.0f;
    g_foc_iq_ma       = 0.0f;
    g_foc_vd          = 0.0f;
    g_foc_vq          = 0.0f;
    g_foc_if_freq_hz  = 0.0f;
    g_foc_if_diff_rad = 0.0f;
    g_foc_if_sync     = 0u;

    s_state           = FOC_STATE_IF_START;
    s_if_tick         = 0u;
    s_if_hold_tick    = 0u;
    s_if_hold_done    = 0u;
    s_if_last_diff    = 0.0f;
    s_if_wrap_cnt     = 0u;
    g_foc_if_evt      = 0u;
    g_foc_if_evt_v4   = 0;
    g_foc_if_sweep_cHz = 0.0f;
    s_if_sweep_last_wrap = 0u;
    s_if_sweep_last_tick = 0u;
    s_if_diff_unwrapped = 0.0f;
    s_if_diff_unwrapped_valid = 0u;
    g_foc_if_lock_diff_rad = 0.0f;
    g_foc_if_rotor_rad   = 0.0f;
    g_foc_if_rel_diff_rad = 0.0f;
    g_foc_if_win_min_rad  = 0.0f;
    g_foc_if_win_max_rad  = 0.0f;
    g_foc_if_win_cnt      = 0u;
    g_foc_if_good_cnt     = 0u;
    s_if_win_cnt      = 0u;
    s_if_good_wins    = 0u;
    s_if_diff_min     = 0.0f;
    s_if_diff_max     = 0.0f;
    s_vlim            = 0.0f;
    s_id_f            = 0.0f;
    s_iq_f            = 0.0f;
    g_foc_align_state = 1u;

    /* Fresh PI state */
    PID_Reset(&s_pid_id);
    PID_Reset(&s_pid_iq);
    PID_Reset(&s_pid_spd);

    /* Complementary PWM, neutral 50% before enabling output */
    TMR4_PWM_SetFocMode(FOC_DEADTIME_NS);
    g_foc_du = 50.0f;
    g_foc_dv = 50.0f;
    g_foc_dw = 50.0f;
    TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);

    TMR4_PWM_StartOutput();
    g_foc_active = 1u;
}

/*******************************************************************************
 * Foc_StartAlign - mode 23: standstill electrical alignment (INMOP-style)
 *
 *   Locks the rotor to the d-axis (theta = 0) with a small d-axis current
 *   (current-controlled, safe for the +-10A sensor), waits until the encoder
 *   count is stable, then records the encoder count as the electrical zero
 *   (s_align_offset so Foc_CurLoopTheta() == 0). Holds briefly and then
 *   releases the output automatically.
 ******************************************************************************/
void Foc_StartAlign(void)
{
    g_foc_fault       = 0u;
    g_foc_fault_i_ma  = 0.0f;
    s_oc_cnt          = 0u;
    g_foc_mode        = FOC_MODE_ALIGN;
    g_foc_phase       = 4u;   /* align */
    g_foc_theta_rad   = 0.0f;
    g_foc_id_ma       = 0.0f;
    g_foc_iq_ma       = 0.0f;
    g_foc_vd          = 0.0f;
    g_foc_vq          = 0.0f;

    s_state            = FOC_STATE_ALIGN;
    s_align_phase      = 0u;                 /* step1: beta axis */
    s_align_phase_tick = 0u;
    s_align_last_cnt   = (int32_t)g_enc_count;
    s_align_stable_cnt = 0u;
    s_align_hold_cnt   = 0u;
    s_vlim             = 0.0f;
    s_id_f             = 0.0f;
    s_iq_f             = 0.0f;
    g_foc_align_state  = 1u;   /* aligning */

    PID_Reset(&s_pid_id);
    PID_Reset(&s_pid_iq);
    PID_Reset(&s_pid_spd);

    g_foc_align_evt    = 1u;
    g_foc_align_evt_v1 = (int32_t)(g_foc_align_volt_v * 1000.0f);   /* mV */
    g_foc_align_evt_v2 = 0;
    g_foc_align_evt_v3 = 0;

    TMR4_PWM_SetFocMode(FOC_DEADTIME_NS);
    g_foc_du = 50.0f;
    g_foc_dv = 50.0f;
    g_foc_dw = 50.0f;
    TMR4_PWM_SetDuty3Phase(50.0f, 50.0f, 50.0f);
    TMR4_PWM_StartOutput();
    g_foc_active = 1u;
}

#define FOC_ALIGN_BETA_CNT     ((uint32_t)FOC_ALIGN_BETA_MS * FOC_ISR_HZ / 1000u)
#define FOC_ALIGN_STABLE_CNT  ((uint32_t)FOC_ALIGN_STABLE_MS * FOC_ISR_HZ / 1000u)
#define FOC_ALIGN_TIMEOUT_CNT ((uint32_t)FOC_ALIGN_TIMEOUT_MS * FOC_ISR_HZ / 1000u)
#define FOC_ALIGN_HOLD_CNT    ((uint32_t)FOC_ALIGN_HOLD_MS * FOC_ISR_HZ / 1000u)

/* ALIGN step: INMOP-style two-step FIXED-voltage positioning (does not depend
 * on the current feedback, so the recorded zero is valid by construction).
 *   step1 (beta):  fixed voltage along theta=90 deg for FOC_ALIGN_BETA_MS.
 *   step2 (alpha): fixed voltage along theta=0 deg; once the encoder count is
 *                  stable for FOC_ALIGN_STABLE_MS the rotor is locked on the
 *                  d-axis and the encoder count is recorded as the zero.
 * Current = V/R (0.4V / 0.1ohm -> ~4A worst case, measured ~2.5A including
 * driver/wire resistance; well within OC 5.5A). OC still protects.
 * The measured id/iq during step2 reveal the current-mapping rotation. */
static void Foc_AlignStep(const stc_i_data_t *pData)
{
    float theta = (s_align_phase == 0u) ? FOC_MATH_HALF_PI : 0.0f;
    float valpha, vbeta, du, dv, dw;
    float id, iq;
    int32_t cnt;

    if (Foc_OverCurrent(pData)) {
        g_foc_align_evt    = 5u;
        g_foc_align_evt_v1 = 1;                      /* code 1 = over-current */
        g_foc_align_evt_v2 = (int32_t)g_foc_fault_i_ma;
        Foc_FaultStop(1u);
        return;
    }

    /* fixed voltage along the d-axis at current theta */
    valpha = g_foc_align_volt_v * Foc_Math_Cos(theta);
    vbeta  = g_foc_align_volt_v * Foc_Math_Sin(theta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);
    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_vd = valpha;              /* applied dq voltage (in theta frame) */
    g_foc_vq = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;

    /* current feedback (info only) */
    Foc_GetDq(pData, theta, &id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    s_align_phase_tick++;

    /* step1: beta axis hold */
    if (s_align_phase == 0u) {
        if (s_align_phase_tick >= FOC_ALIGN_BETA_CNT) {
            s_align_phase      = 1u;
            s_align_phase_tick = 0u;
            s_align_last_cnt   = (int32_t)g_enc_count;
            s_align_stable_cnt = 0u;
            g_foc_align_state  = 1u;
            g_foc_align_evt    = 2u;
            g_foc_align_evt_v1 = 0;
            g_foc_align_evt_v2 = 0;
            g_foc_align_evt_v3 = 0;
        }
        return;
    }

    /* step2: alpha axis, wait for the rotor to settle */
    if (g_foc_align_state == 1u) {
        cnt = (int32_t)g_enc_count;
        if (cnt == s_align_last_cnt) {
            if (++s_align_stable_cnt >= FOC_ALIGN_STABLE_CNT) {
                s_align_offset     = Foc_ModPos((int32_t)g_enc_count * (int32_t)g_foc_enc_dir,
                                                (int32_t)ENCODER_CPR);  /* electrical zero, enc_dir-scaled */
                g_foc_align_offset = s_align_offset;
                g_foc_align_state  = 2u;
                s_align_hold_cnt   = 0u;
                g_foc_align_evt    = 3u;
                g_foc_align_evt_v1 = s_align_offset;
                g_foc_align_evt_v2 = (int32_t)g_foc_id_ma;
                g_foc_align_evt_v3 = (int32_t)g_foc_iq_ma;
            }
        } else {
            s_align_last_cnt   = cnt;
            s_align_stable_cnt = 0u;
        }
        if (s_align_phase_tick >= FOC_ALIGN_TIMEOUT_CNT) {
            g_foc_align_evt    = 5u;
            g_foc_align_evt_v1 = 3;                      /* code 3 = align timeout */
            g_foc_align_evt_v2 = (int32_t)g_foc_id_ma;
            g_foc_align_evt_v3 = (int32_t)g_foc_iq_ma;
            Foc_FaultStop(3u);
            return;
        }
    } else if (g_foc_align_state == 2u) {
        /* hold the lock, then release */
        if (++s_align_hold_cnt >= FOC_ALIGN_HOLD_CNT) {
            g_foc_align_state = 3u;
            g_foc_active      = 0u;
            TMR4_PWM_EmergencyStop();
            g_foc_du = 0.0f;
            g_foc_dv = 0.0f;
            g_foc_dw = 0.0f;
            s_state  = FOC_STATE_IDLE;
            g_foc_align_evt    = 4u;
            g_foc_align_evt_v1 = s_align_offset;
            g_foc_align_evt_v2 = (int32_t)(g_foc_valpha * 1000.0f);
            g_foc_align_evt_v3 = (int32_t)(g_foc_vbeta * 1000.0f);
        }
    }
}
/*******************************************************************************
 * Foc_Stop - disable FOC output
 ******************************************************************************/
void Foc_Stop(void)
{
    g_foc_active      = 0u;
    g_foc_mode        = FOC_MODE_NONE;
    g_foc_phase       = 0u;
    s_state           = FOC_STATE_IDLE;
    g_foc_align_state = 0u;
    TMR4_PWM_EmergencyStop();
    g_foc_du = 0.0f;
    g_foc_dv = 0.0f;
    g_foc_dw = 0.0f;
    g_foc_valpha = 0.0f;
    g_foc_vbeta  = 0.0f;
    s_vlim       = 0.0f;
    g_foc_vlim_v = 0.0f;
}

/*******************************************************************************
 * I-F start step
 ******************************************************************************/
static void Foc_IfHandover(const stc_i_data_t *pData);   /* forward decl */

/* IF_START: current loop with synthetic angle + rotor sync detection. */
static void Foc_IfStartStep(const stc_i_data_t *pData)
{
    float theta, ctrl_theta, id, iq, iq_ref_a, vd, vq, valpha, vbeta, du, dv, dw;
    float enc_elec, diff;

    if (Foc_OverCurrent(pData)) {
        Foc_FaultStop(1u);
        return;
    }

    g_foc_phase = (s_if_hold_done != 0u) ? 2u : 1u;   /* 1=hold, 2=ramp/sync */

    /* 0) initial hold: keep theta=0 and freq=0 until the Iq reference has
     *    ramped up enough to lock the rotor onto the initial current vector,
     *    so the field always starts rotating from a synchronized state. */
    if ((g_foc_iq_ref_ma < g_foc_if_hold_iq_ma) &&
        (s_if_hold_tick < FOC_IF_HOLD_MAX_CNT)) {
        s_if_hold_tick++;
        theta = g_foc_theta_rad;      /* stays 0 during hold */
    } else {
        if (!s_if_hold_done) {
            s_if_hold_done = 1u;
            g_foc_if_evt    = 1u;   /* hold done -> freq ramp starts */
            g_foc_if_evt_v1 = (int32_t)(g_foc_if_freq_hz * 100.0f);
            g_foc_if_evt_v2 = (int32_t)g_foc_iq_ref_ma;
            g_foc_if_evt_v3 = 0;
            /* Rotor is aligned with the hold current vector, so its d-axis is at
             * phi_hold = ctrl_theta + 90deg = pi/2 + pi_off*pi. Anchor the RUN
             * frame here: Foc_CurLoopTheta() then returns the TRUE rotor angle
             * (field-oriented, direction fixed, no dependence on encoder Z). */
            {
                float phi_hold = FOC_MATH_HALF_PI
                               + ((g_foc_pi_off_180 != 0) ? FOC_MATH_PI : 0.0f)
                               + (g_foc_anchor_deg * FOC_MATH_PI / 180.0f);
                float per_cnt = FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR;
                s_align_offset = Foc_ModPos(((int32_t)g_enc_count * (int32_t)g_foc_enc_dir)
                                            - (int32_t)(phi_hold / per_cnt),
                                            (int32_t)ENCODER_CPR);
                g_foc_align_offset = s_align_offset;
            }
        }
        /* 1) frequency ramp 0 -> g_foc_openloop_freq_hz */
        if (g_foc_if_freq_hz < g_foc_openloop_freq_hz) {
        g_foc_if_freq_hz += FOC_IF_FREQ_RAMP_HZ_S / (float)FOC_ISR_HZ;
        if (g_foc_if_freq_hz > g_foc_openloop_freq_hz) {
            g_foc_if_freq_hz = g_foc_openloop_freq_hz;
        }
    }

    /* 2) synthetic angle integration (fold to [0, 2PI)) */
    theta = g_foc_theta_rad
          + (FOC_MATH_2PI * g_foc_if_freq_hz / (float)FOC_ISR_HZ);
    if (theta >= FOC_MATH_2PI) {
        theta -= FOC_MATH_2PI;
    }
    g_foc_theta_rad = theta;
    }

    /* control frame may be +180deg flipped (torque direction) */
    ctrl_theta = theta + ((g_foc_pi_off_180 != 0) ? FOC_MATH_PI : 0.0f);

    /* 3) currents + EMA */
    Foc_GetDq(pData, ctrl_theta, &id, &iq);
    Foc_EmaFilter(&id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    /* 4) Iq soft ramp 0 -> g_foc_iq_ref_cmd_ma (FOC_IQ_REF_MA = 2000 mA) */
    {
        float step = g_foc_iq_ramp_ma_s / (float)FOC_ISR_HZ;
        if (g_foc_iq_ref_ma < g_foc_iq_ref_cmd_ma) {
            g_foc_iq_ref_ma += step;
            if (g_foc_iq_ref_ma > g_foc_iq_ref_cmd_ma) {
                g_foc_iq_ref_ma = g_foc_iq_ref_cmd_ma;
            }
        } else if (g_foc_iq_ref_ma > g_foc_iq_ref_cmd_ma) {
            g_foc_iq_ref_ma -= step;
            if (g_foc_iq_ref_ma < g_foc_iq_ref_cmd_ma) {
                g_foc_iq_ref_ma = g_foc_iq_ref_cmd_ma;
            }
        }
    }

    /* 5) PI: id -> 0, iq -> ramped reference */
    iq_ref_a = g_foc_iq_ref_ma * 0.001f;
    vd = PID_UpdateUs(&s_pid_id, 0.0f,     id, FOC_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, iq_ref_a, iq, FOC_ISR_DT_US);
    g_foc_vd = vd;
    g_foc_vq = vq;

    /* 6) voltage envelope 0 -> g_foc_vmax_v */
    Foc_ApplyVoltageEnvelope(&vd, &vq);

    /* 7) inverse Park + SVPWM */
    Foc_InvPark(vd, vq, ctrl_theta, &valpha, &vbeta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);
    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;

    /* 8) sync detection: encoder-elec angle must track the synthetic angle.
     *    A sliding window checks that the angle difference stays within a
     *    small band (rotor synchronized); a non-synced rotor (or wrong
     *    encoder direction) makes the difference sweep -> never syncs. */
    s_if_tick++;
    enc_elec = (float)Foc_ModPos(((int32_t)g_enc_count * (int32_t)g_foc_enc_dir),
                                 (int32_t)ENCODER_CPR)
             * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
    enc_elec -= (float)((int32_t)(enc_elec * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (enc_elec < 0.0f) {
        enc_elec += FOC_MATH_2PI;
    }
    diff = enc_elec - theta;
    if (diff >  FOC_MATH_PI) diff -= FOC_MATH_2PI;
    if (diff < -FOC_MATH_PI) diff += FOC_MATH_2PI;
    g_foc_if_diff_rad = diff;
    g_foc_if_rotor_rad = enc_elec;   /* rotor electrical angle (folded [0,2PI)) for diagnostics */

    /* count diff wraps + keep a continuous (unwrapped) diff for the sync
     * band: the wrapped diff jumps by +-2PI at the +-PI boundary, which would
     * otherwise make the 100ms band explode even when the rotor is locked. */
    {
        float dd = diff - s_if_last_diff;
        if (dd >  FOC_MATH_PI) { s_if_wrap_cnt++; dd -= FOC_MATH_2PI; }
        if (dd < -FOC_MATH_PI) { s_if_wrap_cnt++; dd += FOC_MATH_2PI; }
        s_if_last_diff = diff;
        if (s_if_diff_unwrapped_valid) {
            s_if_diff_unwrapped += dd;
        } else {
            s_if_diff_unwrapped = diff;
            s_if_diff_unwrapped_valid = 1u;
        }
    }

    /* Lock reference: while the synthetic frequency is still 0 the rotor is
     * aligned to the held current vector, so latch that angle offset as the
     * sync baseline. The sync band is then measured RELATIVE to this offset:
     * a locked rotor keeps diff near a constant, not near absolute zero.
     */
    if (!s_if_hold_done) {
        g_foc_if_lock_diff_rad = diff;
        s_if_diff_unwrapped = 0.0f;
        s_if_diff_unwrapped_valid = 1u;
    }
    g_foc_if_rel_diff_rad = s_if_diff_unwrapped;   /* unwrapped diff relative to lock (sync basis) */

    /* live sweep rate (cHz): wraps per 100ms window -> wraps/s * 100 */
    if ((s_if_tick - s_if_sweep_last_tick) >= (uint32_t)(FOC_ISR_HZ / 10u)) {
        uint32_t dwrap = s_if_wrap_cnt - s_if_sweep_last_wrap;
        g_foc_if_sweep_cHz = (float)dwrap * 1000.0f;
        s_if_sweep_last_wrap = s_if_wrap_cnt;
        s_if_sweep_last_tick = s_if_tick;
    }

    if (g_foc_if_freq_hz >= FOC_IF_SYNC_MIN_HZ) {
        float    band      = g_foc_if_sync_band_rad;
        uint32_t win_cnt   = g_foc_if_sync_win_cnt;
        uint32_t good_wins = g_foc_if_sync_good_wins;

        if (band < 0.01f)    band = 0.01f;
        if (win_cnt == 0u)   win_cnt = 1u;
        if (good_wins == 0u) good_wins = 1u;

        if (s_if_win_cnt == 0u) {
            s_if_diff_min = s_if_diff_unwrapped;
            s_if_diff_max = s_if_diff_unwrapped;
        } else {
            if (s_if_diff_unwrapped < s_if_diff_min) s_if_diff_min = s_if_diff_unwrapped;
            if (s_if_diff_unwrapped > s_if_diff_max) s_if_diff_max = s_if_diff_unwrapped;
        }
        s_if_win_cnt++;
        if (s_if_win_cnt >= win_cnt) {
            if ((s_if_diff_max - s_if_diff_min) < band) {
                if (++s_if_good_wins >= good_wins) {
                    Foc_IfHandover(pData);
                    return;
                }
            } else {
                s_if_good_wins = 0u;
            }
            s_if_win_cnt = 0u;
        }

        /* diagnostics: current window progress + observed rel-diff interval */
        g_foc_if_win_cnt     = s_if_win_cnt;
        g_foc_if_good_cnt    = s_if_good_wins;
        g_foc_if_win_min_rad = s_if_diff_min;
        g_foc_if_win_max_rad = s_if_diff_max;
    } else {
        /* sync not enabled yet (freq < min): no window data */
        g_foc_if_win_cnt     = 0u;
        g_foc_if_good_cnt    = 0u;
        g_foc_if_win_min_rad = s_if_diff_unwrapped;
        g_foc_if_win_max_rad = s_if_diff_unwrapped;
    }

    /* safety timeout: never synchronized -> fault code 2 */
    if (s_if_tick > ((uint32_t)FOC_IF_TIMEOUT_MS * FOC_ISR_HZ / 1000u)) {
        g_foc_if_evt    = 3u;
        g_foc_if_evt_v1 = (int32_t)(g_foc_if_freq_hz * 100.0f);   /* cHz */
        g_foc_if_evt_v2 = (int32_t)g_foc_iq_ma;
        g_foc_if_evt_v3 = (int32_t)(g_foc_if_diff_rad * 1000.0f); /* mrad */
        g_foc_if_evt_v4 = (s_if_tick > 0u)
                        ? (int32_t)((float)s_if_wrap_cnt * (float)FOC_ISR_HZ
                                    / (float)s_if_tick * 100.0f)   /* sweep cHz */
                        : 0;
        Foc_FaultStop(2u);
    }
}

/* I-F -> encoder handover (bumpless: angle, voltage and current continuous). */
static void Foc_IfHandover(const stc_i_data_t *pData)
{
    float theta = g_foc_theta_rad;
    float ctrl_theta = theta + ((g_foc_pi_off_180 != 0) ? FOC_MATH_PI : 0.0f);
    float id, iq;
    float vd_seed, vq_seed;

    /* RUN frame = true rotor d-axis (anchored at hold-done). Do NOT overwrite
     * s_align_offset here. The angle blend rotates the control frame from the
     * synthetic I-F angle to the true rotor angle over ~10 ms. */
    s_run_blend_cnt  = 0u;
    s_run_blend_from = g_foc_theta_rad;   /* RUN frame starts at the synthetic angle, blends to rotor */

    /* Current dq for PI seeding. */
    Foc_GetDq(pData, ctrl_theta, &id, &iq);

    /* Current applied voltage expressed in the dq frame (continuity). */
    vd_seed =  g_foc_valpha * Foc_Math_Cos(ctrl_theta) + g_foc_vbeta * Foc_Math_Sin(ctrl_theta);
    vq_seed = -g_foc_valpha * Foc_Math_Sin(ctrl_theta) + g_foc_vbeta * Foc_Math_Cos(ctrl_theta);
    if (vd_seed >  FOC_PI_UMAX_V) vd_seed =  FOC_PI_UMAX_V;
    if (vd_seed < -FOC_PI_UMAX_V) vd_seed = -FOC_PI_UMAX_V;
    if (vq_seed >  FOC_PI_UMAX_V) vq_seed =  FOC_PI_UMAX_V;
    if (vq_seed < -FOC_PI_UMAX_V) vq_seed = -FOC_PI_UMAX_V;

    PID_Seed(&s_pid_id, 0.0f,                     id, vd_seed);
    /* seed the Iq loop with the speed PI's initial output (kp*error), NOT the
     * I-F ramp current: avoids a 2.4A -> 0 reference step at handover. */
    {
        float iq_seed = g_foc_run_iq_sign * (g_foc_pid_spd_cfg.kp * (g_enc_speed_rpm - g_foc_run_target_rpm));
        if (iq_seed >  (float)FOC_IQ_REF_MA) iq_seed =  (float)FOC_IQ_REF_MA;
        if (iq_seed < -(float)FOC_IQ_REF_MA) iq_seed = -(float)FOC_IQ_REF_MA;
        PID_Seed(&s_pid_iq, iq_seed * 0.001f, iq, vq_seed);
    }

    s_id_f = 0.0f;
    s_iq_f = 0.0f;
    PID_Reset(&s_pid_spd);   /* fresh speed PI: do NOT inject I-F ramp current into the integral (caused windup -> stall) */

    g_foc_align_offset = s_align_offset;   /* expose the RUN anchor (was stale from mode23) */

    s_state           = FOC_STATE_RUN;
    g_foc_phase       = 3u;   /* run */
    g_foc_align_state = 2u;
    g_foc_if_sync     = 1u;
    g_foc_if_evt      = 2u;
    g_foc_if_evt_v1   = (int32_t)g_foc_align_offset;
    g_foc_if_evt_v2   = (int32_t)g_foc_iq_ma;
    g_foc_if_evt_v3   = (int32_t)(g_foc_if_freq_hz * 100.0f);
}

/*******************************************************************************
 * RUN: encoder-angle FOC current loop
 ******************************************************************************/
static void Foc_CurrentLoopStep(const stc_i_data_t *pData)
{
    float theta, ctrl_theta, id, iq, iq_ref_a, vd, vq, valpha, vbeta, du, dv, dw;

    if (Foc_OverCurrent(pData)) {
        Foc_FaultStop(1u);
        return;
    }

    /* Iq ramp (continues from the I-F phase) */
    {
        float step = g_foc_iq_ramp_ma_s / (float)FOC_ISR_HZ;
        if (g_foc_iq_ref_ma < g_foc_iq_ref_cmd_ma) {
            g_foc_iq_ref_ma += step;
            if (g_foc_iq_ref_ma > g_foc_iq_ref_cmd_ma) {
                g_foc_iq_ref_ma = g_foc_iq_ref_cmd_ma;
            }
        } else if (g_foc_iq_ref_ma > g_foc_iq_ref_cmd_ma) {
            g_foc_iq_ref_ma -= step;
            if (g_foc_iq_ref_ma < g_foc_iq_ref_cmd_ma) {
                g_foc_iq_ref_ma = g_foc_iq_ref_cmd_ma;
            }
        }
    }

    /* Electrical angle: true rotor angle (field-oriented), blended from the
     * synthetic I-F angle over FOC_RUN_BLEND_CNT ticks to avoid a current kick. */
    theta = Foc_CurLoopTheta();
    if (s_run_blend_cnt < FOC_RUN_BLEND_CNT) {
        float d = theta - s_run_blend_from;
        if (d >  FOC_MATH_PI) d -= FOC_MATH_2PI;
        if (d < -FOC_MATH_PI) d += FOC_MATH_2PI;
        theta = s_run_blend_from + d * ((float)s_run_blend_cnt / (float)FOC_RUN_BLEND_CNT);
        s_run_blend_cnt++;
    }
    g_foc_theta_rad = theta;
    /* live diagnostics in RUN: rotor = TRUE encoder electrical angle, and
     * diff = RUN control-frame offset vs true rotor (should stay at the handover
     * load angle ~0.3-0.5 rad; ~1.57 rad or +/-pi indicates a frame/phase bug). */
    {
        float enc = (float)Foc_ModPos(((int32_t)g_enc_count * (int32_t)g_foc_enc_dir),
                                      (int32_t)ENCODER_CPR)
                  * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
        enc -= (float)((int32_t)(enc * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
        if (enc < 0.0f) enc += FOC_MATH_2PI;
        g_foc_if_rotor_rad = enc;
        g_foc_if_diff_rad = enc - theta;
        while (g_foc_if_diff_rad >  FOC_MATH_PI) g_foc_if_diff_rad -= FOC_MATH_2PI;
        while (g_foc_if_diff_rad < -FOC_MATH_PI) g_foc_if_diff_rad += FOC_MATH_2PI;
        g_foc_if_rel_diff_rad = 0.0f;
    }
    ctrl_theta = theta + ((g_foc_pi_off_180 != 0) ? FOC_MATH_PI : 0.0f);

    /* Phase currents -> dq + EMA */
    Foc_GetDq(pData, ctrl_theta, &id, &iq);
    Foc_EmaFilter(&id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    /* Iq reference: RUN speed PI (fast cut when rotor exceeds target).
     * PID_UpdateUs(spd, actual, target): error = actual - target, output mA,
     * clamped to [0, FOC_IQ_REF_MA]. Falls back to ramped ref when kp <= 0. */
    if (g_foc_pid_spd_cfg.kp > 0.0f) {
        iq_ref_a = g_foc_run_iq_sign * PID_UpdateUs(&s_pid_spd, g_enc_speed_rpm, g_foc_run_target_rpm,
                                                     FOC_ISR_DT_US) * 0.001f;
        g_foc_iq_pi_ma = iq_ref_a * 1000.0f;
    } else {
        iq_ref_a = g_foc_run_iq_sign * g_foc_iq_ref_ma * 0.001f;
        g_foc_iq_pi_ma = iq_ref_a * 1000.0f;
    }
    vd = PID_UpdateUs(&s_pid_id, 0.0f,     id, FOC_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, iq_ref_a, iq, FOC_ISR_DT_US);
    g_foc_vd = vd;
    g_foc_vq = vq;

    /* Voltage envelope */
    Foc_ApplyVoltageEnvelope(&vd, &vq);

    /* Inverse Park + SVPWM + complementary PWM */
    Foc_InvPark(vd, vq, ctrl_theta, &valpha, &vbeta);
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;
}

/*******************************************************************************
 * MotorScope RTT 实时动画数据发送（ISR 内，非阻塞）
 * ----------------------------------------------------------------------------
 * 把 FOC 控制环关键量以 1kHz 文本帧（或 >2kHz 的 72B 二进制帧）写入独立 RTT
 * 上行通道，PC 端（tools/motor_scope）读取后在浏览器实时绘制电机动画。
 *
 * 配置（可用编译器 -D 覆盖；若想统一收口到 motor_config.h 也可移过去）：
 *   FOC_RTT_ENABLE   : 1 = 使能
 *   FOC_RTT_CH       : RTT 上行通道号（默认 0，与 MAIN_D/E 日志共用通道 0）
 *   FOC_RTT_RATE_HZ  : 帧率，默认 1000；>2000 时自动切换为二进制帧
 *
 * 说明：目标机目前只有通道 0 可用，MOTF 帧与日志都走通道 0；上位机解析器
 *       会自动过滤 MOTF 行、把其余文本当日志显示。若想用 MAIN_E(...) 打印
 *       也一样兼容（解析器会剥掉 ANSI 颜色和 [MAIN] 前缀）。
 *
 * 帧格式（文本）：
 *   MOTF,<mode>,<phase>,<rotor_mrad>,<theta_mrad>,<iq_ma>,<id_ma>,
 *        <vq_mv>,<vd_mv>,<spd_rpm>,<sync>,<diff_mrad>,<freq_cHz>,<ms>,
 *        <mech_mrad>,<is_ma>,<is_angle_mrad>,<v_mv>,<v_angle_mrad>,<theta_mech_mrad>,<cnt>
 ******************************************************************************/
#ifndef FOC_RTT_ENABLE
#define FOC_RTT_ENABLE      1u
#endif
#ifndef FOC_RTT_CH
#define FOC_RTT_CH          0u
#endif
#ifndef FOC_RTT_RATE_HZ
#define FOC_RTT_RATE_HZ     1000u
#endif

#if FOC_RTT_ENABLE && (FOC_RTT_RATE_HZ > 0u) && (FOC_RTT_RATE_HZ <= FOC_ISR_HZ)
#define FOC_RTT_DIV         ((uint16_t)(FOC_ISR_HZ / FOC_RTT_RATE_HZ))

#if FOC_RTT_RATE_HZ > 2000u
/* 二进制帧：76 字节（小端），PC 端按小端解析 */
typedef struct __attribute__((packed)) {
    uint32_t magic;        /* 0x46544F4D = "MOTF" */
    uint32_t ms;
    int32_t  rotor_mrad;   /* g_foc_if_rotor_rad * 1000 */
    int32_t  theta_mrad;   /* g_foc_theta_rad  * 1000 */
    int32_t  iq_ma;        /* g_foc_iq_ma */
    int32_t  id_ma;        /* g_foc_id_ma */
    int32_t  vq_mv;        /* g_foc_vq * 1000 */
    int32_t  vd_mv;        /* g_foc_vd * 1000 */
    int32_t  spd_rpm;      /* g_enc_speed_rpm */
    int32_t  diff_mrad;    /* g_foc_if_diff_rad * 1000 */
    int32_t  freq_cHz;     /* g_foc_if_freq_hz * 100 */
    uint8_t  mode;         /* FOC_MODE_* */
    uint8_t  phase;        /* g_foc_phase */
    uint8_t  sync;         /* g_foc_if_sync */
    uint8_t  rsv;
    int32_t  mech_mrad;   /* 连续机械角（不折叠）mrad：g_enc_count*g_foc_enc_dir 换算 */
    int32_t  is_ma;           /* sqrt(id^2+iq^2) mA（固件直传） */
    int32_t  is_angle_mrad;   /* atan2(iq,id) dq 电角度 mrad */
    int32_t  v_mv;            /* sqrt(vd^2+vq^2) mV */
    int32_t  v_angle_mrad;    /* atan2(vq,vd) dq 电角度 mrad */
    int32_t  theta_mech_mrad; /* g_foc_theta_rad / FOC_POLE_PAIRS * 1000；I-F 阶段 theta∈[0,2π)，RUN 阶段 theta∈[0,2π·PP)，机械角按各自折回 */
    int32_t  cnt;             /* g_enc_count 编码器原始计数（方向诊断） */
} foc_rtt_frame_t;
#endif /* FOC_RTT_RATE_HZ > 2000u */

/* ABZ 编码器计数 -> 转子电角度 [0,2PI)，与 RUN 状态机同公式。
 * 心跳里用它时刻刷新 g_foc_if_rotor_rad：即使 FOC 未运行（停止/开环/手转），
 * 动画里的转子位置也会跟随真实转子。 */
static float Foc_RotorAngleFromEncoderRad(void)
{
    int32_t cnt = Foc_ModPos(((int32_t)g_enc_count * (int32_t)g_foc_enc_dir),
                             (int32_t)ENCODER_CPR);
    float enc = (float)cnt * (FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR);
    enc -= (float)((int32_t)(enc * (1.0f / FOC_MATH_2PI))) * FOC_MATH_2PI;
    if (enc < 0.0f) {
        enc += FOC_MATH_2PI;
    }
    return enc;
}

/* ABZ 编码器计数 -> 连续机械角 (mrad)，方向与 FOC 电角度一致（乘 g_foc_enc_dir）。
 * g_enc_count 连续累计（Z 不复位），/ENCODER_CPR 得机械圈角，*2PI*1000 得 mrad。
 * 与电角度不同：不折叠，PC 端直接按连续值绘制（%360 回绕由前端处理）。
 * 注意：连续值随 int32 溢出回绕（4096 计数/圈，约 52 万圈后），调试工具无影响。 */
static int32_t Foc_MechAngleMrad(void)
{
    float mech_rad = (float)((int32_t)g_enc_count * (int32_t)g_foc_enc_dir)
                   * (FOC_MATH_2PI / (float)ENCODER_CPR);
    return (int32_t)(mech_rad * 1000.0f);
}

/* 电流/电压矢量 + 控制角机械角：dq 合成，供 MOTF 直传（主循环调用，不进 ISR）。
 * 全单精度 sqrtf/atan2f（M4F VSQRT + 硬件 FPU，1kHz 下开销 <1% CPU）。
 * theta_mech_mrad：I-F 阶段 theta∈[0,2π)，RUN 阶段 theta∈[0,2π·PP)，机械角按各自折回。 */
static int32_t Foc_IsMagMa(void)       { return (int32_t)sqrtf(g_foc_id_ma * g_foc_id_ma + g_foc_iq_ma * g_foc_iq_ma); }
static int32_t Foc_IsAngleMrad(void)   { return (int32_t)(atan2f(g_foc_iq_ma, g_foc_id_ma) * 1000.0f); }
static int32_t Foc_VMagMv(void)        { return (int32_t)(sqrtf(g_foc_vd * g_foc_vd + g_foc_vq * g_foc_vq) * 1000.0f); }
static int32_t Foc_VAngleMrad(void)    { return (int32_t)(atan2f(g_foc_vq, g_foc_vd) * 1000.0f); }
static int32_t Foc_ThetaMechMrad(void) { return (int32_t)(g_foc_theta_rad * (1000.0f / (float)FOC_POLE_PAIRS)); }

/* MotorScope 发送节流状态（间隔/阈值参数见 motor_config.h MOTOR_SCOPE_*） */
static uint64_t s_mot_scope_last_us    = 0u;
static int32_t  s_mot_scope_last_mode  = -1;
static int32_t  s_mot_scope_last_phase = -1;
static int32_t  s_mot_scope_last_sync  = -1;
static int32_t  s_mot_scope_last_evt   = -1;
static float    s_mot_scope_last_iq    = 0.0f;
static float    s_mot_scope_last_id    = 0.0f;
static float    s_mot_scope_last_th    = 0.0f;
static float    s_mot_scope_last_spd   = 0.0f;
static float    s_mot_scope_last_vd    = 0.0f;
static float    s_mot_scope_last_vq    = 0.0f;

/* 心跳发送：由主循环以 now_us 调用。发送门控采用"锁"机制（大幅降低打印频率，
 * 避免拖慢主循环导致 RUN 阶段编码器角陈旧而停转）：
 *   1) mode/phase/sync/I-F 事件变化   -> 立即发（无锁）
 *   2) spd/|vd|/|vq| 变化超过阈值     -> 立即发（无锁）
 *   3) id/iq/theta 连续量变化         -> 至少间隔 MOTOR_SCOPE_LOCK_MS_FAST
 *   4) 距上次发送 >= KEEPALIVE        -> 保活补发（防工具断联判定）
 * 放在主循环而不是 FOC ISR —— 保证 comm_mode=0（PWM/ADC 停止、FOC ISR 不触发）
 * 时也持续上报，手扭电机时 g_enc_count/机械角/电角度始终实时更新。 */
void Foc_RttSend(uint64_t now_us)
{
    uint32_t ms;
    uint64_t since_us;
    int32_t  mode, phase, sync, evt;
    float    iq, id, theta, spd, vd, vq;
    uint8_t  send;

    /* MotorScope 总开关：g_motor_scope=0 时完全跳过（Keil Watch 可改，默认 MOTOR_SCOPE_KEY） */
    if (g_motor_scope == 0) {
        return;
    }

    /* ---- 发送门控（锁）：满足任一条件才真正发送 ---- */
    mode  = (int32_t)g_foc_mode;
    phase = (int32_t)g_foc_phase;
    sync  = (int32_t)g_foc_if_sync;
    evt   = (int32_t)g_foc_if_evt;
    iq    = g_foc_iq_ma;
    id    = g_foc_id_ma;
    theta = g_foc_theta_rad;
    spd   = g_enc_speed_rpm;
    vd    = g_foc_vd;
    vq    = g_foc_vq;
    since_us = now_us - s_mot_scope_last_us;

    send = 0u;
    /* 1) 关键状态/事件变化 -> 立即发（无锁） */
    if ((mode != s_mot_scope_last_mode) || (phase != s_mot_scope_last_phase) ||
        (sync != s_mot_scope_last_sync) || (evt != s_mot_scope_last_evt)) {
        send = 1u;
    }
    /* 2) 关键数值变化超过阈值 -> 立即发（无锁） */
    if (!send) {
        if ((fabsf(spd - s_mot_scope_last_spd) > (float)MOTOR_SCOPE_CHG_SPD_RPM) ||
            (fabsf(vd - s_mot_scope_last_vd)    > (float)MOTOR_SCOPE_CHG_VD_V) ||
            (fabsf(vq - s_mot_scope_last_vq)    > (float)MOTOR_SCOPE_CHG_VQ_V)) {
            send = 1u;
        }
    }
    /* 3) id/iq/theta 连续量变化 -> 至少间隔 MOTOR_SCOPE_LOCK_MS_FAST */
    if (!send) {
        if ((iq != s_mot_scope_last_iq) || (id != s_mot_scope_last_id) ||
            (theta != s_mot_scope_last_th)) {
            if (since_us >= ((uint64_t)MOTOR_SCOPE_LOCK_MS_FAST * 1000u)) {
                send = 1u;
            }
        }
    }
    /* 4) 保活：距上次发送 >= MOTOR_SCOPE_LOCK_MS_KEEPALIVE（防工具断联判定） */
    if (!send) {
        if (since_us >= ((uint64_t)MOTOR_SCOPE_LOCK_MS_KEEPALIVE * 1000u)) {
            send = 1u;
        }
    }
    if (!send) {
        return;
    }

    /* 记录本次已发送的快照（下一次以此比较） */
    s_mot_scope_last_us    = now_us;
    s_mot_scope_last_mode  = mode;
    s_mot_scope_last_phase = phase;
    s_mot_scope_last_sync  = sync;
    s_mot_scope_last_evt   = evt;
    s_mot_scope_last_iq    = iq;
    s_mot_scope_last_id    = id;
    s_mot_scope_last_th    = theta;
    s_mot_scope_last_spd   = spd;
    s_mot_scope_last_vd    = vd;
    s_mot_scope_last_vq    = vq;
    ms = (uint32_t)(now_us / 1000u);

    /* 时刻从 ABZ 编码器刷新转子电角度（FOC 未运行时也更新，手转电机动画跟随） */
    g_foc_if_rotor_rad = Foc_RotorAngleFromEncoderRad();
    {
        float rotor_rad = g_foc_if_rotor_rad;

#if FOC_RTT_RATE_HZ > 2000u
    {
        foc_rtt_frame_t fr;

        fr.magic      = 0x46544F4Du;
        fr.ms         = ms;
        fr.rotor_mrad = (int32_t)(rotor_rad * 1000.0f);
        fr.theta_mrad = (int32_t)(g_foc_theta_rad  * 1000.0f);
        fr.iq_ma      = (int32_t)g_foc_iq_ma;
        fr.id_ma      = (int32_t)g_foc_id_ma;
        fr.vq_mv      = (int32_t)(g_foc_vq * 1000.0f);
        fr.vd_mv      = (int32_t)(g_foc_vd * 1000.0f);
        fr.spd_rpm    = (int32_t)g_enc_speed_rpm;
        fr.diff_mrad  = (int32_t)(g_foc_if_diff_rad * 1000.0f);
        fr.freq_cHz   = (int32_t)(g_foc_if_freq_hz * 100.0f);
        fr.mode       = g_foc_mode;
        fr.phase      = g_foc_phase;
        fr.sync       = g_foc_if_sync;
        fr.rsv        = 0u;
        fr.mech_mrad  = Foc_MechAngleMrad();
        fr.is_ma           = Foc_IsMagMa();
        fr.is_angle_mrad   = Foc_IsAngleMrad();
        fr.v_mv            = Foc_VMagMv();
        fr.v_angle_mrad    = Foc_VAngleMrad();
        fr.theta_mech_mrad = Foc_ThetaMechMrad();
        fr.cnt            = (int32_t)g_enc_count;
        SEGGER_RTT_Write(FOC_RTT_CH, (const char *)&fr, (unsigned)sizeof(fr));
    }
#else
    {
        char buf[224];
        int  n;

        n = snprintf(buf, sizeof(buf),
            "MOTF,%u,%u,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%u,%d,%d,%d,%d,%d,%d,%d\r\n",
            (unsigned)g_foc_mode, (unsigned)g_foc_phase,
            (int)(rotor_rad * 1000.0f),
            (int)(g_foc_theta_rad  * 1000.0f),
            (int)g_foc_iq_ma, (int)g_foc_id_ma,
            (int)(g_foc_vq * 1000.0f), (int)(g_foc_vd * 1000.0f),
            (int)g_enc_speed_rpm,
            (unsigned)g_foc_if_sync,
            (int)(g_foc_if_diff_rad * 1000.0f),
            (int)(g_foc_if_freq_hz * 100.0f),
            (unsigned)ms,
            (int)Foc_MechAngleMrad(),
            (int)Foc_IsMagMa(),
            (int)Foc_IsAngleMrad(),
            (int)Foc_VMagMv(),
            (int)Foc_VAngleMrad(),
            (int)Foc_ThetaMechMrad(),
            (int)g_enc_count);
        if (n > 0 && n < (int)sizeof(buf)) {
            SEGGER_RTT_Write(FOC_RTT_CH, buf, (unsigned)n);
        }
    }
#endif
    }
}
#else
void Foc_RttSend(uint64_t now_us) { (void)now_us; }
#endif /* FOC_RTT_ENABLE */

/*******************************************************************************
 * Foc_Isr - 20 kHz ISR callback (ADC1 EOCB, second callback slot)
 ******************************************************************************/
void Foc_Isr(const stc_i_data_t *pData)
{
    float theta, valpha, vbeta;
    float du, dv, dw;

    if (!g_foc_active) {
        return;
    }

    Foc_ClampOpenLoopVolt();   /* open-loop voltage hard cap (overheat protection) */

    /* --- mode 21: open loop --- */
    if (g_foc_mode == FOC_MODE_OPENLOOP) {
        theta = g_foc_theta_rad
              + (FOC_MATH_2PI * g_foc_openloop_freq_hz / (float)FOC_ISR_HZ);
        if (theta >= FOC_MATH_2PI) {
            theta -= FOC_MATH_2PI;
        }
        g_foc_theta_rad = theta;

        valpha = g_foc_openloop_volt_v * Foc_Math_Cos(theta);
        vbeta  = g_foc_openloop_volt_v * Foc_Math_Sin(theta);

        Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
        TMR4_PWM_SetDuty3Phase(du, dv, dw);

        g_foc_valpha = valpha;
        g_foc_vbeta  = vbeta;
        g_foc_du = du;
        g_foc_dv = dv;
        g_foc_dw = dw;
        return;
    }

    /* --- mode 23: standstill electrical alignment --- */
    if (g_foc_mode == FOC_MODE_ALIGN) {
        Foc_AlignStep(pData);
        return;
    }

    if (g_foc_mode != FOC_MODE_CURLOOP) {
        return;
    }

    /* --- mode 22: current-loop state machine --- */
    switch (s_state) {
    case FOC_STATE_IF_START:
        Foc_IfStartStep(pData);
        break;
    case FOC_STATE_RUN:
        Foc_CurrentLoopStep(pData);
        break;
    default:
        break;
    }
}

/*******************************************************************************
 * Foc_EncoderElecAngleRad - mechanical encoder counts -> electrical angle (rad)
 ******************************************************************************/
float Foc_EncoderElecAngleRad(void)
{
    float mech_rad = (float)g_enc_count * (FOC_MATH_2PI / (float)ENCODER_CPR);
    return mech_rad * (float)FOC_POLE_PAIRS;
}

/*******************************************************************************
 * Foc_GetState / Foc_IsRunning
 ******************************************************************************/
uint8_t Foc_GetState(void)
{
    return g_foc_align_state;
}

uint8_t Foc_IsRunning(void)
{
    return g_foc_active;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
