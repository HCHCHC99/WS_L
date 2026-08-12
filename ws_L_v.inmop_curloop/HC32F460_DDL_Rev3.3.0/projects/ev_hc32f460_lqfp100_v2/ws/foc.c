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
 *            - Iq ramps 0 -> g_foc_iq_ref_cmd_ma (100 mA)
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
volatile int8_t  g_foc_cur_sign         = (int8_t)FOC_CUR_SIGN;
volatile int8_t  g_foc_enc_dir         = (int8_t)FOC_ENC_DIR;   /* encoder direction, Watch tunable */
volatile int8_t  g_foc_pi_off_180      = 0;                    /* 1 = +180deg control angle (flip torque), Watch tunable */
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

/* Voltage envelope / feedback filter (all Watch tunable) */
volatile float g_foc_vmax_v       = (float)FOC_VMAX_V;        /* current-loop max |v| (V) */
volatile float g_foc_vramp_v_s    = (float)FOC_VRAMP_V_S;     /* voltage envelope ramp (V/s) */
volatile float g_foc_cur_fb_alpha = FOC_CUR_FB_ALPHA;         /* EMA weight on id/iq (1.0=off) */
volatile float g_foc_iq_ramp_ma_s = (float)FOC_IQ_RAMP_MA_S;  /* Iq soft-start ramp (mA/s) */
volatile float g_foc_vlim_v       = 0.0f;                     /* current voltage envelope (V) */

/* I-F start observables */
volatile float   g_foc_if_freq_hz  = 0.0f;   /* current I-F electrical frequency (Hz) */
volatile float   g_foc_if_diff_rad = 0.0f;   /* encoder-elec angle - synthetic angle (rad) */
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

/* ISR period in us (FOC_ISR_HZ = 20000 -> 50 us) */
#define FOC_ISR_DT_US  (1000000u / FOC_ISR_HZ)
#define FOC_IF_HOLD_MAX_CNT ((uint32_t)FOC_IF_HOLD_MAX_MS * FOC_ISR_HZ / 1000u)

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
    g_foc_openloop_volt_v = FOC_OPENLOOP_VOLT_V;   /* mode 21 starts at configured voltage */
    g_foc_mode        = FOC_MODE_OPENLOOP;
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
    g_foc_theta_rad   = 0.0f;
    g_foc_iq_ref_ma   = 0.0f;      /* ramps to g_foc_iq_ref_cmd_ma (100 mA) */
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
 * Current = V/R (0.4V / 0.15ohm ~ 2.7A, within OC 4A); OC still protects.
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

    /* 0) initial hold: keep theta=0 and freq=0 until the Iq reference has
     *    ramped up enough to lock the rotor onto the initial current vector,
     *    so the field always starts rotating from a synchronized state. */
    if ((g_foc_iq_ref_ma < (float)FOC_IF_HOLD_IQ_MA) &&
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

    /* 4) Iq soft ramp 0 -> g_foc_iq_ref_cmd_ma (100 mA) */
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

    /* count diff wraps to estimate the sweep rate (direction/scale debug) */
    {
        float dd = diff - s_if_last_diff;
        if (dd >  FOC_MATH_PI) s_if_wrap_cnt++;
        if (dd < -FOC_MATH_PI) s_if_wrap_cnt++;
        s_if_last_diff = diff;
    }

    if (g_foc_if_freq_hz >= FOC_IF_SYNC_MIN_HZ) {
        if (s_if_win_cnt == 0u) {
            s_if_diff_min = diff;
            s_if_diff_max = diff;
        } else {
            if (diff < s_if_diff_min) s_if_diff_min = diff;
            if (diff > s_if_diff_max) s_if_diff_max = diff;
        }
        s_if_win_cnt++;
        if (s_if_win_cnt >= FOC_IF_SYNC_WIN_CNT) {
            if ((s_if_diff_max - s_if_diff_min) < FOC_IF_SYNC_BAND_RAD) {
                if (++s_if_good_wins >= FOC_IF_SYNC_GOOD_WINS) {
                    Foc_IfHandover(pData);
                    return;
                }
            } else {
                s_if_good_wins = 0u;
            }
            s_if_win_cnt = 0u;
        }
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

    /* Anchor encoder so Foc_CurLoopTheta() == theta (rotor synchronized). */
    {
        float per_cnt = FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR;
        int32_t diff  = (int32_t)(theta / per_cnt);   /* 0..4095 */
        /* store the anchor in enc_dir-scaled counts so CurLoopTheta() == theta
         * also when g_foc_enc_dir = -1 (no-op for +1). */
        s_align_offset = Foc_ModPos(((int32_t)g_enc_count * (int32_t)g_foc_enc_dir) - diff,
                                    (int32_t)ENCODER_CPR);
    }

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
    PID_Seed(&s_pid_iq, g_foc_iq_ref_ma * 0.001f, iq, vq_seed);

    s_id_f = 0.0f;
    s_iq_f = 0.0f;

    s_state           = FOC_STATE_RUN;
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

    /* Electrical angle from the aligned encoder */
    theta = Foc_CurLoopTheta();
    g_foc_theta_rad = theta;
    ctrl_theta = theta + ((g_foc_pi_off_180 != 0) ? FOC_MATH_PI : 0.0f);

    /* Phase currents -> dq + EMA */
    Foc_GetDq(pData, ctrl_theta, &id, &iq);
    Foc_EmaFilter(&id, &iq);
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    /* PI */
    iq_ref_a = g_foc_iq_ref_ma * 0.001f;
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
