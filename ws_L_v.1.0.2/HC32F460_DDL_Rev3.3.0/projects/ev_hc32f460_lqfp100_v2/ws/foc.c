/**
 *******************************************************************************
 * @file  foc.c
 * @brief FOC runner (open-loop mode 21 + current-loop mode 22).
 *
 *        ISR frequency assumption:
 *          I.c uses the INMOP-style double trigger (SCMP0 @ PEAK + SCMP2 @
 *          VALLEY) -> 10 kHz PWM produces 20 kHz EOCB ISR. Foc_Isr therefore
 *          runs at FOC_ISR_HZ = 20000 (motor_config.h).
 *          If the project is ever switched to a single trigger, set
 *          FOC_ISR_HZ to the actual ISR rate (e.g. 10000).
 *
 *        Mode 21 (open loop): theta integrates g_foc_openloop_freq_hz each ISR
 *        and the voltage vector (g_foc_openloop_volt_v) is transformed by
 *        SVPWM into U/V/W duty, written through TMR4_PWM_SetDuty3Phase().
 *
 *        Mode 22 (current loop, INMOP-style):
 *          Foc_StartCurrentLoop() -> ALIGN phase outputs a fixed vector
 *          (valpha = FOC_ALIGN_VOLT_V, vbeta = 0) for FOC_ALIGN_TIME_MS to
 *          lock the rotor to the alpha axis, records g_enc_count as
 *          s_align_offset, then RUN phase runs:
 *            ia/ib/ic (A) -> Foc_Clarke -> Foc_Park(theta) -> id/iq
 *            -> vd = PI_id(0, id), vq = PI_iq(iq_ref, iq)   (dev_pid)
 *            -> Foc_InvPark(vd, vq, theta) -> Foc_Svpwm -> TMR4 duty
 *          with theta = ((g_enc_count - s_align_offset) mod ENCODER_CPR)
 *          x 2PI / ENCODER_CPR x FOC_POLE_PAIRS (negative mod wrapped).
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
/* Gentle handover / voltage envelope (all Watch tunable) */
volatile float g_foc_vmax_v       = (float)FOC_VMAX_V;        /* current-loop max |v| (V) */
volatile float g_foc_vramp_v_s    = (float)FOC_VRAMP_V_S;     /* voltage envelope ramp (V/s) */
volatile float g_foc_cur_fb_alpha = FOC_CUR_FB_ALPHA;         /* EMA weight on id/iq (1.0=off) */
volatile float g_foc_iq_ramp_ma_s = (float)FOC_IQ_RAMP_MA_S;  /* Iq soft-start ramp (mA/s) */
volatile float g_foc_vlim_v       = 0.0f;                     /* current voltage envelope (V) */

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
    .integral_max = 6.0f,     /* A*s (INMOP reference suggestion) */
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
    .integral_max = 6.0f,     /* A*s (INMOP reference suggestion) */
    .i_term_max   = 0.0f,     /* disabled: bounded by integral_max + output clamp */
    .update_ms    = 0,        /* no throttle: every ISR */
};

/* Local state */
static uint8_t s_bInited = 0u;

/* Current-loop state machine (mirrored to g_foc_align_state for Watch) */
typedef enum {
    FOC_STATE_IDLE     = 0,
    FOC_STATE_ALIGN    = 1,
    FOC_STATE_RUN      = 2,
    FOC_STATE_OL_START = 3,   /* open-loop spin-up, then hand over to RUN */
} foc_state_t;

static foc_state_t s_state        = FOC_STATE_IDLE;
static uint32_t    s_align_tick   = 0u;
static uint32_t    s_align_total  = (uint32_t)FOC_ALIGN_TIME_MS * FOC_ISR_HZ / 1000u;
static int32_t     s_align_offset = 0;
static uint32_t    s_ol_tick   = 0u;
static uint32_t    s_ol_total  = (uint32_t)FOC_OL_START_MS * FOC_ISR_HZ / 1000u;
static float       s_vlim      = 0.0f;   /* current voltage envelope (V) */
static float       s_id_f      = 0.0f;   /* EMA-filtered d current (A) */
static float       s_iq_f      = 0.0f;   /* EMA-filtered q current (A) */

/* PI runtime states (bound to the Watch-tunable configs) */
static pid_state_t s_pid_id;
static pid_state_t s_pid_iq;

/* ISR period in us (FOC_ISR_HZ = 20000 -> 50 us) */
#define FOC_ISR_DT_US  (1000000u / FOC_ISR_HZ)

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

/* Over-current check: any phase |I| > FOC_OC_LIMIT_A (mA conversion). */
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

/* Fault stop: latch fault, disable output immediately (ISR-safe). */
static void Foc_FaultStop(void)
{
    g_foc_fault       = 1u;
    g_foc_active      = 0u;
    s_state           = FOC_STATE_IDLE;
    g_foc_align_state = 0u;
    s_oc_cnt          = 0u;
    TMR4_PWM_EmergencyStop();
    g_foc_du = 0.0f;
    g_foc_dv = 0.0f;
    g_foc_dw = 0.0f;
}

/* Electrical angle from the aligned encoder: mechanical -> electrical x pole
 * pairs, negative difference wrapped to [0, ENCODER_CPR). */
static float Foc_CurLoopTheta(void)
{
    int32_t diff = Foc_ModPos((int32_t)(g_enc_count - s_align_offset),
                              (int32_t)ENCODER_CPR);

    return (float)diff * (FOC_MATH_2PI / (float)ENCODER_CPR)
         * (float)FOC_POLE_PAIRS;
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
 * Foc_StartOpenLoop - enable complementary PWM + open-loop voltage (mode 21)
 ******************************************************************************/
void Foc_StartOpenLoop(void)
{
    g_foc_theta_rad   = 0.0f;
    g_foc_fault       = 0u;
    g_foc_fault_i_ma  = 0.0f;
    s_oc_cnt          = 0u;
    g_foc_mode        = FOC_MODE_OPENLOOP;
    s_state           = FOC_STATE_IDLE;
    g_foc_align_state = 0u;

    /* Reconfigure all 3 channels to complementary (dead-timer) PWM.
     * SetFocMode stops the counter, so no glitch while reconfiguring. */
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
 * Foc_StartCurrentLoop - rotor align + encoder FOC current loop (mode 22)
 ******************************************************************************/
void Foc_StartCurrentLoop(void)
{
    g_foc_fault       = 0u;
    g_foc_fault_i_ma  = 0.0f;
    s_oc_cnt          = 0u;
    g_foc_mode        = FOC_MODE_CURLOOP;
    g_foc_theta_rad   = 0.0f;
    g_foc_iq_ref_ma   = 0.0f;      /* soft-start: ramp to g_foc_iq_ref_cmd_ma */
    g_foc_id_ma       = 0.0f;
    g_foc_iq_ma       = 0.0f;
    g_foc_vd          = 0.0f;
    g_foc_vq          = 0.0f;
#if (FOC_OL_START_MS > 0u)
    /* Gentle start: spin up in open loop (mode-21 settings), then hand over. */
    s_state           = FOC_STATE_OL_START;
    s_ol_tick         = 0u;
#else
    /* Legacy: align from standstill, then run. */
    s_state           = FOC_STATE_ALIGN;
    s_align_tick      = 0u;
#endif
    s_vlim            = FOC_VLIM_START_V;
    s_id_f            = 0.0f;
    s_iq_f            = 0.0f;
    g_foc_align_state = 1u;

    /* Fresh PI state for the run phase */
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
 * Foc_Stop - disable FOC output
 *
 *   Uses TMR4_PWM_EmergencyStop(): stops the counter, clears it and disables
 *   all OC channels (all outputs low). The caller (main.c) restarts the
 *   counter when switching back to six-step modes, because CommRunner keeps
 *   the counter running and never restarts it itself.
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
 * Current-loop state machine steps (called from Foc_Isr)
 ******************************************************************************/

/* ALIGN: hold a fixed alpha-axis vector until the rotor locks, then record
 * the encoder offset and enter RUN. */
static void Foc_AlignStep(const stc_i_data_t *pData)
{
    float du, dv, dw;

    if (Foc_OverCurrent(pData)) {
        Foc_FaultStop();
        return;
    }

    /* Fixed vector along alpha axis: locks the rotor to the alpha axis. */
    Foc_Svpwm(FOC_ALIGN_VOLT_V, 0.0f, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    g_foc_theta_rad = 0.0f;
    g_foc_valpha    = FOC_ALIGN_VOLT_V;
    g_foc_vbeta     = 0.0f;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;

    s_align_tick++;
    if (s_align_tick >= s_align_total) {
        s_align_offset    = g_enc_count;   /* encoder position at alpha axis */
        PID_Reset(&s_pid_id);
        PID_Reset(&s_pid_iq);
        s_id_f            = 0.0f;
        s_iq_f            = 0.0f;
        s_vlim            = FOC_VLIM_START_V;
        s_state           = FOC_STATE_RUN;
        g_foc_align_state = 2u;
    }
}

#if (FOC_OL_START_MS > 0u)
static void Foc_Handover(const stc_i_data_t *pData);   /* forward decl */

/* OL_START: spin up exactly like mode 21 (same Watch settings), then hand
 * over to the current loop without any voltage or angle step. */
static void Foc_OlStartStep(const stc_i_data_t *pData)
{
    float theta, valpha, vbeta;
    float du, dv, dw;

    if (Foc_OverCurrent(pData)) {
        Foc_FaultStop();
        return;
    }

    /* identical to mode-21 open loop */
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

    s_ol_tick++;
    if (s_ol_tick >= s_ol_total) {
        Foc_Handover(pData);
    }
}

/* Hand over from open loop to current loop:
 *   - re-anchor the encoder so encoder-theta == synthetic theta (no angle jump),
 *   - seed the d/q PIs with the current open-loop voltage (no voltage jump),
 *   - Iq ref starts at 0 and ramps (g_foc_iq_ramp_ma_s),
 *   - voltage envelope starts at FOC_VLIM_START_V and ramps to g_foc_vmax_v. */
static void Foc_Handover(const stc_i_data_t *pData)
{
    float theta = g_foc_theta_rad;
    float ia, ib, ic, ialpha, ibeta, id, iq;
    float vd_seed, vq_seed;
    float sign = (float)g_foc_cur_sign;

    /* encoder offset so that Foc_CurLoopTheta() == theta */
    {
        float per_cnt = FOC_MATH_2PI * (float)FOC_POLE_PAIRS / (float)ENCODER_CPR;
        int32_t diff  = (int32_t)(theta / per_cnt);   /* 0..4095 */
        s_align_offset = (int32_t)g_enc_count - diff;
    }

    /* current dq (for PI seeding) */
    if (pData != NULL) {
        ia = (float)pData->i16IU_mA * 0.001f * sign;
        ib = (float)pData->i16IV_mA * 0.001f * sign;
        ic = (float)pData->i16IW_mA * 0.001f * sign;
    } else {
        ia = ib = ic = 0.0f;
    }
    Foc_Clarke(ia, ib, ic, &ialpha, &ibeta);
    Foc_Park(ialpha, ibeta, theta, &id, &iq);

    /* Open-loop vector at angle theta = pure d-axis voltage (V, 0). */
    vd_seed = g_foc_openloop_volt_v;
    vq_seed = 0.0f;
    if (vd_seed >  FOC_PI_UMAX_V) vd_seed =  FOC_PI_UMAX_V;
    if (vd_seed < -FOC_PI_UMAX_V) vd_seed = -FOC_PI_UMAX_V;

    /* bumpless: back-calculate PI integral so next output ~= current voltage */
    PID_Seed(&s_pid_id, 0.0f, id, vd_seed);
    PID_Seed(&s_pid_iq, 0.0f, iq, vq_seed);

    g_foc_iq_ref_ma = 0.0f;
    s_id_f = 0.0f;
    s_iq_f = 0.0f;
    s_vlim = FOC_VLIM_START_V;

    s_state           = FOC_STATE_RUN;
    g_foc_align_state = 2u;

}
#endif /* FOC_OL_START_MS > 0 */

/* RUN: encoder-angle FOC current loop (Clarke/Park/PI/InvPark/SVPWM). */
static void Foc_CurrentLoopStep(const stc_i_data_t *pData)
{
    float ia, ib, ic;
    float ialpha, ibeta;
    float id, iq;
    float iq_ref_a;
    float vd, vq;
    float valpha, vbeta;
    float du, dv, dw;
    float theta;
    float sign;

    if (Foc_OverCurrent(pData)) {
        Foc_FaultStop();
        return;
    }

    /* Iq soft-start ramp: move the actual reference toward the Watch target
     * by FOC_IQ_RAMP_MA_S / FOC_ISR_HZ mA per ISR. */
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

    /* Phase currents (A), sign-corrected (g_foc_cur_sign, Watch editable) */
    sign = (float)g_foc_cur_sign;
    if (pData != NULL) {
        ia = (float)pData->i16IU_mA * 0.001f * sign;
        ib = (float)pData->i16IV_mA * 0.001f * sign;
        ic = (float)pData->i16IW_mA * 0.001f * sign;
    } else {
        ia = 0.0f;
        ib = 0.0f;
        ic = 0.0f;
    }

    /* Clarke + Park -> dq currents */
    Foc_Clarke(ia, ib, ic, &ialpha, &ibeta);
    Foc_Park(ialpha, ibeta, theta, &id, &iq);

    /* EMA on dq feedback (g_foc_cur_fb_alpha: 1.0 = off, lower = smoother) */
    {
        float alpha = g_foc_cur_fb_alpha;
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        s_id_f += alpha * (id - s_id_f);
        s_iq_f += alpha * (iq - s_iq_f);
        id = s_id_f;
        iq = s_iq_f;
    }
    g_foc_id_ma = id * 1000.0f;
    g_foc_iq_ma = iq * 1000.0f;

    /* Id -> 0, Iq -> ramped reference (A), INMOP-style PI (dev_pid) */
    iq_ref_a = g_foc_iq_ref_ma * 0.001f;
    vd = PID_UpdateUs(&s_pid_id, 0.0f,     id, FOC_ISR_DT_US);
    vq = PID_UpdateUs(&s_pid_iq, iq_ref_a, iq, FOC_ISR_DT_US);
    g_foc_vd = vd;
    g_foc_vq = vq;

    /* Voltage envelope: ramp allowed |v| from FOC_VLIM_START_V to
     * g_foc_vmax_v, then clamp magnitude. No voltage step at handover. */
    s_vlim += g_foc_vramp_v_s / (float)FOC_ISR_HZ;
    if (s_vlim > g_foc_vmax_v) {
        s_vlim = g_foc_vmax_v;
    }
    g_foc_vlim_v = s_vlim;
    {
        float v2    = vd * vd + vq * vq;
        float vmax2 = s_vlim * s_vlim;
        if (v2 > vmax2) {
            float k = s_vlim / sqrtf(v2);
            vd *= k;
            vq *= k;
        }
    }

    /* Inverse Park + SVPWM + complementary PWM */
    Foc_InvPark(vd, vq, theta, &valpha, &vbeta);
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
 *
 *   Must stay short: LUT sin/cos + SVPWM + PI + 3 compare writes. No prints,
 *   no blocking, no malloc.
 ******************************************************************************/
void Foc_Isr(const stc_i_data_t *pData)
{
    float theta, valpha, vbeta;
    float du, dv, dw;

    if (!g_foc_active) {
        return;
    }

    /* --- mode 21: open loop (unchanged) --- */
    if (g_foc_mode == FOC_MODE_OPENLOOP) {
        /* electrical angle integration (fold to [0, 2PI) to keep float precision) */
        theta = g_foc_theta_rad
              + (FOC_MATH_2PI * g_foc_openloop_freq_hz / (float)FOC_ISR_HZ);
        if (theta >= FOC_MATH_2PI) {
            theta -= FOC_MATH_2PI;
        }
        g_foc_theta_rad = theta;

        /* open-loop voltage vector (V) */
        valpha = g_foc_openloop_volt_v * Foc_Math_Cos(theta);
        vbeta  = g_foc_openloop_volt_v * Foc_Math_Sin(theta);

        /* SVPWM -> duty % -> TMR4 complementary PWM */
        Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
        TMR4_PWM_SetDuty3Phase(du, dv, dw);

        /* JScope observables */
        g_foc_valpha = valpha;
        g_foc_vbeta  = vbeta;
        g_foc_du = du;
        g_foc_dv = dv;
        g_foc_dw = dw;
        return;
    }

    if (g_foc_mode != FOC_MODE_CURLOOP) {
        return;
    }

    /* --- mode 22: current-loop state machine --- */
    switch (s_state) {
    case FOC_STATE_ALIGN:
        Foc_AlignStep(pData);
        break;
    case FOC_STATE_OL_START:
        Foc_OlStartStep(pData);
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
 *
 *   g_enc_count = 4096 counts/rev (ENCODER_CPR), pole pairs = FOC_POLE_PAIRS.
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
