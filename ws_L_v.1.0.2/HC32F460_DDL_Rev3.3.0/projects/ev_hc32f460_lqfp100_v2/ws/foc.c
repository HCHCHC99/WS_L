/**
 *******************************************************************************
 * @file  foc.c
 * @brief FOC runner (open-loop, mode 21).
 *
 *        ISR frequency assumption:
 *          I.c uses the INMOP-style double trigger (SCMP0 @ PEAK + SCMP2 @
 *          VALLEY) -> 10 kHz PWM produces 20 kHz EOCB ISR. Foc_Isr therefore
 *          runs at FOC_ISR_HZ = 20000 (motor_config.h).
 *          If the project is ever switched to a single trigger, set
 *          FOC_ISR_HZ to the actual ISR rate (e.g. 10000).
 *
 *        Open loop: theta integrates g_foc_openloop_freq_hz each ISR and the
 *        voltage vector (g_foc_openloop_volt_v) is transformed by SVPWM into
 *        U/V/W duty, written through TMR4_PWM_SetDuty3Phase().
 *
 *        Reserved (not implemented): closed loop plugs in here -
 *          I_GetData() -> Foc_Clarke() -> Foc_Park() ->
 *          Id/Iq PI (cur_loop-style) -> Foc_InvPark() -> Foc_Svpwm(),
 *          with theta from Foc_EncoderElecAngleRad() instead of the
 *          open-loop integrator.
 *******************************************************************************
 */

#include "foc.h"
#include "foc_math.h"
#include "tmr4_pwm.h"
#include "encoder.h"
#include "motor_config.h"

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

/* Local state */
static uint8_t s_bInited = 0u;

/*******************************************************************************
 * Foc_Init - math LUT + register ISR callback (no output)
 ******************************************************************************/
void Foc_Init(void)
{
    if (s_bInited) {
        return;
    }

    Foc_Math_Init();
    I_RegisterFocCallback(Foc_Isr);
    s_bInited = 1u;
}

/*******************************************************************************
 * Foc_StartOpenLoop - enable complementary PWM + open-loop voltage
 ******************************************************************************/
void Foc_StartOpenLoop(void)
{
    g_foc_theta_rad = 0.0f;

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
 * Foc_Stop - disable FOC output
 *
 *   Uses TMR4_PWM_EmergencyStop(): stops the counter, clears it and disables
 *   all OC channels (all outputs low). The caller (main.c) restarts the
 *   counter when switching back to six-step modes, because CommRunner keeps
 *   the counter running and never restarts it itself.
 ******************************************************************************/
void Foc_Stop(void)
{
    g_foc_active = 0u;
    TMR4_PWM_EmergencyStop();
    g_foc_du = 0.0f;
    g_foc_dv = 0.0f;
    g_foc_dw = 0.0f;
    g_foc_valpha = 0.0f;
    g_foc_vbeta  = 0.0f;
}

/*******************************************************************************
 * Foc_Isr - 20 kHz ISR callback (ADC1 EOCB, second callback slot)
 *
 *   Must stay short: LUT sin/cos + SVPWM + 3 compare writes. No prints,
 *   no blocking, no malloc.
 ******************************************************************************/
void Foc_Isr(const stc_i_data_t *pData)
{
    float theta, valpha, vbeta;
    float du, dv, dw;

    (void)pData;   /* current data not used in open loop */

    if (!g_foc_active) {
        return;
    }

    /* --- electrical angle integration (fold to [0, 2PI) to keep float precision) --- */
    theta = g_foc_theta_rad
          + (FOC_MATH_2PI * g_foc_openloop_freq_hz / (float)FOC_ISR_HZ);
    if (theta >= FOC_MATH_2PI) {
        theta -= FOC_MATH_2PI;
    }
    g_foc_theta_rad = theta;

    /* --- open-loop voltage vector (V) --- */
    valpha = g_foc_openloop_volt_v * Foc_Math_Cos(theta);
    vbeta  = g_foc_openloop_volt_v * Foc_Math_Sin(theta);

    /* --- SVPWM -> duty % -> TMR4 complementary PWM --- */
    Foc_Svpwm(valpha, vbeta, FOC_VBUS_V, &du, &dv, &dw);
    TMR4_PWM_SetDuty3Phase(du, dv, dw);

    /* --- JScope observables --- */
    g_foc_valpha = valpha;
    g_foc_vbeta  = vbeta;
    g_foc_du = du;
    g_foc_dv = dv;
    g_foc_dw = dw;

    /* Reserved: closed loop
     *   stc_i_data_t data; I_GetData(&data);
     *   Foc_Clarke((float)data.i16IU_mA/1000, ...);
     *   Foc_Park(alpha, beta, Foc_EncoderElecAngleRad(), &id, &iq);
     *   -> Id/Iq PI -> Foc_InvPark(vd, vq, theta, &valpha, &vbeta);
     *   -> Foc_Svpwm(...) as above. */
}

/*******************************************************************************
 * Foc_EncoderElecAngleRad - reserved: encoder -> electrical angle (rad)
 *
 *   g_enc_count = 4096 counts/rev (ENCODER_CPR), pole pairs = FOC_POLE_PAIRS.
 ******************************************************************************/
float Foc_EncoderElecAngleRad(void)
{
    float mech_rad = (float)g_enc_count * (FOC_MATH_2PI / (float)ENCODER_CPR);
    return mech_rad * (float)FOC_POLE_PAIRS;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
