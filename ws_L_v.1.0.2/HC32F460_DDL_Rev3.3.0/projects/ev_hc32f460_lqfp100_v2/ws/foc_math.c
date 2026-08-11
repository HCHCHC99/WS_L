/**
 *******************************************************************************
 * @file  foc_math.c
 * @brief Pure FOC math library (hardware independent, float).
 *
 *        Sin/Cos: 1024-point float LUT filled once by Foc_Math_Init()
 *        (sinf at init time only), plus linear interpolation at runtime.
 *        No sinf/cosf in the ISR path.
 *
 *        Transform conventions (same as STM32 INMOP reference foc.c):
 *          Clarke : alpha = ia,                 beta = (ia + 2*ib)/sqrt(3)
 *          Park   : id = c*alpha + s*beta,      iq = -s*alpha + c*beta
 *          InvPark: valpha = c*vd - s*vq,       vbeta = s*vd + c*vq
 *******************************************************************************
 */

#include "foc_math.h"
#include <math.h>

/*=============================================================================
 * Local state
 *=============================================================================*/
static float   s_fSinLut[FOC_MATH_LUT_SIZE];
static uint8_t s_bLutInit = 0;

/* index scale: 1024 samples per 2*PI rad */
#define FOC_MATH_LUT_SCALE  ((float)FOC_MATH_LUT_SIZE / FOC_MATH_2PI)

/*******************************************************************************
 * Foc_Math_Init - Build sin LUT (sinf used once here, never in ISR)
 ******************************************************************************/
void Foc_Math_Init(void)
{
    uint32_t i;

    if (s_bLutInit) {
        return;
    }
    for (i = 0u; i < FOC_MATH_LUT_SIZE; i++) {
        s_fSinLut[i] = sinf(FOC_MATH_2PI * (float)i / (float)FOC_MATH_LUT_SIZE);
    }
    s_bLutInit = 1u;
}

/*******************************************************************************
 * Local helpers
 ******************************************************************************/

/* Fold theta into [0, 2*PI). Bounded: single division + one adjustment.
 * Works for negative and > 2*PI inputs. */
static float Foc_Math_Fold(float theta)
{
    float k = (float)(int32_t)(theta * (1.0f / FOC_MATH_2PI));

    theta -= k * FOC_MATH_2PI;
    if (theta < 0.0f) {
        theta += FOC_MATH_2PI;
    }
    if (theta >= FOC_MATH_2PI) {
        theta -= FOC_MATH_2PI;   /* guard against float rounding */
    }
    return theta;
}

/* Lookup + linear interpolation. Input must be in [0, 2*PI). */
static float Foc_Math_LutSin(float theta)
{
    float    fPos = theta * FOC_MATH_LUT_SCALE;
    uint32_t i0   = (uint32_t)fPos & FOC_MATH_LUT_MASK;
    uint32_t i1   = (i0 + 1u) & FOC_MATH_LUT_MASK;
    float    frac = fPos - (float)(uint32_t)fPos;

    return s_fSinLut[i0] + (s_fSinLut[i1] - s_fSinLut[i0]) * frac;
}

/*******************************************************************************
 * Foc_Math_Sin / Foc_Math_Cos
 ******************************************************************************/
float Foc_Math_Sin(float theta)
{
    return Foc_Math_LutSin(Foc_Math_Fold(theta));
}

float Foc_Math_Cos(float theta)
{
    /* cos(theta) = sin(theta + PI/2) */
    return Foc_Math_LutSin(Foc_Math_Fold(theta + FOC_MATH_HALF_PI));
}

/*******************************************************************************
 * Foc_Clarke - 3-phase -> stationary 2-axis (equal-amplitude)
 *   alpha = ia, beta = (ia + 2*ib)/sqrt(3)
 *   ic is not needed because ia+ib+ic=0 in a star-connected motor.
 ******************************************************************************/
void Foc_Clarke(float ia, float ib, float ic, float *alpha, float *beta)
{
    (void)ic;

    if ((alpha == NULL) || (beta == NULL)) {
        return;
    }

    *alpha = ia;
    *beta  = (ia + 2.0f * ib) * (1.0f / FOC_MATH_SQRT3);
}

/*******************************************************************************
 * Foc_Park - stationary -> rotating frame
 ******************************************************************************/
void Foc_Park(float alpha, float beta, float theta, float *id, float *iq)
{
    float c, s;

    if ((id == NULL) || (iq == NULL)) {
        return;
    }

    c = Foc_Math_Cos(theta);
    s = Foc_Math_Sin(theta);

    *id =  c * alpha + s * beta;
    *iq = -s * alpha + c * beta;
}

/*******************************************************************************
 * Foc_InvPark - rotating -> stationary frame
 ******************************************************************************/
void Foc_InvPark(float vd, float vq, float theta, float *valpha, float *vbeta)
{
    float c, s;

    if ((valpha == NULL) || (vbeta == NULL)) {
        return;
    }

    c = Foc_Math_Cos(theta);
    s = Foc_Math_Sin(theta);

    *valpha =  c * vd - s * vq;
    *vbeta  =  s * vd + c * vq;
}

/*******************************************************************************
 * Foc_Svpwm - 7-segment sector SVPWM -> 3-phase duty (%)
 *
 *   Per-unit voltage: 1.0 pu = vbus/sqrt(3) (max linear SVPWM amplitude).
 *   u1 = vbeta_pu, u2 = 0.5*vbeta_pu + 0.866*valpha_pu, u3 = u2 - u1.
 *   Sector detection and Ta/Tb/Tc identical to STM32 INMOP SvpwmCal().
 *   duty(%) = 50 + pu*50, clamped to [2,98] (pre-driver limit).
 ******************************************************************************/
void Foc_Svpwm(float valpha, float vbeta, float vbus,
               float *du, float *dv, float *dw)
{
    float    uAlphaPu, uBetaPu;
    float    u1, u2, u3;
    float    ta, tb, tc;
    int32_t  sector;

    if ((du == NULL) || (dv == NULL) || (dw == NULL)) {
        return;
    }
    if (vbus <= 0.0f) {
        vbus = 1.0f;   /* guard: avoid div-by-zero */
    }

    /* volts -> per-unit (1.0 pu = vbus/sqrt(3)) */
    uAlphaPu = valpha * (FOC_MATH_SQRT3 / vbus);
    uBetaPu  = vbeta  * (FOC_MATH_SQRT3 / vbus);

    u1 = uBetaPu;
    u2 = uBetaPu * 0.5f + uAlphaPu * 0.8660254f;
    u3 = u2 - u1;

    /* sector detection (1..6) */
    sector = 3;
    if (u2 > 0.0f) {
        sector -= 1;
    }
    if (u3 > 0.0f) {
        sector -= 1;
    }
    if (u1 < 0.0f) {
        sector = 7 - sector;
    }

    /* active-vector duty components (per-unit) */
    switch (sector) {
    case 1:
    case 4:
        ta = u2;
        tb = u1 - u3;
        tc = -u2;
        break;
    case 2:
    case 5:
        ta = u3 + u2;
        tb = u1;
        tc = -u1;
        break;
    case 3:
    case 6:
        ta = u3;
        tb = -u3;
        tc = -(u1 + u2);
        break;
    default:
        /* abnormal sector: apply zero vector */
        ta = 0.0f;
        tb = 0.0f;
        tc = 0.0f;
        break;
    }

    /* per-unit -> duty %, center 50% */
    *du = 50.0f + ta * 50.0f;
    *dv = 50.0f + tb * 50.0f;
    *dw = 50.0f + tc * 50.0f;

    /* pre-driver limit */
    if (*du < FOC_DUTY_MIN) { *du = FOC_DUTY_MIN; }
    if (*du > FOC_DUTY_MAX) { *du = FOC_DUTY_MAX; }
    if (*dv < FOC_DUTY_MIN) { *dv = FOC_DUTY_MIN; }
    if (*dv > FOC_DUTY_MAX) { *dv = FOC_DUTY_MAX; }
    if (*dw < FOC_DUTY_MIN) { *dw = FOC_DUTY_MIN; }
    if (*dw > FOC_DUTY_MAX) { *dw = FOC_DUTY_MAX; }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
