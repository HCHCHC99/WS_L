/**
 *******************************************************************************
 * @file  foc_math.h
 * @brief Pure FOC math library (hardware independent, float).
 *
 *        - Sin/Cos via 1024-point float LUT + linear interpolation
 *          (table built once at Foc_Math_Init(), no runtime sinf/cosf).
 *        - Clarke / Park / InvPark (equal-amplitude transform).
 *        - SVPWM 7-segment sector method -> 3-phase duty (0~100%).
 *
 *        All angles are in radians; negative / >2*PI inputs are folded
 *        internally (see Foc_Math_Sin / Foc_Math_Cos).
 *******************************************************************************
 */

#ifndef __FOC_MATH_H__
#define __FOC_MATH_H__

#include <stdint.h>
#include <stddef.h>   /* NULL */

/*=============================================================================
 * LUT configuration: 1024 points + linear interpolation
 *   Error < 0.1% of full scale; table = 4 KB RAM.
 *   Index = theta * (1024 / 2PI), wrap by masking.
 *=============================================================================*/
#define FOC_MATH_LUT_BITS   10u
#define FOC_MATH_LUT_SIZE   (1u << FOC_MATH_LUT_BITS)   /* 1024 */
#define FOC_MATH_LUT_MASK   (FOC_MATH_LUT_SIZE - 1u)

#define FOC_MATH_PI         3.141592653589793f
#define FOC_MATH_2PI        6.283185307179586f
#define FOC_MATH_HALF_PI    1.570796326794897f
#define FOC_MATH_SQRT3      1.732050807568877f

/* SVPWM duty limits (pre-driver constraint: keep 2%~98%) */
#define FOC_DUTY_MIN        2.0f
#define FOC_DUTY_MAX        98.0f

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/

/* Build the sin LUT (call once; safe to call multiple times) */
void Foc_Math_Init(void);

/* Fast LUT sin/cos. theta in radians, any value (folded internally) */
float Foc_Math_Sin(float theta);
float Foc_Math_Cos(float theta);

/* Clarke transform (equal-amplitude): alpha = ia
 *   beta = (ia + 2*ib) / sqrt(3), valid when ia+ib+ic=0 */
void Foc_Clarke(float ia, float ib, float ic,
                float *alpha, float *beta);

/* Park transform (rotating frame): theta = electrical angle (rad) */
void Foc_Park(float alpha, float beta, float theta,
              float *id, float *iq);

/* Inverse Park transform (stationary frame) */
void Foc_InvPark(float vd, float vq, float theta,
                 float *valpha, float *vbeta);

/* SVPWM 7-segment sector method.
 *   valpha/vbeta in volts, vbus = DC bus voltage (V).
 *   Outputs du/dv/dw in % (0~100), clamped to [FOC_DUTY_MIN, FOC_DUTY_MAX].
 *   Mapping: duty = 50 + (u / (vbus/sqrt(3))) * 50 */
void Foc_Svpwm(float valpha, float vbeta, float vbus,
               float *du, float *dv, float *dw);

#endif /* __FOC_MATH_H__ */
