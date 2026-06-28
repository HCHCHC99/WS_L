#ifndef __DEV_COMMUTATION_H__
#define __DEV_COMMUTATION_H__

#include "tmr4_pwm.h"

/*=============================================================================
 * Six-step commutation states
 *
 *   Macro             High-side(PWM)  Low-side(ON)  OFF
 *   -----             --------------  ------------  ---
 *   COMM_STEP_UH_VL   U (SYNC, duty)  V (SYNC, 2%)  W (COMP, 50%)
 *   COMM_STEP_UH_WL   U (SYNC, duty)  W (SYNC, 2%)  V (COMP, 50%)
 *   COMM_STEP_VH_WL   V (SYNC, duty)  W (SYNC, 2%)  U (COMP, 50%)
 *   COMM_STEP_VH_UL   V (SYNC, duty)  U (SYNC, 2%)  W (COMP, 50%)
 *   COMM_STEP_WH_UL   W (SYNC, duty)  U (SYNC, 2%)  V (COMP, 50%)
 *   COMM_STEP_WH_VL   W (SYNC, duty)  V (SYNC, 2%)  U (COMP, 50%)
 *
 * SDH21263 pre-driver truth table (per phase):
 *   H,H -> high-side ON   L,L -> low-side ON   H!=L -> interlock OFF
 *=============================================================================*/

/* Pre-driver duty limits */
#define COMM_DUTY_MIN_F  2.0f    /* 2% */
#define COMM_DUTY_MAX_F  98.0f   /* 98% */
#define COMM_DUTY_OFF_F  50.0f   /* 50% (complementary OFF) */

/* Commutation step macros â€ freq_hz (e.g. 50000), duty_pct (e.g. 95.0f) */
#define COMM_STEP_UH_VL(f, d)  Commutation_Step(0, (f), (d))
#define COMM_STEP_UH_WL(f, d)  Commutation_Step(1, (f), (d))
#define COMM_STEP_VH_WL(f, d)  Commutation_Step(2, (f), (d))
#define COMM_STEP_VH_UL(f, d)  Commutation_Step(3, (f), (d))
#define COMM_STEP_WH_UL(f, d)  Commutation_Step(4, (f), (d))
#define COMM_STEP_WH_VL(f, d)  Commutation_Step(5, (f), (d))

/* Initialize all 3 phases to complementary OFF */
void Commutation_Init(void);

/* Execute one commutation step (0-5), freq_hz, duty_pct clamped to 2%~98% */
void Commutation_Step(uint8_t state, uint16_t freq_hz, float duty_pct);

/* All phases to complementary OFF */
void Commutation_Stop(void);

/* Step metadata for debug display */
const char* Commutation_GetHighPhase(uint8_t step);   /* "UH"/"VH"/"WH" */
const char* Commutation_GetLowPhase(uint8_t step);    /* "UL"/"VL"/"WL" */
uint16_t    Commutation_GetFieldAngle(uint8_t step);  /* 0-360 degrees */

#endif /* __DEV_COMMUTATION_H__ */
