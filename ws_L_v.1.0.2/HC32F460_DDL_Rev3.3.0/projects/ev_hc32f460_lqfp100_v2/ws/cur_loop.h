/**
 *******************************************************************************
 * @file  cur_loop.h
 * @brief 25kHz current PI loop for six-step BLDC (learning)
 *        Mounted on ADC1 EOCB ISR (25kHz, 1:1 with PWM) via I_RegisterCallback.
 *        PI runs on every ADC sample. Feedback = active high-side phase current
 *        (fixed state table) with 5-tap sliding average, dt = Timer6 us timestamp.
 *        Output = Commutation_SetActiveDuty().
 *        Active only in COMM_RUNNER_CURLOOP_FW mode while hall FSM is RUNNING.
 *******************************************************************************
 */

#ifndef __CUR_LOOP_H__
#define __CUR_LOOP_H__

#include "../Utils/dev_pid.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Keil Watch: current setpoint (mA, signed) */
extern volatile float g_i_ref_ma;

/* J-Scope observability (updated in ADC ISR at control rate) */
extern volatile float g_scope_i_ref;
extern volatile float g_scope_i_fb;
extern volatile float g_scope_i_duty;
extern volatile float g_scope_i_err;
extern volatile float g_scope_i_ol;

/* Current-loop PID config (volatile, Keil Watch tunable) */
extern pid_config_t g_cur_pid_cfg;

/* Register ADC EOCB callback. Call once after I_Init(). */
void CurLoop_Init(void);

void  CurLoop_SetRef(float ma);
float CurLoop_GetRef(void);

#ifdef __cplusplus
}
#endif

#endif /* __CUR_LOOP_H__ */
