/**
 *******************************************************************************
 * @file  foc.h
 * @brief FOC runner (open-loop mode 21) on HC32F460 + TMR4 complementary PWM.
 *
 *        Mounted on the ADC1 EOCB ISR via I_RegisterFocCallback()
 *        (second callback slot, runs after CurLoop). ISR rate = FOC_ISR_HZ
 *        (20000 Hz default: PWM 10 kHz x double trigger PEAK+VALLEY).
 *
 *        Open-loop only for now; closed loop (Id/Iq PI + encoder angle)
 *        hooks are reserved, see foc.c comments.
 *******************************************************************************
 */

#ifndef __FOC_H__
#define __FOC_H__

#include "I.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Global variables for JScope / Keil Watch
 ******************************************************************************/

extern volatile float    g_foc_theta_rad;        /* electrical angle (rad) */
extern volatile float    g_foc_du;               /* U duty (%) */
extern volatile float    g_foc_dv;               /* V duty (%) */
extern volatile float    g_foc_dw;               /* W duty (%) */
extern volatile float    g_foc_valpha;           /* stationary alpha voltage (V) */
extern volatile float    g_foc_vbeta;            /* stationary beta voltage (V) */
extern volatile uint8_t  g_foc_active;           /* 1 = open-loop output enabled */
extern volatile float    g_foc_openloop_freq_hz; /* electrical freq (Hz), Keil Watch editable */
extern volatile float    g_foc_openloop_volt_v;  /* voltage amplitude (V), Keil Watch editable */

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/

/* Init: build math LUT, register Foc_Isr as the 2nd current callback.
 * Does NOT start any PWM output. Call once after I_Init()/CurLoop_Init(). */
void Foc_Init(void);

/* Start open-loop FOC: reset theta, reconfigure TMR4 to complementary,
 * enable output, set active. */
void Foc_StartOpenLoop(void);

/* Stop FOC: clear active, disable PWM output, zero duty observables. */
void Foc_Stop(void);

/* 20 kHz ISR callback (registered via I_RegisterFocCallback). Must be short:
 * no blocking, no prints, no malloc. */
void Foc_Isr(const stc_i_data_t *pData);

/* Reserved for closed loop: mechanical encoder counts -> electrical angle (rad). */
float Foc_EncoderElecAngleRad(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H__ */
