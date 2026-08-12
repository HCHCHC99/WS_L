/**
 *******************************************************************************
 * @file  foc.h
 * @brief FOC runner (open-loop mode 21 + current-loop mode 22) on HC32F460 +
 *        TMR4 complementary PWM.
 *
 *        Mounted on the ADC1 EOCB ISR via I_RegisterFocCallback()
 *        (second callback slot, runs after CurLoop). ISR rate = FOC_ISR_HZ
 *        (20000 Hz default: PWM 10 kHz x double trigger PEAK+VALLEY).
 *
 *        Mode 21 (open loop): theta integrates g_foc_openloop_freq_hz and the
 *        voltage vector (g_foc_openloop_volt_v) is transformed by SVPWM into
 *        U/V/W duty, written through TMR4_PWM_SetDuty3Phase().
 *
 *        Mode 22 (current loop, I-F start): Foc_StartCurrentLoop() runs the
 *        current loop from the first tick with a synthetic angle (I-F):
 *          ia/ib/ic -> Foc_Clarke -> Foc_Park(theta) -> id/iq
 *          -> vd = PI_id(0, id), vq = PI_iq(iq_ref, iq)
 *          -> Foc_InvPark(vd, vq, theta) -> Foc_Svpwm -> TMR4 duty.
 *******************************************************************************
 */

#ifndef __FOC_H__
#define __FOC_H__

#include "I.h"
#include "../Utils/dev_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * FOC run modes (g_foc_mode)
 *=============================================================================*/
#define FOC_MODE_NONE      0u   /* FOC stopped */
#define FOC_MODE_OPENLOOP  1u   /* comm_mode 21: open-loop V/f */
#define FOC_MODE_CURLOOP   2u   /* comm_mode 22: encoder FOC current loop */
#define FOC_MODE_ALIGN     3u   /* comm_mode 23: standstill electrical alignment */

/*******************************************************************************
 * Global variables for JScope / Keil Watch
 ******************************************************************************/

extern volatile float    g_foc_theta_rad;        /* electrical angle (rad) */
extern volatile float    g_foc_du;               /* U duty (%) */
extern volatile float    g_foc_dv;               /* V duty (%) */
extern volatile float    g_foc_dw;               /* W duty (%) */
extern volatile float    g_foc_valpha;           /* stationary alpha voltage (V) */
extern volatile float    g_foc_vbeta;            /* stationary beta voltage (V) */
extern volatile uint8_t  g_foc_active;           /* 1 = FOC output enabled */
extern volatile float    g_foc_openloop_freq_hz; /* electrical freq (Hz), Keil Watch editable */
extern volatile float    g_foc_openloop_volt_v;  /* voltage amplitude (V), Keil Watch editable */

/* Current-loop run mode / observables (all volatile, Keil Watch editable) */
extern volatile uint8_t  g_foc_mode;             /* 0=idle, 1=open-loop, 2=current-loop */
extern volatile int8_t   g_foc_cur_sign;         /* current sign correction (+1/-1) */
extern volatile float    g_foc_iq_ref_cmd_ma;    /* Iq target (mA), Keil Watch editable */
extern volatile float    g_foc_iq_ref_ma;        /* ramped Iq setpoint actually used (mA) */
extern volatile float    g_foc_id_ma;            /* d-axis feedback (mA) */
extern volatile float    g_foc_iq_ma;            /* q-axis feedback (mA) */
extern volatile float    g_foc_vd;               /* d-axis PI output (V) */
extern volatile float    g_foc_vq;               /* q-axis PI output (V) */
extern volatile uint8_t  g_foc_align_state;      /* 0=idle, 1=aligning, 2=running */
extern volatile uint8_t  g_foc_fault;            /* 1 = over-current fault */

/* Over-current limit (A, Keil Watch editable) and trip diagnostic (mA) */
extern volatile float    g_foc_oc_limit_a;
extern volatile float    g_foc_fault_i_ma;
/* Gentle handover / voltage envelope / feedback filter (Keil Watch editable) */
extern volatile float    g_foc_vmax_v;       /* current-loop max |v| (V) */
extern volatile float    g_foc_vramp_v_s;    /* voltage envelope ramp (V/s) */
extern volatile float    g_foc_cur_fb_alpha; /* EMA weight on id/iq (1.0 = off) */
extern volatile float    g_foc_iq_ramp_ma_s; /* Iq soft-start ramp (mA/s) */
extern volatile float    g_foc_vlim_v;       /* current voltage envelope (V) */

/* I-F start observables */
extern volatile float    g_foc_if_freq_hz;   /* current I-F electrical frequency (Hz) */
extern volatile float    g_foc_if_diff_rad;  /* encoder-elec angle - synthetic angle (rad) */
extern volatile uint8_t  g_foc_if_sync;      /* 1 = synchronized, handed over to encoder */
/* Align calibration (mode 23) */
extern volatile float    g_foc_align_id_ma;  /* d-axis current during align (mA), Watch tunable */
extern volatile int32_t  g_foc_align_offset; /* recorded encoder electrical-zero count */

/* Current-loop PI configs (volatile, Keil Watch can tune kp/ki live) */
extern pid_config_t g_foc_pid_id_cfg;
extern pid_config_t g_foc_pid_iq_cfg;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/

/* Init: build math LUT, register Foc_Isr as the 2nd current callback.
 * Does NOT start any PWM output. Call once after I_Init()/CurLoop_Init(). */
void Foc_Init(void);

/* Start open-loop FOC (comm_mode 21): reset theta, reconfigure TMR4 to
 * complementary, enable output, set active. */
void Foc_StartOpenLoop(void);

/* Start current-loop FOC (comm_mode 22): clear fault, reset PIs, enter
 * I-F start (current-controlled, synthetic angle), hand over to encoder angle
 * when synchronized, then RUN (encoder angle + Id/Iq PI). */
void Foc_StartCurrentLoop(void);
/* Start standstill electrical alignment (comm_mode 23): lock rotor to the
 * d-axis with a small current, record the encoder electrical zero, release. */
void Foc_StartAlign(void);

/* Stop FOC: clear active, disable PWM output, zero duty observables. */
void Foc_Stop(void);

/* Current-loop state (0=idle, 1=aligning, 2=running) */
uint8_t Foc_GetState(void);

/* 1 = FOC output enabled (open-loop or current-loop) */
uint8_t Foc_IsRunning(void);

/* 20 kHz ISR callback (registered via I_RegisterFocCallback). Must be short:
 * no blocking, no prints, no malloc. */
void Foc_Isr(const stc_i_data_t *pData);

/* Mechanical encoder counts -> electrical angle (rad). */
float Foc_EncoderElecAngleRad(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H__ */
