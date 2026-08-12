/**
 *******************************************************************************
 * @file  motor_config.h
 * @brief Motor control frequency configuration (single source of truth)
 *******************************************************************************
 */

#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

/* ============================================================================
 * Motor control frequency master config: change this one macro.
 *
 *   MOTOR_PWM_FREQ_HZ = PWM switching frequency.
 *   INMOP-style double update (branch inmop_cur_loop):
 *     PWM = 10kHz, ADC/current-loop read = 2x = 20kHz
 *     (TMR4 SCMP0 @ PEAK + SCMP2 @ VALLEY both trigger ADC1 SEQ_B EOCB ISR).
 *
 *   Other values 8kHz~100kHz are possible (limited by ADC conversion time /
 *   ISR load). For 1x sampling per PWM period, keep a single peak trigger
 *   (I_ADC_HARDTRIG = EVT0 only).
 * ==========================================================================*/
#define MOTOR_PWM_FREQ_HZ   10000u

/* Hall sensor enable: 0 = hall disabled (PA8/9/10 freed for encoder ABZ) */
#define MOTOR_HALL_ENABLE   0

/* ============================================================================
 * 环拓扑配置（改这里切换控制结构）
 * ==========================================================================*/
/* 位置环（预留，默认关 - 需位置反馈如编码器） */
#define MOTOR_LOOP_POSITION_ENABLE   0
#define POS_REF_EXTERNAL             0
#define MOTOR_POS_REF_SRC            POS_REF_EXTERNAL

/* 速度环 */
#define MOTOR_LOOP_SPEED_ENABLE      1
#define SPD_REF_TARGET_RPM           0
#define SPD_REF_FROM_POS             1
#define MOTOR_SPD_REF_SRC            SPD_REF_TARGET_RPM

/* 电流环 */
#define MOTOR_LOOP_CURRENT_ENABLE    1
#define CUR_REF_FIXED                0
#define CUR_REF_FROM_SPEED           1
#define MOTOR_CUR_REF_SRC            CUR_REF_FROM_SPEED

/* 占空比来源 */
#define DUTY_DIRECT                  0
#define DUTY_FROM_CURRENT            1
#define MOTOR_DUTY_SRC               DUTY_FROM_CURRENT

/* ============================================================================
 * FOC parameters (comm_mode 21 = FOC open-loop)
 * ==========================================================================*/
#define MOTOR_FOC_ENABLE        1          /* 1 = compile/enable FOC mode 21 */
#define FOC_POLE_PAIRS          10         /* motor pole pairs */
#define FOC_VBUS_V              12.0f      /* DC bus voltage (V): board changed from 24V to 12V */
#define FOC_OPENLOOP_FREQ_HZ    5.0f       /* default open-loop electrical freq (Hz) */
#define FOC_OPENLOOP_VOLT_V     0.4f       /* open-loop voltage (V): ~1.2A peak (R=0.33) / ~1.7A (R=0.23), well within +-10A sensor range */
#define FOC_OPENLOOP_VOLT_MAX  1.5f       /* HARD CAP: open-loop phase voltage never exceeds this (overheat protection) */
#define FOC_ISR_HZ              20000      /* FOC ISR rate (Hz): 10k PWM x double trigger = 20k */
#define FOC_DEADTIME_NS         500u       /* complementary PWM dead-time (ns) */
/* ============================================================================
 * FOC current-loop parameters (comm_mode 22 = FOC current loop)
 * ==========================================================================*/
#define FOC_IQ_REF_MA        100         /* Iq target (mA), Keil Watch editable */
#define FOC_IQ_RAMP_MA_S     100         /* Iq soft-start ramp rate (mA/s) */

/* I-F start (mode 22): current-controlled startup with synthetic angle */
#define FOC_IF_FREQ_RAMP_HZ_S   2.0f     /* synthetic frequency ramp (Hz/s) */
#define FOC_IF_SYNC_MIN_HZ      2.0f     /* min frequency before sync detection */
#define FOC_IF_SYNC_WIN_CNT     2000u    /* sync window length (samples, 100ms @20k) */
#define FOC_IF_SYNC_BAND_RAD    0.10f    /* allowed angle-diff band per window (rad) */
#define FOC_IF_SYNC_GOOD_WINS   2u       /* consecutive good windows required */
#define FOC_IF_TIMEOUT_MS       10000u   /* start timeout -> fault code 2 */
/* Align calibration (comm_mode 23) - INMOP-style two-step (beta -> alpha) FIXED voltage */
#define FOC_ALIGN_VOLT_V     0.4f       /* fixed align voltage (V): 0.4V/0.15ohm ~ 2.7A (< OC 4A), Watch tunable */
#define FOC_ALIGN_BETA_MS    1000       /* step1: hold on beta axis (ms), INMOP-style */
#define FOC_ALIGN_STABLE_MS  300        /* encoder-stable window to declare lock (ms) */
#define FOC_ALIGN_TIMEOUT_MS 3000       /* step2 (alpha) timeout before fault code 3 */
#define FOC_ALIGN_HOLD_MS    2000       /* hold lock before releasing output (ms) - long enough to read id/iq */
#define FOC_OC_LIMIT_A       4.0f        /* over-current trip default (A, per phase; runtime: g_foc_oc_limit_a) */
#define FOC_CUR_SIGN         -1          /* current sign correction: -1 confirmed by mode-23 align (id must be positive) */
#define FOC_PI_KP            0.05f       /* current PI Kp (V/A) - low-R motor: 100mA needs only ~13mV */
#define FOC_PI_KI            50.0f       /* current PI Ki (1/s) - low-R motor */
#define FOC_PI_UMAX_V        0.5f        /* current PI output clamp (V): 0.5V/R=~4A worst case; normal op ~13mV */

#define FOC_VMAX_V            0.5f       /* current-loop max voltage magnitude (V), Watch tunable - low-R motor */
#define FOC_VRAMP_V_S         0.2f       /* voltage envelope ramp rate (V/s) - gentle for low-R motor */
#define FOC_CUR_FB_ALPHA      0.3f       /* EMA weight on id/iq feedback (1.0 = off) */
#endif /* __MOTOR_CONFIG_H__ */
