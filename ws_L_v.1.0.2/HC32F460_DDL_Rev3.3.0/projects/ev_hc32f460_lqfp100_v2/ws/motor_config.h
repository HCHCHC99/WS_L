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
#define FOC_OPENLOOP_VOLT_V     2.0f       /* default open-loop voltage amplitude (V), start small */
#define FOC_ISR_HZ              20000      /* FOC ISR rate (Hz): 10k PWM x double trigger = 20k */
#define FOC_DEADTIME_NS         500u       /* complementary PWM dead-time (ns) */
/* ============================================================================
 * FOC current-loop parameters (comm_mode 22 = FOC current loop)
 * ==========================================================================*/
#define FOC_IQ_REF_MA        300         /* Iq target (mA), Keil Watch editable */
#define FOC_IQ_RAMP_MA_S     200         /* Iq soft-start ramp rate (mA/s) */
#define FOC_ALIGN_VOLT_V     2.0f        /* rotor alignment voltage (V) on alpha axis */
#define FOC_ALIGN_TIME_MS    1000        /* rotor alignment duration (ms) */
#define FOC_OC_LIMIT_A       2.0f        /* over-current trip (A, per phase) */
#define FOC_CUR_SIGN         1           /* current sign correction (+1/-1) */
#define FOC_PI_KP            0.3f        /* current PI proportional gain (V/A) */
#define FOC_PI_KI            1000.0f     /* current PI integral gain (1/s) */
#define FOC_PI_UMAX_V        6.0f        /* current PI output clamp (V) on 12V bus */
#endif /* __MOTOR_CONFIG_H__ */
