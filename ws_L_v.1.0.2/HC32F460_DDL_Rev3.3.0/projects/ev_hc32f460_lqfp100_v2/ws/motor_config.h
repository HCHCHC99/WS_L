/**
 *******************************************************************************
 * @file  motor_config.h
 * @brief Motor control frequency configuration (single source of truth)
 *******************************************************************************
 */

#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

/* ============================================================================
 * 电机控制频率总配置：改这一个宏即可
 *
 *   MOTOR_PWM_FREQ_HZ = PWM 频率 = ADC 采样频率 = 电流环频率（1:1�??
 *
 *   常用值：
 *     10000u = 10 kHz
 *     25000u = 25 kHz
 *     50000u = 50 kHz
 *   其它 8kHz~100kHz 均可（受 ADC 转换时间 / 中断负载限制�??
 * ==========================================================================*/
#define MOTOR_PWM_FREQ_HZ   20000u

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
#endif /* __MOTOR_CONFIG_H__ */
