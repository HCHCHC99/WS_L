#ifndef __MOTOR_HALL_H__
#define __MOTOR_HALL_H__

#include "hc32_ll.h"
#include "Adapter.h"

/* ========== 电机配置参数（仅转速、转向相关） ========== */

/**
 * @brief 霍尔传感器配置
 */
typedef struct {
    /* GPIO配置 */
    uint8_t hall_a_port;        /* GPIO_PORT_A 等 */
    uint16_t hall_a_pin;        /* GPIO_PIN_xx */
    uint8_t hall_b_port;
    uint16_t hall_b_pin;
    
    /* 中断配置 */
    uint32_t eirq_ch_a;         /* EXTINT_CHxx */
    uint32_t eirq_ch_b;
    uint8_t irqn_a;             /* INTxxx_IRQn */
    uint8_t irqn_b;
    uint32_t irq_src_a;         /* INT_PORT_EIRQx */
    uint32_t irq_src_b;
    uint8_t irq_priority;
    
    /* 电机参数（转速转向相关） */
    uint8_t pole_pairs;
    uint8_t hall_count;
    uint16_t custom_pulses_per_rev;
    
} motor_hall_config_t;


/* ========== 默认配置示例（原电机配置 - PA9, PA10） ========== */
#define DEFAULT_HALL_A_PORT      GPIO_PORT_A
#define DEFAULT_HALL_A_PIN       GPIO_PIN_09
#define DEFAULT_HALL_B_PORT      GPIO_PORT_A
#define DEFAULT_HALL_B_PIN       GPIO_PIN_10

#define DEFAULT_HALL_A_EIRQ_CH   EXTINT_CH09
#define DEFAULT_HALL_B_EIRQ_CH   EXTINT_CH10
#define DEFAULT_HALL_A_IRQN      INT009_IRQn
#define DEFAULT_HALL_B_IRQN      INT010_IRQn
#define DEFAULT_HALL_A_IRQ_SRC   INT_PORT_EIRQ9
#define DEFAULT_HALL_B_IRQ_SRC   INT_PORT_EIRQ10

#define DEFAULT_HALL_IRQ_PRIORITY DDL_IRQ_PRIORITY_02

/* 默认电机参数 */
#define DEFAULT_POLE_PAIRS       (3)     
#define DEFAULT_HALL_COUNT       (2)     

/* 自动计算每转脉冲数：极对数 × 霍尔数 × 2（双边沿） */
#define CALC_PULSES_PER_REV(pole_pairs, hall_count) ((pole_pairs) * (hall_count) * 2)

/* ========== 方向状态枚举 ========== */
typedef enum {
    MOTOR_DIRECTION_NONE = 0,
    MOTOR_DIRECTION_FORWARD,
    MOTOR_DIRECTION_REVERSE,
    MOTOR_DIRECTION_STOP,
} motor_direction_t;

/* ========== 霍尔工作状态枚举 ========== */
typedef enum {
    HALL_STATUS_NONE = 0,
    HALL_STATUS_A_ONLY,
    HALL_STATUS_B_ONLY,
    HALL_STATUS_BOTH,
    HALL_STATUS_ERROR
} hall_working_status_t;

/* ========== 霍尔句柄（不透明指针） ========== */
typedef struct motor_hall_handle_t* motor_hall_handle_t;

/* ========== 创建/销毁接口 ========== */

motor_hall_handle_t motor_hall_create(const motor_hall_config_t* config);
void motor_hall_destroy(motor_hall_handle_t handle);

/* ========== 初始化/更新接口 ========== */

void motor_hall_system_init(void);
void motor_hall_start(motor_hall_handle_t handle);
void motor_hall_stop(motor_hall_handle_t handle);
void motor_hall_update(motor_hall_handle_t handle);

/* ========== 转速相关接口 ========== */

float motor_hall_get_rpm(motor_hall_handle_t handle);
float motor_hall_get_rpm_raw(motor_hall_handle_t handle);
uint32_t motor_hall_get_pulse_interval_us(motor_hall_handle_t handle);
uint8_t motor_hall_is_running(motor_hall_handle_t handle);
uint8_t motor_hall_is_stalled(motor_hall_handle_t handle);

/* ========== 方向相关接口 ========== */

motor_direction_t motor_hall_get_direction(motor_hall_handle_t handle);
uint8_t motor_hall_get_direction_confidence(motor_hall_handle_t handle);
uint8_t motor_hall_is_direction_changed(motor_hall_handle_t handle);

/* ========== 霍尔计数接口 ========== */

uint32_t motor_hall_get_hall_a_count(motor_hall_handle_t handle);
uint32_t motor_hall_get_hall_b_count(motor_hall_handle_t handle);
uint32_t motor_hall_get_total_pulse_count(motor_hall_handle_t handle);
void motor_hall_reset_counts(motor_hall_handle_t handle);
hall_working_status_t motor_hall_get_status(motor_hall_handle_t handle);
uint8_t motor_hall_get_active_hall_count(motor_hall_handle_t handle);
uint16_t motor_hall_get_pulses_per_rev(motor_hall_handle_t handle);
uint8_t motor_hall_get_pole_pairs(motor_hall_handle_t handle);

#endif /* MOTOR_HALL_H */
