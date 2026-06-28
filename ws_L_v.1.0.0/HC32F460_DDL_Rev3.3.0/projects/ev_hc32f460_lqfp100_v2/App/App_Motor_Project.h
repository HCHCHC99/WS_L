#ifndef APP_MOTOR_PROJECT_H_
#define APP_MOTOR_PROJECT_H_

#include "device_manager.h"
#include "EventBus.h"
#include "dev_motor.h"          // 电机设备（包含仲裁逻辑）
#include "dev_pwm.h"            // PWM设备
#include "dev_power.h"          // 电源设备
#include "dev_io.h"             // IO设备
#include "dev_hall.h"           // 霍尔设备

// 暂时注释掉未实现的设备
#include "dev_adc.h"            // ADC设备
#include "dev_voltage.h"        // 电压母线设备
#include "dev_sensor.h"         // 电流传感器设备
// #include "dev_actuator.h"       // 执行器设备
// #include "dev_can.h"            // CAN设备
#include "dev_motor_hall.h"     // 电机霍尔设备

// --- 模拟模式控制宏（1=启用模拟, 0=使用真实硬件）---
#ifndef ENABLE_SIMULATION_MODE
#define ENABLE_SIMULATION_MODE  1
#endif

// ========== 电机状态枚举 ==========
#define MOTOR_STOPPED    0
#define MOTOR_FORWARD    1
#define MOTOR_REVERSE    2
#define MOTOR_FAULT      3

// ========== 电源状态枚举 ==========
#define POWER_BOTH_OFF   0
#define POWER_POS_ON     1
#define POWER_NEG_ON     2
#define POWER_BOTH_ON    3

// ========== 霍尔状态枚举 ==========
#define HALL_NO_LIMIT    0
#define HALL_UP_LIMIT    1
#define HALL_DOWN_LIMIT  2
#define HALL_BOTH_LIMIT  3

// ========== 全局设备ID定义 ==========
#define ID_PWM_MOTOR        9   // 电机PWM设备
#define ID_PWR_POS          1   // 正电源
#define ID_PWR_NEG          2   // 负电源
#define ID_PWR_TEST1        3   // 测试电源1 (PB10)
#define ID_PWR_TEST2        4   // 测试电源2 (PA02)
#define ID_HALL_UP          5   // 上限位霍尔 (已注释)
#define ID_HALL_DOWN        6   // 下限位霍尔 (已注释)
#define ID_IO_FWD           7   // 正转IO设备
#define ID_IO_REV           8   // 反转IO设备
#define ID_MOTOR            0   // 电机仲裁设备
#define ID_MOTOR_HALL       10  // 电机霍尔
#define ID_ADC_CURRENT      11  // 电流采样
#define ID_ADC_VOLTAGE      12  // 母线电压采样
#define ID_VOLTAGE_BUS      13  // 电压母线设备（基于ADC_VOLTAGE计算）
#define ID_SENSOR_CURRENT   14  // 电流传感器设备（基于ADC_CURRENT计算）
#define ID_RTURN            15  // 圆弧转动机构设备

// ========== 硬件引脚宏定义 (新添加) ==========

// --- 正电源控制引脚 ---
#define PIN_PWR_POS_PORT        GPIO_PORT_C
#define PIN_PWR_POS_PIN         GPIO_PIN_13

// --- 负电源控制引脚 ---
#define PIN_PWR_NEG_PORT        GPIO_PORT_C
#define PIN_PWR_NEG_PIN         GPIO_PIN_14   // 假设，请根据实际修改

// --- 电机霍尔传感器引脚 ---
#define PIN_HALL_A_PORT         GPIO_PORT_A
#define PIN_HALL_A_PIN          GPIO_PIN_10
#define PIN_HALL_B_PORT         GPIO_PORT_A
#define PIN_HALL_B_PIN          GPIO_PIN_09

// --- ADC引脚 ---
#define PIN_ADC_CURRENT_PORT    GPIO_PORT_A
#define PIN_ADC_CURRENT_PIN     GPIO_PIN_05
#define PIN_ADC_CURRENT_CH      (5)           // 对应ADC通道

#define PIN_ADC_VOLTAGE_PORT    GPIO_PORT_A
#define PIN_ADC_VOLTAGE_PIN     GPIO_PIN_06
#define PIN_ADC_VOLTAGE_CH      (6)           // 对应ADC通道

// --- 电机方向IO引脚 ---
#define PIN_IO_FWD_PORT         GPIO_PORT_B   // 假设，请根据实际修改
#define PIN_IO_FWD_PIN          GPIO_PIN_00
#define PIN_IO_REV_PORT         GPIO_PORT_B
#define PIN_IO_REV_PIN          GPIO_PIN_01

// ========== 硬件配置参数宏 ==========

// --- 电机霍尔配置 ---
#define MOTOR_HALL_POLE_PAIRS   (3)
#define MOTOR_HALL_COUNT        (2)
#define MOTOR_HALL_UPDATE_MS    (1)

// --- 电机霍尔中断配置 ---
#define HALL_EIRQ_CH_A          EXTINT_CH10
#define HALL_EIRQ_CH_B          EXTINT_CH09
#define HALL_IRQN_A             INT010_IRQn
#define HALL_IRQN_B             INT009_IRQn
#define HALL_IRQ_SRC_A          INT_SRC_PORT_EIRQ10
#define HALL_IRQ_SRC_B          INT_SRC_PORT_EIRQ9
#define HALL_IRQ_PRIORITY       (2)

// --- 圆弧转动机构配置 ---
#define RTURN_REDUCTION_RATIO           (1183.0f)
#define RTURN_MAX_ANGLE                 (88.0f)
#define RTURN_MIN_ANGLE                 (-2.0f)
#define RTURN_UPDATE_INTERVAL_MS        (1)
#define RTURN_REVERSE_OUTPUT            (0)

// ========== 电压告警阈值配置（已移至 Params.h） ==========

// 保留这些宏（编译时固定，硬件相关）
#define OVERCURRENT_MODE                    OVERCURRENT_MODE_TIME_WINDOW  // 固定使用时间模式
#define CURRENT_TRIGGER_WINDOW_SIZE         (0)      // 点数模式时无用
#define CURRENT_RELEASE_WINDOW_SIZE         (0)      // 点数模式时无用

// ========== 模拟数据结构体 ==========
typedef struct {
    uint8_t sim_pwr_pos;
    uint8_t sim_pwr_neg;
    uint8_t sim_hall_up;
    uint8_t sim_hall_down;
    uint8_t sim_io_fwd;
    uint8_t sim_io_rev;
    float   sim_io_speed;
    uint16_t sim_adc_val;
    int32_t sim_motor_speed;
    uint8_t sim_motor_dir;
} SystemSim_t;

// ========== 状态指示变量 ==========
typedef struct {
    uint8_t motor_status;
    uint8_t power_status;
    uint8_t hall_status;
    uint8_t io_status;
    float   current_duty;
} SystemStatus_t;

extern SystemSim_t g_sim;
extern SystemStatus_t g_status;

void ESystem_Init(void);
void ESystem_MainLoop(void);

#if ENABLE_SIMULATION_MODE
void Sim_ProcessInput(void);
void Sim_PublishEvents(void);
#endif

#endif /* APP_MOTOR_PROJECT_H_ */
