#ifndef DEV_VOLTAGE_H_
#define DEV_VOLTAGE_H_

#include "device_manager.h"
#include "dev_adc.h"
#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

// ========== 调试宏定义 ==========
// 开关在 rtt_manager.h 中统一管理：DEV_VOLTAGE

#ifdef DEV_VOLTAGE
    #define VOLTAGE_DEBUG(fmt, ...)    MAIN_D("[VOLTAGE_DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define VOLTAGE_DEBUG(fmt, ...)    ((void)0)
#endif

// ========== 模拟模式宏定义 ==========
// 开启模拟模式后，将使用手动设置的测试值代替真实ADC采样
// #define VOLTAGE_SIMULATION_MODE      // 开启模拟模式

// ========== 电压告警清除模式选择 ==========
// VOLTAGE_CLEAR_AUTO:  自动滞回清除（默认）。电压恢复正常后自动清除告警，解除电机block
// VOLTAGE_CLEAR_MANUAL: 手动清除。电压恢复正常后告警保持，需通过 Modbus 写 REG_FAULT_STATUS 清除
#define VOLTAGE_CLEAR_AUTO      0
#define VOLTAGE_CLEAR_MANUAL    1

#ifndef VOLTAGE_CLEAR_MODE
#define VOLTAGE_CLEAR_MODE      VOLTAGE_CLEAR_MANUAL   
#endif

// ========== 电压母线分压参数 ==========
// 分压网络：110kΩ + 10kΩ
// 分压比 = (110 + 10) / 10 = 12
#define VOLTAGE_DIVIDER_TOP_OHM        (150000UL)   // 上分压电阻 110kΩ
#define VOLTAGE_DIVIDER_BOTTOM_OHM     (10000UL)    // 下分压电阻 10kΩ
#define VOLTAGE_DIVIDER_RATIO          ((VOLTAGE_DIVIDER_TOP_OHM + VOLTAGE_DIVIDER_BOTTOM_OHM) / VOLTAGE_DIVIDER_BOTTOM_OHM)  

// ========== 电压母线设备命令码 ==========
#define CMD_VOLTAGE_GET_BUS_MV         (CMD_BASE_ADC + 0x10)   // 获取母线电压(mV)
#define CMD_VOLTAGE_GET_BUS_V          (CMD_BASE_ADC + 0x11)   // 获取母线电压(V，扩大100倍整数)
#ifdef VOLTAGE_SIMULATION_MODE
#define CMD_VOLTAGE_SET_SIM_VALUE      (CMD_BASE_ADC + 0x12)   // 设置模拟电压值(mV)
#endif

// ========== 电压母线设备配置 ==========
typedef struct {
    uint8_t     u8AdcDevId;             // 对应的ADC设备ID（如 ID_ADC_VOLTAGE）
    
    // 过压检测配置
    uint32_t    u32OvervoltageThresholdMv;   // 过压阈值(mV)
    uint32_t    u32OvervoltageHysteresisMv;  // 过压滞回电压(mV)
    uint8_t     u8OvervoltageTriggerCount;   // 连续超过过压阈值次数触发告警
    
    // 欠压检测配置
    uint32_t    u32UndervoltageThresholdMv;   // 欠压阈值(mV)
    uint32_t    u32UndervoltageHysteresisMv;  // 欠压滞回电压(mV)
    uint8_t     u8UndervoltageTriggerCount;   // 连续低于欠压阈值次数触发告警
} Voltage_Config_t;

// ========== 过压/欠压检测状态 ==========
typedef struct {
    // 过压检测
    uint8_t  u8OvervoltageAlarm;         // 过压告警标志（0=正常，1=告警中）
    uint8_t  u8OvervoltageCount;         // 连续超过过压阈值计数
    uint32_t u32OvervoltageReleaseMv;    // 过压解除阈值(mV) = 阈值 - 滞回
    
    // 欠压检测
    uint8_t  u8UndervoltageAlarm;        // 欠压告警标志（0=正常，1=告警中）
    uint8_t  u8UndervoltageCount;        // 连续低于欠压阈值计数
    uint32_t u32UndervoltageReleaseMv;   // 欠压解除阈值(mV) = 阈值 + 滞回
} Voltage_AlarmState_t;

// ========== 电压告警事件结构体（用于EventBus发布） ==========
typedef struct {
    uint8_t  u8AlarmType;               // 告警类型：0=过压, 1=欠压
    uint32_t u32BusVoltageMv;           // 触发时的母线电压(mV)
    uint32_t u32ThresholdMv;            // 阈值(mV)
    uint8_t  u8IsActive;                // 1=告警触发, 0=告警解除
} Voltage_AlarmEvent_t;

// 告警类型常量
#define VOLTAGE_ALARM_OVERVOLTAGE   0
#define VOLTAGE_ALARM_UNDERVOLTAGE  1

// ========== 电压母线设备结构体 ==========
typedef struct {
    Voltage_Config_t    stcConfig;      // 设备配置（含告警阈值等）
    uint8_t             u8Initialized;  // 初始化标志
    
    // 缓存数据
    uint16_t            u16AdcRawValue;     // 从ADC读取的原始值
    uint16_t            u16AdcVoltageMv;    // ADC测量点电压(mV) = 10kΩ两端电压
    uint32_t            u32BusVoltageMv;    // 实际母线电压(mV) = ADC电压 × 12
    uint16_t            u16BusVoltageVx100; // 母线电压(V)×100，用于整数打印（如 2400 = 24.00V）
    
    // 过压/欠压检测状态
    Voltage_AlarmState_t stcAlarmState;
    
    // 时间管理
    uint32_t            u32LastUpdateTime;
} Voltage_Device_t;

// ========== 读取响应结构体 ==========
typedef struct {
    uint32_t u32BusVoltageMv;       // 母线电压(mV)
    uint16_t u16BusVoltageVx100;    // 母线电压(V)×100
    uint16_t u16AdcRawValue;        // ADC原始值
    uint16_t u16AdcVoltageMv;       // ADC测量点电压(mV)
} Voltage_ReadResponse_t;

// ========== 标准设备操作（Device Manager接口） ==========
DeviceResult_t Voltage_Device_Init(void* handle);
DeviceResult_t Voltage_Device_Deinit(void* handle);
DeviceResult_t Voltage_Device_Read(void* handle, void* data, uint32_t size);
DeviceResult_t Voltage_Device_Write(void* handle, const void* data, uint32_t size);
DeviceResult_t Voltage_Device_Control(void* handle, DeviceCommandData_t* cmd);
DeviceResult_t Voltage_Device_Update(void* handle);

// ========== 电压母线特定接口 ==========
uint32_t Voltage_Device_GetBusVoltageMV(Voltage_Device_t* pstcDev);
uint16_t Voltage_Device_GetBusVoltageVx100(Voltage_Device_t* pstcDev);
Voltage_Device_t* Voltage_Device_Create(const Voltage_Config_t* pstcConfig);

// ========== 模拟模式接口 ==========
#ifdef VOLTAGE_SIMULATION_MODE
/**
 * @brief 设置模拟电压值（仅在模拟模式下有效）
 * @param u16VoltageMv 模拟的ADC测量点电压(mV)，注意不是母线电压
 * @note 母线电压 = 模拟电压 × 12
 *       例如：设置2000mV → 母线电压 = 2000×12 = 24000mV = 24.00V
 */
void Voltage_SetSimulationValue(uint16_t u16VoltageMv);
#endif

// ========== 电压告警清除接口（用于手动清除模式） ==========
/**
 * @brief 清除电压告警状态（过压或欠压）
 * @param pstcDev 电压设备指针
 * @param u8AlarmType 告警类型：VOLTAGE_ALARM_OVERVOLTAGE 或 VOLTAGE_ALARM_UNDERVOLTAGE
 * @return DeviceResult_t
 * @note 仅在 VOLTAGE_CLEAR_MODE == VOLTAGE_CLEAR_MANUAL 时使用
 *       清除后如果电压仍异常，下次 Update 会重新触发告警
 */
DeviceResult_t Voltage_Device_ClearAlarm(Voltage_Device_t* pstcDev, uint8_t u8AlarmType);

// ========== 全局操作函数表 ==========
extern const DeviceOps_t g_voltage_ops;

#endif /* DEV_VOLTAGE_H_ */
