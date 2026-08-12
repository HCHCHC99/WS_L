#include "dev_voltage.h"
#include "TickTimer.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>

// ========== 模拟模式全局变量 ==========
#ifdef VOLTAGE_SIMULATION_MODE
    // 模拟测试值（可在Keil Watch窗口中直接修改）
    // volatile防止编译器优化，确保Watch窗口能实时读取到变化
    static volatile uint16_t s_u16SimAdcVoltageMv = 2000;  // 默认2000mV（24.00V母线电压）
    // 全局指针，用于Keil Watch窗口直接观察和修改模拟电压值
    // 在Watch窗口中添加: *g_pu16DbgSimVoltageMv
    volatile uint16_t* const g_pu16DbgSimVoltageMv = &s_u16SimAdcVoltageMv;
    #define SIM_ADC_RAW_VALUE(voltage)  ((uint16_t)((uint32_t)(voltage) * 4095 / 3300))
#endif

// ========== 内部辅助函数 ==========

// 从ADC设备读取原始数据
static DeviceResult_t Voltage_ReadFromAdc(Voltage_Device_t* pstcDev) {
    if (!pstcDev) return RESULT_PARAM_ERR;
    
#ifdef VOLTAGE_SIMULATION_MODE
    // ========== 模拟模式：使用手动设置的值 ==========
    pstcDev->u16AdcVoltageMv = s_u16SimAdcVoltageMv;
    pstcDev->u16AdcRawValue = SIM_ADC_RAW_VALUE(s_u16SimAdcVoltageMv);
    
    VOLTAGE_DEBUG("SIM Mode: Voltage=%d mV, Raw=%d\r\n", 
                 pstcDev->u16AdcVoltageMv, pstcDev->u16AdcRawValue);
    return RESULT_OK;
#else
    // ========== 真实模式：从ADC设备读取 ==========
    // 通过Device Manager读取ADC设备的数据
    ADC_ReadResponse_t stcAdcResp;
    DeviceResult_t res = Device_Read(pstcDev->stcConfig.u8AdcDevId, &stcAdcResp, sizeof(ADC_ReadResponse_t));
    
    if (res == RESULT_OK) {
        pstcDev->u16AdcRawValue = stcAdcResp.u16RawValue;
        pstcDev->u16AdcVoltageMv = stcAdcResp.u16VoltageMv;
    }
    
    return res;
#endif
}

// 计算实际母线电压
// 分压比 = (110k + 10k) / 10k = 12
// 实际电压 = ADC测量点电压 × 12
static void Voltage_CalcBusVoltage(Voltage_Device_t* pstcDev) {
    if (!pstcDev) return;
    
    // 母线电压(mV) = ADC测量点电压(mV) × 分压比
    pstcDev->u32BusVoltageMv = (uint32_t)pstcDev->u16AdcVoltageMv * VOLTAGE_DIVIDER_RATIO;
    
    // 母线电压(V)×100 = 母线电压(mV) / 10
    // 例如：24000mV → 2400 (即24.00V)
    pstcDev->u16BusVoltageVx100 = (uint16_t)(pstcDev->u32BusVoltageMv / 10UL);
}

// ========== 过压/欠压检测 ==========
// 阈值、滞回、触发计数等配置值从 stcConfig 结构体中读取
// 这些值在 Voltage_Device_Create() 时由调用者传入

// 过压/欠压检测函数
static void Voltage_CheckAlarm(Voltage_Device_t* pstcDev) {
    if (!pstcDev) return;
    
    uint32_t u32BusMv = pstcDev->u32BusVoltageMv;
    Voltage_AlarmState_t* pstcAlarm = &pstcDev->stcAlarmState;
    Voltage_Config_t* pstcCfg = &pstcDev->stcConfig;
    
    // ---- 过压检测 ----
    // 注意：过压和欠压是互斥的，触发过压时自动清除欠压状态
    if (u32BusMv >= pstcCfg->u32OvervoltageThresholdMv) {
        // 当前电压超过过压阈值
        
        // 互斥保护：过压触发时，清除欠压状态
        if (pstcAlarm->u8UndervoltageAlarm) {
            pstcAlarm->u8UndervoltageAlarm = 0;
            pstcAlarm->u8UndervoltageCount = 0;
            
            Voltage_AlarmEvent_t stcReleaseEvent;
            stcReleaseEvent.u8AlarmType = VOLTAGE_ALARM_UNDERVOLTAGE;
            stcReleaseEvent.u32BusVoltageMv = u32BusMv;
            stcReleaseEvent.u32ThresholdMv = pstcCfg->u32UndervoltageThresholdMv;
            stcReleaseEvent.u8IsActive = 0;
            EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcReleaseEvent);
            
            VOLTAGE_DEBUG("UNDERVOLTAGE auto-cleared by overvoltage\r\n");
        }
        
        pstcAlarm->u8OvervoltageCount++;
        
        if (pstcAlarm->u8OvervoltageCount >= pstcCfg->u8OvervoltageTriggerCount) {
            if (!pstcAlarm->u8OvervoltageAlarm) {
                // 首次触发过压告警
                pstcAlarm->u8OvervoltageAlarm = 1;
                pstcAlarm->u32OvervoltageReleaseMv = pstcCfg->u32OvervoltageThresholdMv - pstcCfg->u32OvervoltageHysteresisMv;
                
                Voltage_AlarmEvent_t stcEvent;
                stcEvent.u8AlarmType = VOLTAGE_ALARM_OVERVOLTAGE;
                stcEvent.u32BusVoltageMv = u32BusMv;
                stcEvent.u32ThresholdMv = pstcCfg->u32OvervoltageThresholdMv;
                stcEvent.u8IsActive = 1;
                
                VOLTAGE_DEBUG("OVERVOLTAGE TRIGGERED! Bus=%lu mV, Threshold=%lu mV\r\n",
                              u32BusMv, pstcCfg->u32OvervoltageThresholdMv);
                
                EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcEvent);
            }
        }
    } else if (u32BusMv <= pstcAlarm->u32OvervoltageReleaseMv) {
        // 电压降到解除阈值以下，清除过压告警
        if (pstcAlarm->u8OvervoltageAlarm) {
            pstcAlarm->u8OvervoltageAlarm = 0;
            pstcAlarm->u8OvervoltageCount = 0;
            
            VOLTAGE_DEBUG("OVERVOLTAGE RELEASED! Bus=%lu mV, Release=%lu mV\r\n",
                          u32BusMv, pstcAlarm->u32OvervoltageReleaseMv);
            
#if VOLTAGE_CLEAR_MODE == VOLTAGE_CLEAR_AUTO
            // 自动清除模式：发布解除事件，通知电机仲裁器移除block
            Voltage_AlarmEvent_t stcEvent;
            stcEvent.u8AlarmType = VOLTAGE_ALARM_OVERVOLTAGE;
            stcEvent.u32BusVoltageMv = u32BusMv;
            stcEvent.u32ThresholdMv = pstcCfg->u32OvervoltageThresholdMv;
            stcEvent.u8IsActive = 0;
            EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcEvent);
#else
            // 手动清除模式：不发布解除事件，告警状态保持
            // 故障码和电机block需通过 Modbus 写 REG_FAULT_STATUS 手动清除
            VOLTAGE_DEBUG("  (Manual clear mode - alarm state kept)\r\n");
#endif
        }
        pstcAlarm->u8OvervoltageCount = 0;
    } else {
        // 在滞回区内，不清除计数但也不触发
        if (!pstcAlarm->u8OvervoltageAlarm) {
            pstcAlarm->u8OvervoltageCount = 0;
        }
    }
    
    // ---- 欠压检测 ----
    // 注意：过压和欠压是互斥的，触发欠压时自动清除过压状态
    if (u32BusMv <= pstcCfg->u32UndervoltageThresholdMv) {
        // 当前电压低于欠压阈值
        
        // 互斥保护：欠压触发时，清除过压状态
        if (pstcAlarm->u8OvervoltageAlarm) {
            pstcAlarm->u8OvervoltageAlarm = 0;
            pstcAlarm->u8OvervoltageCount = 0;
            
            Voltage_AlarmEvent_t stcReleaseEvent;
            stcReleaseEvent.u8AlarmType = VOLTAGE_ALARM_OVERVOLTAGE;
            stcReleaseEvent.u32BusVoltageMv = u32BusMv;
            stcReleaseEvent.u32ThresholdMv = pstcCfg->u32OvervoltageThresholdMv;
            stcReleaseEvent.u8IsActive = 0;
            EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcReleaseEvent);
            
            VOLTAGE_DEBUG("OVERVOLTAGE auto-cleared by undervoltage\r\n");
        }
        
        pstcAlarm->u8UndervoltageCount++;
        
        if (pstcAlarm->u8UndervoltageCount >= pstcCfg->u8UndervoltageTriggerCount) {
            if (!pstcAlarm->u8UndervoltageAlarm) {
                // 首次触发欠压告警
                pstcAlarm->u8UndervoltageAlarm = 1;
                pstcAlarm->u32UndervoltageReleaseMv = pstcCfg->u32UndervoltageThresholdMv + pstcCfg->u32UndervoltageHysteresisMv;
                
                Voltage_AlarmEvent_t stcEvent;
                stcEvent.u8AlarmType = VOLTAGE_ALARM_UNDERVOLTAGE;
                stcEvent.u32BusVoltageMv = u32BusMv;
                stcEvent.u32ThresholdMv = pstcCfg->u32UndervoltageThresholdMv;
                stcEvent.u8IsActive = 1;
                
                VOLTAGE_DEBUG("UNDERVOLTAGE TRIGGERED! Bus=%lu mV, Threshold=%lu mV\r\n",
                              u32BusMv, pstcCfg->u32UndervoltageThresholdMv);
                
                EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcEvent);
            }
        }
    } else if (u32BusMv >= pstcAlarm->u32UndervoltageReleaseMv) {
        // 电压升到解除阈值以上，清除欠压告警
        if (pstcAlarm->u8UndervoltageAlarm) {
            pstcAlarm->u8UndervoltageAlarm = 0;
            pstcAlarm->u8UndervoltageCount = 0;
            
            VOLTAGE_DEBUG("UNDERVOLTAGE RELEASED! Bus=%lu mV, Release=%lu mV\r\n",
                          u32BusMv, pstcAlarm->u32UndervoltageReleaseMv);
            
#if VOLTAGE_CLEAR_MODE == VOLTAGE_CLEAR_AUTO
            // 自动清除模式：发布解除事件，通知电机仲裁器移除block
            Voltage_AlarmEvent_t stcEvent;
            stcEvent.u8AlarmType = VOLTAGE_ALARM_UNDERVOLTAGE;
            stcEvent.u32BusVoltageMv = u32BusMv;
            stcEvent.u32ThresholdMv = pstcCfg->u32UndervoltageThresholdMv;
            stcEvent.u8IsActive = 0;
            EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcEvent);
#else
            // 手动清除模式：不发布解除事件，告警状态保持
            VOLTAGE_DEBUG("  (Manual clear mode - alarm state kept)\r\n");
#endif
        }
        pstcAlarm->u8UndervoltageCount = 0;
    } else {
        // 在滞回区内
        if (!pstcAlarm->u8UndervoltageAlarm) {
            pstcAlarm->u8UndervoltageCount = 0;
        }
    }
}

// ========== 标准设备操作实现 ==========

DeviceResult_t Voltage_Device_Init(void* handle) {
    Voltage_Device_t* pstcDev = (Voltage_Device_t*)handle;
    if (!pstcDev) return RESULT_PARAM_ERR;
    
    VOLTAGE_DEBUG("Init: ADC Dev ID=%d, Divider=%lu+%lu, Ratio=%lu\r\n",
                  pstcDev->stcConfig.u8AdcDevId,
                  VOLTAGE_DIVIDER_TOP_OHM,
                  VOLTAGE_DIVIDER_BOTTOM_OHM,
                  VOLTAGE_DIVIDER_RATIO);
    
    VOLTAGE_DEBUG("Init: OverV Threshold=%lu mV, Hysteresis=%lu mV, TriggerCnt=%d\r\n",
                  pstcDev->stcConfig.u32OvervoltageThresholdMv,
                  pstcDev->stcConfig.u32OvervoltageHysteresisMv,
                  pstcDev->stcConfig.u8OvervoltageTriggerCount);
    
    VOLTAGE_DEBUG("Init: UnderV Threshold=%lu mV, Hysteresis=%lu mV, TriggerCnt=%d\r\n",
                  pstcDev->stcConfig.u32UndervoltageThresholdMv,
                  pstcDev->stcConfig.u32UndervoltageHysteresisMv,
                  pstcDev->stcConfig.u8UndervoltageTriggerCount);
    
#ifdef VOLTAGE_SIMULATION_MODE
    VOLTAGE_DEBUG("SIMULATION MODE ENABLED - Use sim voltage to set test value\r\n");
    VOLTAGE_DEBUG("  Voltage 2000mV = 24.00V, 2500mV = 30.00V, 3000mV = 36.00V, 3300mV = 39.60V\r\n");
#else
    VOLTAGE_DEBUG("REAL MODE - Reading from ADC device ID=%d\r\n", pstcDev->stcConfig.u8AdcDevId);
#endif
    
    // 初始化缓存
    pstcDev->u16AdcRawValue = 0;
    pstcDev->u16AdcVoltageMv = 0;
    pstcDev->u32BusVoltageMv = 0;
    pstcDev->u16BusVoltageVx100 = 0;
    
    // 首次读取并计算
    Voltage_ReadFromAdc(pstcDev);
    Voltage_CalcBusVoltage(pstcDev);
    
    pstcDev->u8Initialized = 1;
    pstcDev->u32LastUpdateTime = tickTimer_GetCount();
    
    VOLTAGE_DEBUG("Init success: ADC Raw=%d, ADC mV=%d, Bus mV=%lu\r\n",
                  pstcDev->u16AdcRawValue,
                  pstcDev->u16AdcVoltageMv,
                  pstcDev->u32BusVoltageMv);
    
    return RESULT_OK;
}

DeviceResult_t Voltage_Device_Deinit(void* handle) {
    Voltage_Device_t* pstcDev = (Voltage_Device_t*)handle;
    if (!pstcDev) return RESULT_PARAM_ERR;
    
    VOLTAGE_DEBUG("Deinit\r\n");
    pstcDev->u8Initialized = 0;
    return RESULT_OK;
}

DeviceResult_t Voltage_Device_Read(void* handle, void* data, uint32_t size) {
    Voltage_Device_t* pstcDev = (Voltage_Device_t*)handle;
    if (!pstcDev || !data) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;
    
    if (size == sizeof(Voltage_ReadResponse_t)) {
        Voltage_ReadResponse_t* pstcResp = (Voltage_ReadResponse_t*)data;
        pstcResp->u32BusVoltageMv = pstcDev->u32BusVoltageMv;
        pstcResp->u16BusVoltageVx100 = pstcDev->u16BusVoltageVx100;
        pstcResp->u16AdcRawValue = pstcDev->u16AdcRawValue;
        pstcResp->u16AdcVoltageMv = pstcDev->u16AdcVoltageMv;
        return RESULT_OK;
    }
    
    return RESULT_PARAM_ERR;
}

DeviceResult_t Voltage_Device_Write(void* handle, const void* data, uint32_t size) {
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t Voltage_Device_Control(void* handle, DeviceCommandData_t* pstcCmd) {
    Voltage_Device_t* pstcDev = (Voltage_Device_t*)handle;
    if (!pstcDev || !pstcCmd) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;
    
    switch (pstcCmd->cmd) {
        case CMD_VOLTAGE_GET_BUS_MV:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(uint32_t)) {
                *(uint32_t*)pstcCmd->response = pstcDev->u32BusVoltageMv;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_VOLTAGE_GET_BUS_V:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(uint16_t)) {
                *(uint16_t*)pstcCmd->response = pstcDev->u16BusVoltageVx100;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
#ifdef VOLTAGE_SIMULATION_MODE
        case CMD_VOLTAGE_SET_SIM_VALUE:
            if (pstcCmd->params && pstcCmd->param_size >= sizeof(uint16_t)) {
                s_u16SimAdcVoltageMv = *(uint16_t*)pstcCmd->params;
                VOLTAGE_DEBUG("SIM: Voltage set to %d mV (Bus = %lu mV = %lu.%02luV)\r\n", 
                              s_u16SimAdcVoltageMv,
                              (uint32_t)s_u16SimAdcVoltageMv * VOLTAGE_DIVIDER_RATIO,
                              ((uint32_t)s_u16SimAdcVoltageMv * VOLTAGE_DIVIDER_RATIO) / 1000UL,
                              ((uint32_t)s_u16SimAdcVoltageMv * VOLTAGE_DIVIDER_RATIO / 10UL) % 100UL);
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
#endif
            
        default:
            return RESULT_ERROR;
    }
}

DeviceResult_t Voltage_Device_Update(void* handle) {
    Voltage_Device_t* pstcDev = (Voltage_Device_t*)handle;
    if (!pstcDev || !pstcDev->u8Initialized) return RESULT_ERROR;
    
    // 从ADC设备读取最新数据
    DeviceResult_t res = Voltage_ReadFromAdc(pstcDev);
    if (res != RESULT_OK) {
        return res;
    }
    
    // 计算实际母线电压
    Voltage_CalcBusVoltage(pstcDev);
    
    // 过压/欠压检测
    Voltage_CheckAlarm(pstcDev);
    
    // 打印结果
    // 格式：母线电压 = 24.00V (ADC原始值=2000, ADC测量点=2000mV)
    VOLTAGE_DEBUG("[VOLTAGE] Bus=%lu.%02luV (ADC Raw=%d, ADC mV=%d)%s%s\r\n",
           pstcDev->u16BusVoltageVx100 / 100UL,
           (unsigned long)pstcDev->u16BusVoltageVx100 % 100UL,
           pstcDev->u16AdcRawValue,
           pstcDev->u16AdcVoltageMv,
           pstcDev->stcAlarmState.u8OvervoltageAlarm ? " [OVERVOLTAGE]" : "",
           pstcDev->stcAlarmState.u8UndervoltageAlarm ? " [UNDERVOLTAGE]" : "");
    
    pstcDev->u32LastUpdateTime = tickTimer_GetCount();
    
    return RESULT_OK;
}

// ========== 电压母线特定接口 ==========

uint32_t Voltage_Device_GetBusVoltageMV(Voltage_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->u32BusVoltageMv;
}

uint16_t Voltage_Device_GetBusVoltageVx100(Voltage_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->u16BusVoltageVx100;
}

Voltage_Device_t* Voltage_Device_Create(const Voltage_Config_t* pstcConfig) {
    if (!pstcConfig) return NULL;
    
    Voltage_Device_t* pstcDev = (Voltage_Device_t*)malloc(sizeof(Voltage_Device_t));
    if (!pstcDev) return NULL;
    
    memset(pstcDev, 0, sizeof(Voltage_Device_t));
    pstcDev->stcConfig = *pstcConfig;
    pstcDev->u8Initialized = 0;
    
    VOLTAGE_DEBUG("Create: ADC Dev ID=%d\r\n", pstcConfig->u8AdcDevId);
    VOLTAGE_DEBUG("Create: OverV Threshold=%lu mV, Hysteresis=%lu mV, TriggerCnt=%d\r\n",
                  pstcConfig->u32OvervoltageThresholdMv,
                  pstcConfig->u32OvervoltageHysteresisMv,
                  pstcConfig->u8OvervoltageTriggerCount);
    VOLTAGE_DEBUG("Create: UnderV Threshold=%lu mV, Hysteresis=%lu mV, TriggerCnt=%d\r\n",
                  pstcConfig->u32UndervoltageThresholdMv,
                  pstcConfig->u32UndervoltageHysteresisMv,
                  pstcConfig->u8UndervoltageTriggerCount);
    
    return pstcDev;
}

// ========== 模拟模式接口 ==========
#ifdef VOLTAGE_SIMULATION_MODE
void Voltage_SetSimulationValue(uint16_t u16VoltageMv) {
    s_u16SimAdcVoltageMv = u16VoltageMv;
    VOLTAGE_DEBUG("SIM: Voltage set to %d mV via API (Bus = %lu mV)\r\n", 
                  u16VoltageMv, (uint32_t)u16VoltageMv * VOLTAGE_DIVIDER_RATIO);
}
#endif

// ========== 电压告警清除接口（用于手动清除模式） ==========
DeviceResult_t Voltage_Device_ClearAlarm(Voltage_Device_t* pstcDev, uint8_t u8AlarmType) {
    if (!pstcDev || !pstcDev->u8Initialized) return RESULT_PARAM_ERR;
    
    Voltage_AlarmState_t* pstcAlarm = &pstcDev->stcAlarmState;
    
    switch (u8AlarmType) {
    case VOLTAGE_ALARM_OVERVOLTAGE:
        if (pstcAlarm->u8OvervoltageAlarm) {
            pstcAlarm->u8OvervoltageAlarm = 0;
            pstcAlarm->u8OvervoltageCount = 0;
            
            VOLTAGE_DEBUG("ClearAlarm: Overvoltage cleared by manual command\r\n");
            
            // 发布解除事件，通知电机仲裁器移除block
            Voltage_AlarmEvent_t stcEvent;
            stcEvent.u8AlarmType = VOLTAGE_ALARM_OVERVOLTAGE;
            stcEvent.u32BusVoltageMv = pstcDev->u32BusVoltageMv;
            stcEvent.u32ThresholdMv = pstcDev->stcConfig.u32OvervoltageThresholdMv;
            stcEvent.u8IsActive = 0;
            EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcEvent);
        }
        return RESULT_OK;
        
    case VOLTAGE_ALARM_UNDERVOLTAGE:
        if (pstcAlarm->u8UndervoltageAlarm) {
            pstcAlarm->u8UndervoltageAlarm = 0;
            pstcAlarm->u8UndervoltageCount = 0;
            
            VOLTAGE_DEBUG("ClearAlarm: Undervoltage cleared by manual command\r\n");
            
            // 发布解除事件，通知电机仲裁器移除block
            Voltage_AlarmEvent_t stcEvent;
            stcEvent.u8AlarmType = VOLTAGE_ALARM_UNDERVOLTAGE;
            stcEvent.u32BusVoltageMv = pstcDev->u32BusVoltageMv;
            stcEvent.u32ThresholdMv = pstcDev->stcConfig.u32UndervoltageThresholdMv;
            stcEvent.u8IsActive = 0;
            EventBus_Publish(TOPIC_VOLTAGE_ALARM, &stcEvent);
        }
        return RESULT_OK;
        
    default:
        return RESULT_PARAM_ERR;
    }
}

// ========== 全局操作函数表 ==========
const DeviceOps_t g_voltage_ops = {
    .init = Voltage_Device_Init,
    .deinit = Voltage_Device_Deinit,
    .read = Voltage_Device_Read,
    .write = Voltage_Device_Write,
    .control = Voltage_Device_Control,
    .update = Voltage_Device_Update
};
