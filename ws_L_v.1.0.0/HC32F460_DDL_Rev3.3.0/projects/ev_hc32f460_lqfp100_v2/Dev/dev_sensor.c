#include "dev_sensor.h"
#include "TickTimer.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>

// ========== 模拟模式全局变量 ==========
#ifdef SENSOR_SIMULATION_MODE
    // 模拟的是传感器原始输出电压（mV），不是ADC输入电压
    static volatile uint16_t s_u16SimSensorRawMv = 1650;
    volatile uint16_t* const g_pu16DbgSimSensorRawMv = &s_u16SimSensorRawMv;
    
    // 根据传感器原始电压计算ADC输入电压（考虑分压电路）
    static uint16_t Sensor_Sim_GetAdcVoltageMv(uint16_t u16SensorRawMv) {
#if SENSOR_VOLTAGE_DIVIDER_ENABLE
        // V_adc = V_sensor * R2 / (R1 + R2)
        return (uint16_t)((uint32_t)u16SensorRawMv * SENSOR_DIVIDER_R2 / (SENSOR_DIVIDER_R1 + SENSOR_DIVIDER_R2));
#else
        return u16SensorRawMv;
#endif
    }
    
    // 根据期望电流值设置模拟值（更直观的接口）
    static void Sensor_Sim_SetCurrent(int32_t s32CurrentMa) {
        // 反向计算：Current -> 传感器原始电压
        // V_sensor = ZeroPoint + Current * Sensitivity
        int32_t s32SensorRawMv = SENSOR_RAW_ZERO_MV + 
                                  (s32CurrentMa * SENSOR_RAW_SENSITIVITY_MV_PER_A) / 1000;
        
        if (s32SensorRawMv < 0) s32SensorRawMv = 0;
        if (s32SensorRawMv > 3300) s32SensorRawMv = 3300;
        
        s_u16SimSensorRawMv = (uint16_t)s32SensorRawMv;
    }
    
    #define SIM_ADC_RAW_VALUE(voltage)  ((uint16_t)((uint32_t)(voltage) * 4095 / 3300))
#endif

// ========== 慢速打印 ==========
#ifdef DEBUG_SENSOR_SLOW
    static uint32_t s_u32LastSlowPrintTime = 0;
    #define SLOW_PRINT_INTERVAL_MS   4000
#endif

// ========== 真实模式调试 ==========
#ifdef DEV_SENSOR_REAL
    static NonBlockingDelay_t s_stcRealDbgTimer;
    static uint8_t s_u8RealDbgTimerInit = 0;
    #define SENSOR_REAL_PRINT_INTERVAL_MS   (3000)
    
    static uint8_t Sensor_RealDbg_IsTime(void) {
        if (!s_u8RealDbgTimerInit) {
            nbDelay_Init(&s_stcRealDbgTimer, SENSOR_REAL_PRINT_INTERVAL_MS);
            nbDelay_Start(&s_stcRealDbgTimer);
            s_u8RealDbgTimerInit = 1;
            return 1;
        }
        if (nbDelay_IsComplete(&s_stcRealDbgTimer)) {
            nbDelay_Start(&s_stcRealDbgTimer);
            return 1;
        }
        return 0;
    }
#endif

// ========== 窗口调试缓冲区 ==========
#ifdef DEBUG_SENSOR_WINDOW_BUFFER
    volatile uint16_t g_u16DbgSensorBuffer[SENSOR_WINDOW_BUFFER_SIZE] = {0};
    volatile uint16_t g_u16DbgSensorBufIndex = 0;
    volatile uint16_t g_u16DbgSensorBufCount = 0;
    volatile uint8_t  g_u8DbgSensorBufOverflow = 0;
    
    volatile int32_t  g_dbg_sensor_cur_ma = 0;
    volatile int16_t  g_dbg_sensor_cur_ax100 = 0;
    volatile uint16_t g_dbg_sensor_adc_mv = 0;
    volatile uint8_t  g_dbg_sensor_alarm = 0;
    volatile uint8_t  g_dbg_sensor_mode = 0;
    volatile uint16_t g_dbg_sensor_trigger_cnt = 0;
    volatile uint16_t g_dbg_sensor_trigger_win = 0;
    volatile uint16_t g_dbg_sensor_release_win = 0;
    volatile uint32_t g_dbg_sensor_elapsed_ms = 0;
    volatile uint32_t g_dbg_sensor_trigger_ms = 0;
    volatile uint32_t g_dbg_sensor_release_ms = 0;
    volatile uint8_t  g_dbg_sensor_timer_run = 0;
    
    static void Sensor_Debug_AddToWindow(uint16_t u16VoltageMv) {
        g_u16DbgSensorBuffer[g_u16DbgSensorBufIndex] = u16VoltageMv;
        g_u16DbgSensorBufIndex++;
        if (g_u16DbgSensorBufIndex >= SENSOR_WINDOW_BUFFER_SIZE) {
            g_u16DbgSensorBufIndex = 0;
            g_u8DbgSensorBufOverflow = 1;
        }
        if (g_u16DbgSensorBufCount < SENSOR_WINDOW_BUFFER_SIZE) {
            g_u16DbgSensorBufCount++;
        }
    }
#endif

// ========== 校准静态函数 ==========
static void Sensor_CalibrateZeroInternal(Sensor_Device_t* pstcDev, uint16_t u16AdcVoltageMv) {
    if (!pstcDev) return;
    
    int32_t s32ZeroTheory = SENSOR_VOUT_ZERO_MA_INT;
    int32_t s32ZeroMeas = (int32_t)u16AdcVoltageMv;
    int32_t s32ZeroOffset = 0;

#if SENSOR_TYPE_DIFF_AMP_ENABLE
    // 差分放大器模式：理论零点就是 0mV（或者硬件上的微小失调）
    // 偏移量 = 实测电压 - 0
    s32ZeroOffset = s32ZeroMeas;
    SENSOR_DEBUG("DiffAmp Calib: Meas=%ld mV, Offset=%ld mV\r\n",
                 (long)s32ZeroMeas, (long)s32ZeroOffset);
#else
    // 霍尔传感器模式：理论零点 1650mV
    s32ZeroOffset = s32ZeroMeas - s32ZeroTheory;
    SENSOR_DEBUG("Hall Calib: V_theory=%ld mV, V_meas=%ld mV, ZeroOffset=%ld mV\r\n",
                 (long)s32ZeroTheory, (long)s32ZeroMeas, (long)s32ZeroOffset);
#endif
    
    pstcDev->stcCalibration.s32ZeroOffsetMv = s32ZeroOffset;
    pstcDev->stcCalibration.s32CalibrationValid = 0x5A5A5A5A;
}

// ========== 电流计算静态函数 ==========
static int32_t Sensor_CalcCurrentInternal(Sensor_Device_t* pstcDev, uint16_t u16AdcVoltageMv) {
    if (!pstcDev) return 0;
    
    int32_t s32CurrentMa = 0;
    int32_t s32ZeroTheory = SENSOR_VOUT_ZERO_MA_INT; // 理论零点
    int32_t s32Sensitivity = SENSOR_SENSITIVITY_INT; // 灵敏度 mV/A

#if SENSOR_TYPE_DIFF_AMP_ENABLE
    // ===== 差分放大器模式 =====
    // 差分放大器通常只有微小失调，零点近似0。但为了支持校准，依然使用 ZeroOffset
    int32_t s32ZeroOffset = pstcDev->stcCalibration.s32ZeroOffsetMv;
    
    if (pstcDev->stcCalibration.s32CalibrationValid != 0x5A5A5A5A) {
        s32ZeroOffset = 0; // 未校准时，直接取0
    }
    
    // 计算电压差：实测电压 - 零点偏移
    // 注意：差分放大器 0A 时电压接近0，不需要减去 SENSOR_VOUT_ZERO_MA_INT (通常为0)
    int32_t s32Diff = (int32_t)u16AdcVoltageMv - s32ZeroOffset;

    // 转换为 mA: V_diff / Sensitivity (V/A) * 1000
    // Sensitivity 单位是 mV/A，所以直接 s32Diff / s32Sensitivity * 1000 单位是 mA
    // 但更好的方式是： s32Diff(mV) / (Sensitivity_mV_per_A) = A
    // 然后乘以 1000 转为 mA
    if (s32Sensitivity != 0) {
        s32CurrentMa = (int32_t)(((int64_t)s32Diff * 1000) / s32Sensitivity);
    }
    
#else
    // ===== 霍尔传感器原始逻辑 =====
    int32_t s32ZeroOffset = pstcDev->stcCalibration.s32ZeroOffsetMv;
    if (pstcDev->stcCalibration.s32CalibrationValid != 0x5A5A5A5A) {
        s32ZeroOffset = 0;
    }
    
    int32_t s32Diff = (int32_t)u16AdcVoltageMv - s32ZeroTheory - s32ZeroOffset;
    
    // 转换为 mA
    if (s32Sensitivity != 0) {
        int64_t s64Temp = (int64_t)s32Diff * 1000;
        s32CurrentMa = (int32_t)(s64Temp / s32Sensitivity);
    }
#endif

    // 灵敏度微调
    if (pstcDev->stcCalibration.s16SensitivityScale != 0 && 
        pstcDev->stcCalibration.s16SensitivityScale != 100) {
        s32CurrentMa = (s32CurrentMa * pstcDev->stcCalibration.s16SensitivityScale) / 100;
    }
    
    return s32CurrentMa;
}

// ========== 从ADC读取数据 ==========
static DeviceResult_t Sensor_ReadFromAdc(Sensor_Device_t* pstcDev) {
    if (!pstcDev) return RESULT_PARAM_ERR;
    
#ifdef SENSOR_SIMULATION_MODE
    // 模拟模式：先根据传感器原始电压计算ADC输入电压
    uint16_t u16AdcVoltageMv = Sensor_Sim_GetAdcVoltageMv(s_u16SimSensorRawMv);
    pstcDev->u16AdcVoltageMv = u16AdcVoltageMv;
    pstcDev->u16AdcRawValue = SIM_ADC_RAW_VALUE(u16AdcVoltageMv);
    return RESULT_OK;
#else
    ADC_ReadResponse_t stcAdcResp;
    DeviceResult_t res = Device_Read(pstcDev->stcConfig.u8AdcDevId, &stcAdcResp, sizeof(ADC_ReadResponse_t));
    
#ifdef DEV_SENSOR_REAL
    if (Sensor_RealDbg_IsTime()) {
        SENSOR_REAL_DEBUG("ADC_Read: id=%d res=%d raw=%d mV=%d\r\n",
                          pstcDev->stcConfig.u8AdcDevId, res,
                          stcAdcResp.u16RawValue, stcAdcResp.u16VoltageMv);
    }
#endif
    
    if (res == RESULT_OK) {
        pstcDev->u16AdcRawValue = stcAdcResp.u16RawValue;
        pstcDev->u16AdcVoltageMv = stcAdcResp.u16VoltageMv;
    }
    
    return res;
#endif
}

// ========== 计算电流 ==========
static void Sensor_CalcCurrent(Sensor_Device_t* pstcDev) {
    if (!pstcDev) return;
    
    pstcDev->s32CurrentMa = Sensor_CalcCurrentInternal(pstcDev, pstcDev->u16AdcVoltageMv);
    pstcDev->s16CurrentAx100 = (int16_t)(pstcDev->s32CurrentMa / 10);
}

// ========== 过流检测 - 点数模式 ==========
static void Sensor_CheckOvercurrent_SampleCount(Sensor_Device_t* pstcDev, 
                                                   uint8_t u8IsOvercurrent, 
                                                   uint8_t u8IsNormal,
                                                   int32_t s32CurrentMa,
                                                   int32_t s32TriggerThreshold,
                                                   int32_t s32AbsThreshold,
                                                   int32_t s32AbsHysteresis) {
    Sensor_AlarmState_t* pstcAlarm = &pstcDev->stcAlarmState;
    Sensor_Config_t* pstcCfg = &pstcDev->stcConfig;
    
    if (pstcAlarm->u8OvercurrentAlarm == 0) {
        if (u8IsOvercurrent) {
            pstcAlarm->u16ConsecutiveCount++;
            if (pstcAlarm->u16ConsecutiveCount >= pstcCfg->u16TriggerWindowSize) {
                pstcAlarm->u8OvercurrentAlarm = 1;
                pstcAlarm->u16ConsecutiveCount = 0;
                
                Current_AlarmEvent_t stcEvent;
                stcEvent.s32CurrentMa = s32CurrentMa;
                stcEvent.s32ThresholdMa = pstcCfg->s32OvercurrentThresholdMa;
                stcEvent.u8IsActive = 1;
                
                GPIO_RESET(GPIO_LED_PORT, GPIO_LED_PIN);
                EventBus_Publish(TOPIC_CURRENT_ALARM, &stcEvent);
            }
        } else {
            pstcAlarm->u16ConsecutiveCount = 0;
        }
    } else {
        if (u8IsNormal) {
            pstcAlarm->u16ConsecutiveCount++;
            if (pstcAlarm->u16ConsecutiveCount >= pstcCfg->u16ReleaseWindowSize) {
                pstcAlarm->u8OvercurrentAlarm = 0;
                pstcAlarm->u16ConsecutiveCount = 0;
                
#if OVERCURRENT_CLEAR_MODE == OVERCURRENT_CLEAR_AUTO
                Current_AlarmEvent_t stcEvent;
                stcEvent.s32CurrentMa = s32CurrentMa;
                stcEvent.s32ThresholdMa = pstcCfg->s32OvercurrentThresholdMa;
                stcEvent.u8IsActive = 0;
                EventBus_Publish(TOPIC_CURRENT_ALARM, &stcEvent);
#endif
            }
        } else if (u8IsOvercurrent) {
            pstcAlarm->u16ConsecutiveCount = 0;
        } else {
            pstcAlarm->u16ConsecutiveCount = 0;
        }
    }
}

// ========== 过流检测 - 时间模式 ==========
static void Sensor_CheckOvercurrent_TimeWindow(Sensor_Device_t* pstcDev, 
                                                 uint8_t u8IsOvercurrent, 
                                                 uint8_t u8IsNormal,
                                                 int32_t s32CurrentMa,
                                                 int32_t s32TriggerThreshold,
                                                 int32_t s32AbsThreshold,
                                                 int32_t s32AbsHysteresis) {
    (void)s32AbsThreshold;
    (void)s32AbsHysteresis;
    
    Sensor_AlarmState_t* pstcAlarm = &pstcDev->stcAlarmState;
    Sensor_Config_t* pstcCfg = &pstcDev->stcConfig;
    
    if (pstcAlarm->u8OvercurrentAlarm == 0) {
        if (u8IsOvercurrent) {
            if (!pstcAlarm->u8TimerRunning) {
                nbDelay_Init(&pstcAlarm->stcTriggerTimer, pstcCfg->u32TriggerWindowMs);
                nbDelay_Start(&pstcAlarm->stcTriggerTimer);
                pstcAlarm->u8TimerRunning = 1;
            } else {
                if (nbDelay_IsComplete_noclose(&pstcAlarm->stcTriggerTimer)) {
                    pstcAlarm->u8OvercurrentAlarm = 1;
                    pstcAlarm->u8TimerRunning = 0;
                    nbDelay_Stop(&pstcAlarm->stcTriggerTimer);
                    
                    Current_AlarmEvent_t stcEvent;
                    stcEvent.s32CurrentMa = s32CurrentMa;
                    stcEvent.s32ThresholdMa = pstcCfg->s32OvercurrentThresholdMa;
                    stcEvent.u8IsActive = 1;
                    EventBus_Publish(TOPIC_CURRENT_ALARM, &stcEvent);
                }
            }
        } else {
            if (pstcAlarm->u8TimerRunning) {
                nbDelay_Stop(&pstcAlarm->stcTriggerTimer);
                pstcAlarm->u8TimerRunning = 0;
            }
        }
    } else {
        if (u8IsNormal) {
            if (!pstcAlarm->u8TimerRunning) {
                nbDelay_Init(&pstcAlarm->stcReleaseTimer, pstcCfg->u32ReleaseWindowMs);
                nbDelay_Start(&pstcAlarm->stcReleaseTimer);
                pstcAlarm->u8TimerRunning = 1;
            } else {
                if (nbDelay_IsComplete_noclose(&pstcAlarm->stcReleaseTimer)) {
                    pstcAlarm->u8OvercurrentAlarm = 0;
                    pstcAlarm->u8TimerRunning = 0;
                    nbDelay_Stop(&pstcAlarm->stcReleaseTimer);
                    
#if OVERCURRENT_CLEAR_MODE == OVERCURRENT_CLEAR_AUTO
                    Current_AlarmEvent_t stcEvent;
                    stcEvent.s32CurrentMa = s32CurrentMa;
                    stcEvent.s32ThresholdMa = pstcCfg->s32OvercurrentThresholdMa;
                    stcEvent.u8IsActive = 0;
                    EventBus_Publish(TOPIC_CURRENT_ALARM, &stcEvent);
#endif
                }
            }
        } else if (u8IsOvercurrent) {
            if (pstcAlarm->u8TimerRunning) {
                nbDelay_Stop(&pstcAlarm->stcReleaseTimer);
                pstcAlarm->u8TimerRunning = 0;
            }
        } else {
            if (pstcAlarm->u8TimerRunning) {
                nbDelay_Stop(&pstcAlarm->stcTriggerTimer);
                nbDelay_Stop(&pstcAlarm->stcReleaseTimer);
                pstcAlarm->u8TimerRunning = 0;
            }
        }
    }
}

// ========== 过流检测统一入口 ==========
static void Sensor_CheckOvercurrent(Sensor_Device_t* pstcDev) {
    if (!pstcDev) return;
    
    int32_t s32CurrentMa = pstcDev->s32CurrentMa;
    Sensor_Config_t* pstcCfg = &pstcDev->stcConfig;
    
    int32_t s32AbsCurrent = (s32CurrentMa >= 0) ? s32CurrentMa : -s32CurrentMa;
    int32_t s32AbsThreshold = (pstcCfg->s32OvercurrentThresholdMa >= 0) ? 
                              pstcCfg->s32OvercurrentThresholdMa : -pstcCfg->s32OvercurrentThresholdMa;
    int32_t s32AbsHysteresis = (pstcCfg->s32OvercurrentHysteresisMa >= 0) ?
                               pstcCfg->s32OvercurrentHysteresisMa : -pstcCfg->s32OvercurrentHysteresisMa;
    
    int32_t s32TriggerThreshold = s32AbsThreshold + s32AbsHysteresis;
    int32_t s32ReleaseThreshold = s32AbsThreshold - s32AbsHysteresis;
    if (s32ReleaseThreshold < 0) s32ReleaseThreshold = 0;
    
    uint8_t u8IsOvercurrent = (s32AbsCurrent >= s32TriggerThreshold) ? 1 : 0;
    uint8_t u8IsNormal = (s32AbsCurrent <= s32ReleaseThreshold) ? 1 : 0;
    
    if (pstcCfg->u8OvercurrentMode == OVERCURRENT_MODE_SAMPLE_COUNT) {
        Sensor_CheckOvercurrent_SampleCount(pstcDev, u8IsOvercurrent, u8IsNormal,
                                             s32CurrentMa, s32TriggerThreshold,
                                             s32AbsThreshold, s32AbsHysteresis);
    } else {
        Sensor_CheckOvercurrent_TimeWindow(pstcDev, u8IsOvercurrent, u8IsNormal,
                                           s32CurrentMa, s32TriggerThreshold,
                                           s32AbsThreshold, s32AbsHysteresis);
    }
}

// ========== 标准设备操作 ==========
DeviceResult_t Sensor_Device_Init(void* handle) {
    Sensor_Device_t* pstcDev = (Sensor_Device_t*)handle;
    if (!pstcDev) return RESULT_PARAM_ERR;
    
    memset(&pstcDev->stcCalibration, 0, sizeof(Sensor_Calibration_t));
    pstcDev->stcCalibration.s16SensitivityScale = 100;
    pstcDev->u8Calibrated = 0;
    pstcDev->u32InitTime = tickTimer_GetCount();
    
    SENSOR_DEBUG("Init: ADC Dev ID=%d\r\n", pstcDev->stcConfig.u8AdcDevId);
    
#if SENSOR_TYPE_DIFF_AMP_ENABLE
    SENSOR_DEBUG("Mode: Differential Amplifier ENABLED (Zero=0mV, Sens=100mV/A)\r\n");
    SENSOR_DEBUG("Calibration Range: %d ~ %d mV\r\n", 
                 SENSOR_CALIB_VALID_MIN_MV, SENSOR_CALIB_VALID_MAX_MV);
#else
    SENSOR_DEBUG("Mode: Hall Sensor ENABLED (Zero=1650mV, Sens=66mV/A)\r\n");
    SENSOR_DEBUG("Calibration Range: %d ~ %d mV\r\n", 
                 SENSOR_CALIB_VALID_MIN_MV, SENSOR_CALIB_VALID_MAX_MV);
#endif
    
#if SENSOR_VOLTAGE_DIVIDER_ENABLE
    SENSOR_DEBUG("  Voltage Divider ENABLED (R1=%ld Ohm, R2=%ld Ohm)\r\n", 
                 (long)SENSOR_DIVIDER_R1, (long)SENSOR_DIVIDER_R2);
    SENSOR_DEBUG("  V_zero_theory=%ld mV, Sensitivity=%ld mV/A\r\n",
                 (long)SENSOR_VOUT_ZERO_MA_INT, (long)SENSOR_SENSITIVITY_INT);
#else
    SENSOR_DEBUG("Mode: Voltage Divider DISABLED\r\n");
#endif
    
    pstcDev->u16AdcRawValue = 0;
    pstcDev->u16AdcVoltageMv = 0;
    pstcDev->s32CurrentMa = 0;
    pstcDev->s16CurrentAx100 = 0;
    
    pstcDev->stcAlarmState.u8OvercurrentAlarm = 0;
    pstcDev->stcAlarmState.u16ConsecutiveCount = 0;
    pstcDev->stcAlarmState.u8TimerRunning = 0;
    nbDelay_Init(&pstcDev->stcAlarmState.stcTriggerTimer, 0);
    nbDelay_Init(&pstcDev->stcAlarmState.stcReleaseTimer, 0);
    
    Sensor_ReadFromAdc(pstcDev);
    Sensor_CalcCurrent(pstcDev);
    
    pstcDev->u8Initialized = 1;
    pstcDev->u32LastUpdateTime = tickTimer_GetCount();
    
    SENSOR_DEBUG("Init success: ADC mV=%d, Current=%ld mA (waiting for calibration)\r\n",
                 pstcDev->u16AdcVoltageMv, (long)pstcDev->s32CurrentMa);
    
    return RESULT_OK;
}

DeviceResult_t Sensor_Device_Update(void* handle) {
    Sensor_Device_t* pstcDev = (Sensor_Device_t*)handle;
    if (!pstcDev || !pstcDev->u8Initialized) return RESULT_ERROR;
    
    DeviceResult_t res = Sensor_ReadFromAdc(pstcDev);
    if (res != RESULT_OK) {
        return res;
    }
    
    // ========== 首次有效数据校准 ==========
    if (!pstcDev->u8Calibrated) {
        uint16_t u16Voltage = pstcDev->u16AdcVoltageMv;
        uint32_t u32Now = tickTimer_GetCount();
        uint32_t u32Elapsed = u32Now - pstcDev->u32InitTime;
        
        // 使用新定义的校准阈值范围
        if (u16Voltage >= SENSOR_CALIB_VALID_MIN_MV && 
            u16Voltage <= SENSOR_CALIB_VALID_MAX_MV) {
            SENSOR_DEBUG("Detected zero current state: %d mV (elapsed=%lu ms), calibrating...\r\n",
                         u16Voltage, (unsigned long)u32Elapsed);
            Sensor_CalibrateZeroInternal(pstcDev, u16Voltage);
            pstcDev->u8Calibrated = 1;
        } else if (u32Elapsed > 5000) {
            SENSOR_DEBUG("Calibration timeout! Using theoretical zero\r\n");
            // 使用理论零点
#if SENSOR_TYPE_DIFF_AMP_ENABLE
            pstcDev->stcCalibration.s32ZeroOffsetMv = 0;
#else
            pstcDev->stcCalibration.s32ZeroOffsetMv = 0;
#endif
            pstcDev->stcCalibration.s32CalibrationValid = 0x5A5A5A5A;
            pstcDev->u8Calibrated = 1;
        } else {
            pstcDev->s32CurrentMa = 0;
            pstcDev->s16CurrentAx100 = 0;
            return RESULT_OK;
        }
    }
    
    Sensor_CalcCurrent(pstcDev);
    Sensor_CheckOvercurrent(pstcDev);
    
#ifdef DEBUG_SENSOR_WINDOW_BUFFER
    Sensor_Debug_AddToWindow(pstcDev->u16AdcVoltageMv);
    g_dbg_sensor_cur_ma = pstcDev->s32CurrentMa;
    g_dbg_sensor_cur_ax100 = pstcDev->s16CurrentAx100;
    g_dbg_sensor_adc_mv = pstcDev->u16AdcVoltageMv;
    g_dbg_sensor_alarm = pstcDev->stcAlarmState.u8OvercurrentAlarm;
#endif
    
#ifdef DEBUG_SENSOR_SLOW
    uint32_t u32Now = tickTimer_GetCount();
    if (u32Now - s_u32LastSlowPrintTime >= SLOW_PRINT_INTERVAL_MS) {
        s_u32LastSlowPrintTime = u32Now;
        int32_t s32AbsCurrent = (pstcDev->s32CurrentMa >= 0) ? pstcDev->s32CurrentMa : -pstcDev->s32CurrentMa;
        int16_t s16AbsAx100 = (pstcDev->s16CurrentAx100 >= 0) ? pstcDev->s16CurrentAx100 : -pstcDev->s16CurrentAx100;
        
        SENSOR_DEBUG_SLOW("Current=%s%ld.%02ldA (ADC Raw=%d, ADC mV=%d)%s\r\n",
            (pstcDev->s16CurrentAx100 < 0) ? "-" : "",
            (long)(s16AbsAx100 / 100),
            (long)(s16AbsAx100 % 100),
            pstcDev->u16AdcRawValue,
            pstcDev->u16AdcVoltageMv,
            pstcDev->stcAlarmState.u8OvercurrentAlarm ? " [OVERCURRENT]" : "");
    }
#endif
    
    pstcDev->u32LastUpdateTime = tickTimer_GetCount();
    return RESULT_OK;
}

DeviceResult_t Sensor_Device_Deinit(void* handle) {
    Sensor_Device_t* pstcDev = (Sensor_Device_t*)handle;
    if (!pstcDev) return RESULT_PARAM_ERR;
    pstcDev->u8Initialized = 0;
    return RESULT_OK;
}

DeviceResult_t Sensor_Device_Read(void* handle, void* data, uint32_t size) {
    Sensor_Device_t* pstcDev = (Sensor_Device_t*)handle;
    if (!pstcDev || !data) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;
    
    if (size == sizeof(Sensor_ReadResponse_t)) {
        Sensor_ReadResponse_t* pstcResp = (Sensor_ReadResponse_t*)data;
        pstcResp->s32CurrentMa = pstcDev->s32CurrentMa;
        pstcResp->s16CurrentAx100 = pstcDev->s16CurrentAx100;
        pstcResp->u16AdcRawValue = pstcDev->u16AdcRawValue;
        pstcResp->u16AdcVoltageMv = pstcDev->u16AdcVoltageMv;
        return RESULT_OK;
    }
    return RESULT_PARAM_ERR;
}

DeviceResult_t Sensor_Device_Write(void* handle, const void* data, uint32_t size) {
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t Sensor_Device_Control(void* handle, DeviceCommandData_t* pstcCmd) {
    Sensor_Device_t* pstcDev = (Sensor_Device_t*)handle;
    if (!pstcDev || !pstcCmd) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;
    
    switch (pstcCmd->cmd) {
        case CMD_SENSOR_GET_CURRENT_MA:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(int32_t)) {
                *(int32_t*)pstcCmd->response = pstcDev->s32CurrentMa;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
        case CMD_SENSOR_GET_CURRENT_AX100:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(int16_t)) {
                *(int16_t*)pstcCmd->response = pstcDev->s16CurrentAx100;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
        case CMD_SENSOR_GET_ALARM_STATUS:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)pstcCmd->response = pstcDev->stcAlarmState.u8OvercurrentAlarm;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
        case CMD_SENSOR_GET_CALIBRATION:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(Sensor_Calibration_t)) {
                *(Sensor_Calibration_t*)pstcCmd->response = pstcDev->stcCalibration;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
        default:
            return RESULT_ERROR;
    }
}

// ========== 电流传感器特定接口 ==========
int32_t Sensor_Device_GetCurrentMA(Sensor_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->s32CurrentMa;
}

int16_t Sensor_Device_GetCurrentAx100(Sensor_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->s16CurrentAx100;
}

Sensor_Device_t* Sensor_Device_Create(const Sensor_Config_t* pstcConfig) {
    if (!pstcConfig) return NULL;
    
    Sensor_Device_t* pstcDev = (Sensor_Device_t*)malloc(sizeof(Sensor_Device_t));
    if (!pstcDev) return NULL;
    
    memset(pstcDev, 0, sizeof(Sensor_Device_t));
    pstcDev->stcConfig = *pstcConfig;
    pstcDev->u8Initialized = 0;
    
    return pstcDev;
}

// ========== 校准接口实现 ==========
void Sensor_Device_CalibrateZero(Sensor_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return;
    
    Sensor_ReadFromAdc(pstcDev);
    Sensor_CalibrateZeroInternal(pstcDev, pstcDev->u16AdcVoltageMv);
    Sensor_CalcCurrent(pstcDev);
    pstcDev->u8Calibrated = 1;
    
    SENSOR_DEBUG("Manual calibration done: ZeroOffset=%ld mV\r\n",
                 (long)pstcDev->stcCalibration.s32ZeroOffsetMv);
}

void Sensor_Device_SetSensitivityScale(Sensor_Device_t* pstcDev, int16_t s16ScalePercent) {
    if (!pstcDev) return;
    if (s16ScalePercent >= 50 && s16ScalePercent <= 200) {
        pstcDev->stcCalibration.s16SensitivityScale = s16ScalePercent;
        if (pstcDev->u8Initialized) {
            Sensor_CalcCurrent(pstcDev);
        }
    }
}

void Sensor_Device_GetCalibration(Sensor_Device_t* pstcDev, Sensor_Calibration_t* pstcCal) {
    if (!pstcDev || !pstcCal) return;
    *pstcCal = pstcDev->stcCalibration;
}

// ========== 模拟模式接口 ==========
#ifdef SENSOR_SIMULATION_MODE
void Sensor_SetSimulationValue(uint16_t u16VoltageMv) {
    s_u16SimSensorRawMv = u16VoltageMv;
}

void Sensor_SetSimulationCurrent(int32_t s32CurrentMa) {
    Sensor_Sim_SetCurrent(s32CurrentMa);
}

uint16_t Sensor_GetSimulationSensorRawMv(void) {
    return s_u16SimSensorRawMv;
}
#endif

// ========== 过流告警手动清除接口 ==========
void Sensor_Device_ClearAlarm(Sensor_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return;
    if (!pstcDev->stcAlarmState.u8OvercurrentAlarm) return;
    
    pstcDev->stcAlarmState.u8OvercurrentAlarm = 0;
    pstcDev->stcAlarmState.u16ConsecutiveCount = 0;
    pstcDev->stcAlarmState.u8TimerRunning = 0;
    nbDelay_Stop(&pstcDev->stcAlarmState.stcTriggerTimer);
    nbDelay_Stop(&pstcDev->stcAlarmState.stcReleaseTimer);
    
    Current_AlarmEvent_t stcEvent;
    stcEvent.s32CurrentMa = pstcDev->s32CurrentMa;
    stcEvent.s32ThresholdMa = pstcDev->stcConfig.s32OvercurrentThresholdMa;
    stcEvent.u8IsActive = 0;
    EventBus_Publish(TOPIC_CURRENT_ALARM, &stcEvent);
}

// ========== 全局操作函数表 ==========
const DeviceOps_t g_sensor_ops = {
    .init = Sensor_Device_Init,
    .deinit = Sensor_Device_Deinit,
    .read = Sensor_Device_Read,
    .write = Sensor_Device_Write,
    .control = Sensor_Device_Control,
    .update = Sensor_Device_Update
};
