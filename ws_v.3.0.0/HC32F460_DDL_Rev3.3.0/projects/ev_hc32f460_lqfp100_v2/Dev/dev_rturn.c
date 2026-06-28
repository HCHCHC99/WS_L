#include "dev_rturn.h"
#include "TickTimer.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "dev_sensor.h"
#include "dev_motor.h"   

static uint32_t s_u32LastLockPrintTime = 0;
static uint32_t s_u32LastLockDebugPrintTime = 0;
static uint32_t s_u32LastLimitPrintTime = 0;
#define LOCK_PRINT_INTERVAL_MS         2000
#define LOCK_DEBUG_PRINT_INTERVAL_MS   4000
#define LIMIT_PRINT_INTERVAL_MS        4000

// ========== Keil Watch调试全局变量 ==========
volatile float    g_fDbgRTurnAngle       = 0.0f;
volatile float    g_fDbgRTurnSpeed       = 0.0f;
volatile uint8_t  g_u8DbgRTurnDir        = 0;
volatile uint8_t  g_u8DbgRTurnDesiredDir = 0;
volatile uint8_t  g_u8DbgRTurnLockedDir  = 0;
volatile uint8_t  g_u8DbgRTurnLockActive = 0;
volatile uint8_t  g_u8DbgRTurnCalibrated = 0;
volatile uint8_t  g_u8DbgRTurnLimitTrig  = 0;

// ========== 内部辅助函数 ==========
static DeviceResult_t RTurn_GetDesiredDirection(RTurn_Device_t* pstcDev, uint8_t* pu8MotorDir) {
    if (!pstcDev || !pu8MotorDir) return RESULT_PARAM_ERR;
    
    Motor_StateInfo_t stcMotorState;
    DeviceResult_t res = Device_Read(pstcDev->stcConfig.u8MotorArbiterDevId, 
                                      &stcMotorState, sizeof(Motor_StateInfo_t));
    
    if (res == RESULT_OK) {
        *pu8MotorDir = (uint8_t)stcMotorState.desired_dir;
        RTURN_DEBUG("Read motor state: desired_dir=%d, state=%d\r\n", 
                    stcMotorState.desired_dir, stcMotorState.state);
        return RESULT_OK;
    }
    
    RTURN_DEBUG("Failed to read motor state, res=%d\r\n", res);
    return RESULT_ERROR;
}

static uint8_t RTurn_ConvertMotorDirToRTurnDir(uint8_t u8MotorDir, uint8_t u8ReverseOutput) {
    if (u8MotorDir == MOTOR_DIRECTION_NONE) {
        return RTURN_DIR_STOP;
    }
    
    if (u8ReverseOutput) {
        if (u8MotorDir == MOTOR_DIRECTION_FORWARD) {
            return RTURN_DIR_REVERSE;
        } else if (u8MotorDir == MOTOR_DIRECTION_REVERSE) {
            return RTURN_DIR_FORWARD;
        }
    } else {
        if (u8MotorDir == MOTOR_DIRECTION_FORWARD) {
            return RTURN_DIR_FORWARD;
        } else if (u8MotorDir == MOTOR_DIRECTION_REVERSE) {
            return RTURN_DIR_REVERSE;
        }
    }
    
    return RTURN_DIR_STOP;
}

static DeviceResult_t RTurn_GetMotorSpeedAndDir(RTurn_Device_t* pstcDev, float* pfRpm, uint8_t* pu8MotorDir) {
    if (!pstcDev) return RESULT_PARAM_ERR;
    
    uint8_t u8DevId = pstcDev->stcConfig.u8MotorHallDevId;
    
    if (pfRpm) {
        DeviceResult_t res = Device_Read(u8DevId, pfRpm, sizeof(float));
        if (res != RESULT_OK) {
            RTURN_DEBUG("Read RPM failed, res=%d\r\n", res);
        }
    }
    
    if (pu8MotorDir) {
        DeviceCommandData_t cmd;
        cmd.cmd = CMD_MOTOR_HALL_GET_DIRECTION;
        cmd.params = NULL;
        cmd.param_size = 0;
        cmd.response = pu8MotorDir;
        cmd.response_size = sizeof(uint8_t);
        
        DeviceResult_t res = Device_Control(u8DevId, &cmd);
        if (res != RESULT_OK) {
            RTURN_DEBUG("Read motor direction failed, res=%d\r\n", res);
        }
    }
    
    return RESULT_OK;
}

static float RTurn_RpmToAngularSpeed(float fRpm, float fReductionRatio) {
    if (fReductionRatio <= 0) return 0;
    return fRpm * 360.0f / fReductionRatio / 60.0f;
}

// 读取传感器告警状态（0=正常, 1=过流告警）
static uint8_t RTurn_ReadSensorAlarm(RTurn_Device_t* pstcDev) {
    if (!pstcDev) return 0;
    
    uint8_t u8Alarm = 0;
    DeviceCommandData_t cmd;
    cmd.cmd = CMD_SENSOR_GET_ALARM_STATUS;
    cmd.params = NULL;
    cmd.param_size = 0;
    cmd.response = &u8Alarm;
    cmd.response_size = sizeof(uint8_t);
    
    DeviceResult_t res = Device_Control(pstcDev->stcConfig.u8SensorDevId, &cmd);
    if (res != RESULT_OK) {
        RTURN_DEBUG("Read sensor alarm failed, res=%d\r\n", res);
        return 0;
    }
    
    return u8Alarm;
}

static void RTurn_UpdateAngle(RTurn_Device_t* pstcDev) {
    if (!pstcDev) return;
    
    uint32_t u32Now = tickTimer_GetCount();
    uint32_t u32DeltaMs = u32Now - pstcDev->u32LastAngleTime;
    
    if (u32DeltaMs == 0) return;
    
    float fRpm = 0;
    uint8_t u8MotorDir = MOTOR_DIRECTION_NONE;
    DeviceResult_t res = RTurn_GetMotorSpeedAndDir(pstcDev, &fRpm, &u8MotorDir);
    
    if (res != RESULT_OK) {
        RTURN_DEBUG("Failed to get motor speed and direction\r\n");
        pstcDev->u32LastAngleTime = u32Now;
        return;
    }
    
    pstcDev->fCurrentSpeed = RTurn_RpmToAngularSpeed(fRpm, pstcDev->stcConfig.fReductionRatio);
    pstcDev->u8CurrentDir = RTurn_ConvertMotorDirToRTurnDir(u8MotorDir, pstcDev->stcConfig.u8ReverseOutput);
    
    // 获取期望方向（仲裁器输出），用于解锁判断
    uint8_t u8DesiredDir = MOTOR_DIRECTION_NONE;
    RTurn_GetDesiredDirection(pstcDev, &u8DesiredDir);
    uint8_t u8DesiredRTurnDir = RTurn_ConvertMotorDirToRTurnDir(u8DesiredDir, pstcDev->stcConfig.u8ReverseOutput);
    
    // ========== 缓存期望方向（方案4） ==========
    // 如果期望方向有效（非STOP），缓存下来
    // 用于主动检测过流时，如果期望方向已被电机仲裁器输出STOP，使用缓存方向
    if (u8DesiredRTurnDir != RTURN_DIR_STOP) {
        pstcDev->u8LastDesiredDir = u8DesiredRTurnDir;
    }
    
    // ========== 主动检测过流告警状态（不依赖校准状态） ==========
    // 方案4：放宽条件，使用缓存方向作为备选
    // 如果当前期望方向为STOP但缓存方向有效，使用缓存方向
    {
        uint8_t u8CheckDir = u8DesiredRTurnDir;
        if (u8CheckDir == RTURN_DIR_STOP && pstcDev->u8LastDesiredDir != RTURN_DIR_STOP) {
            u8CheckDir = pstcDev->u8LastDesiredDir;
        }
        
        if (!pstcDev->stcLockState.u8LockActive && u8CheckDir != RTURN_DIR_STOP && !pstcDev->u8LimitTriggered) {
            uint8_t u8Alarm = RTurn_ReadSensorAlarm(pstcDev);
            
            if (u8Alarm) {
                // 过流告警且方向有效，立即锁死
                uint8_t u8LimitDir = (u8CheckDir == RTURN_DIR_FORWARD) ? RTURN_LIMIT_FORWARD : RTURN_LIMIT_REVERSE;
                
                if (u8CheckDir == RTURN_DIR_REVERSE) {
                    /* 关闭时过流：重置角度为下限位角度，并标记为已校准 */
                    pstcDev->fCurrentAngle = pstcDev->stcConfig.fMinAngle;
                    if (!pstcDev->u8Calibrated) {
                        pstcDev->u8Calibrated = 1;
                        RTURN_OUT("Calibrated! Angle set to min angle: %f deg\r\n", pstcDev->fCurrentAngle);
                    }
                }
                /* 打开时过流：不重置角度，保持当前角度 */
                
                pstcDev->stcLockState.u8LockedDir = u8CheckDir;
                pstcDev->stcLockState.u8LockActive = 1;
                pstcDev->u8LimitTriggered = 1;
                
                int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 100);
                RTURN_OUT("LIMIT TRIGGERED (active check)! Dir=%s, Angle=%ld.%02ld deg, Calibrated=%d, UsedCachedDir=%d\r\n",
                          (u8LimitDir == RTURN_LIMIT_FORWARD) ? "FORWARD" : "REVERSE",
                          (long)(s32AngleInt / 100), (long)(s32AngleInt % 100),
                          pstcDev->u8Calibrated,
                          (u8DesiredRTurnDir == RTURN_DIR_STOP) ? 1 : 0);
                
                RTurn_LimitEvent_t stcEvent;
                stcEvent.u8Direction = u8LimitDir;
                stcEvent.fAngle = pstcDev->fCurrentAngle;
                stcEvent.u8IsActive = 1;
                EventBus_Publish(TOPIC_RTURN_LIMIT, &stcEvent);
            }
        }
    }
    
    // ========== 解除锁死逻辑（不依赖校准状态） ==========
    if (pstcDev->stcLockState.u8LockActive) {
        if ((pstcDev->stcLockState.u8LockedDir == RTURN_DIR_FORWARD && u8DesiredRTurnDir == RTURN_DIR_REVERSE) ||
            (pstcDev->stcLockState.u8LockedDir == RTURN_DIR_REVERSE && u8DesiredRTurnDir == RTURN_DIR_FORWARD)) {
            
            uint8_t u8ReleaseDir = (pstcDev->stcLockState.u8LockedDir == RTURN_DIR_FORWARD) ? RTURN_LIMIT_FORWARD : RTURN_LIMIT_REVERSE;
            
            pstcDev->stcLockState.u8LockActive = 0;
            pstcDev->stcLockState.u8LockedDir = 0;
            pstcDev->u8LimitTriggered = 0;
            
            RTURN_OUT("LOCK RELEASED! Desired direction changed to opposite, release dir=%d\r\n", u8ReleaseDir);
            
            RTurn_LimitEvent_t stcEvent;
            stcEvent.u8Direction = u8ReleaseDir;
            stcEvent.fAngle = pstcDev->fCurrentAngle;
            stcEvent.u8IsActive = 0;
            EventBus_Publish(TOPIC_RTURN_LIMIT, &stcEvent);
            
            // 解锁后立即检测告警状态（不依赖校准状态）
            if (u8DesiredRTurnDir != RTURN_DIR_STOP) {
                uint8_t u8Alarm = RTurn_ReadSensorAlarm(pstcDev);
                
                if (u8Alarm) {
                    // 仍然过流，锁死新方向
                    uint8_t u8NewLimitDir = (u8DesiredRTurnDir == RTURN_DIR_FORWARD) ? RTURN_LIMIT_FORWARD : RTURN_LIMIT_REVERSE;
                    
                    if (u8DesiredRTurnDir == RTURN_DIR_REVERSE) {
                        /* 关闭时过流：重置角度为下限位角度（仅在校准状态下才重置） */
                        if (pstcDev->u8Calibrated) {
                            pstcDev->fCurrentAngle = pstcDev->stcConfig.fMinAngle;
                        }
                    }
                    /* 打开时过流：不重置角度，保持当前角度 */
                    
                    pstcDev->stcLockState.u8LockedDir = u8DesiredRTurnDir;
                    pstcDev->stcLockState.u8LockActive = 1;
                    pstcDev->u8LimitTriggered = 1;
                    
                    int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 100);
                    RTURN_OUT("LOCK RE-LOCK! Still overcurrent, new Dir=%s, Angle=%ld.%02ld deg, Calibrated=%d\r\n",
                              (u8NewLimitDir == RTURN_LIMIT_FORWARD) ? "FORWARD" : "REVERSE",
                              (long)(s32AngleInt / 100), (long)(s32AngleInt % 100),
                              pstcDev->u8Calibrated);
                    
                    RTurn_LimitEvent_t stcNewEvent;
                    stcNewEvent.u8Direction = u8NewLimitDir;
                    stcNewEvent.fAngle = pstcDev->fCurrentAngle;
                    stcNewEvent.u8IsActive = 1;
                    EventBus_Publish(TOPIC_RTURN_LIMIT, &stcNewEvent);
                }
            }
        }
    }
    
    // ========== 校准后的逻辑（角度积分、限位检测等） ==========
    if (pstcDev->u8Calibrated) {
        // 检查锁死方向：如果当前运动方向被锁死
        // - 关闭（反转）锁死：跳过角度积分，角度冻结在 fMinAngle
        // - 打开（正转）锁死：允许继续积分（霍尔计数继续累加），直到电机停止
        if (pstcDev->stcLockState.u8LockActive && 
            pstcDev->stcLockState.u8LockedDir == pstcDev->u8CurrentDir) {
            
            if (pstcDev->stcLockState.u8LockedDir == RTURN_DIR_REVERSE) {
                /* 关闭锁死：跳过角度积分，角度冻结 */
                if (u32Now - s_u32LastLockDebugPrintTime >= LOCK_DEBUG_PRINT_INTERVAL_MS) {
                    s_u32LastLockDebugPrintTime = u32Now;
                    RTURN_DEBUG("Direction %d is locked (REVERSE), ignore angle change\r\n", pstcDev->u8CurrentDir);
                }
                pstcDev->u32LastAngleTime = u32Now;
                return;
            }
            /* 打开锁死：允许继续积分，直到电机停止 */
            if (u32Now - s_u32LastLockDebugPrintTime >= LOCK_DEBUG_PRINT_INTERVAL_MS) {
                s_u32LastLockDebugPrintTime = u32Now;
                RTURN_DEBUG("Direction %d is locked (FORWARD), allow angle integration\r\n", pstcDev->u8CurrentDir);
            }
        }
        
        float fDeltaAngle = pstcDev->fCurrentSpeed * ((float)u32DeltaMs / 1000.0f);
        
        if (pstcDev->u8CurrentDir == RTURN_DIR_FORWARD) {
            pstcDev->fCurrentAngle += fDeltaAngle;
        } else if (pstcDev->u8CurrentDir == RTURN_DIR_REVERSE) {
            pstcDev->fCurrentAngle -= fDeltaAngle;
        }
        
        // 正转角度上限检测（角度到位锁死）
        if (pstcDev->fCurrentAngle >= pstcDev->stcConfig.fMaxAngle) {
            pstcDev->fCurrentAngle = pstcDev->stcConfig.fMaxAngle;
            
            // 只有当限位未被触发时才发布事件
            if (!pstcDev->u8LimitTriggered) {
                pstcDev->u8LimitTriggered = 1;
                
                // 添加锁死状态设置
                pstcDev->stcLockState.u8LockedDir = RTURN_DIR_FORWARD;
                pstcDev->stcLockState.u8LockActive = 1;
                
                RTurn_LimitEvent_t stcEvent;
                stcEvent.u8Direction = RTURN_LIMIT_FORWARD;  // 正转限位
                stcEvent.fAngle = pstcDev->fCurrentAngle;
                stcEvent.u8IsActive = 1;
                EventBus_Publish(TOPIC_RTURN_LIMIT, &stcEvent);
                
                RTURN_OUT("Position limit reached! Forward blocked\r\n");
            }
        }
        
        // 反转角度下限检测：只调整角度，不触发限位锁死
        // 反转到位靠过流检测来锁死，不由角度触发
        if (pstcDev->fCurrentAngle <= pstcDev->stcConfig.fMinAngle) {
            pstcDev->fCurrentAngle = pstcDev->stcConfig.fMinAngle;
            // 注意：这里不发布限位事件，不设置锁死状态
        }
    }
    
    pstcDev->u32LastAngleTime = u32Now;
}

// ========== 过流事件处理 ==========
// 由 EventBus 回调 RTurn_OnCurrentAlarm 调用
// 收到过流事件后，直接锁死当前期望方向，不再判断阈值（阈值判断在 dev_sensor 中完成）
static void RTurn_HandleOvercurrent(RTurn_Device_t* pstcDev) {
    if (!pstcDev) return;
    
    // 如果已经锁死或限位已触发，不再重复处理
    if (pstcDev->stcLockState.u8LockActive || pstcDev->u8LimitTriggered) {
        return;
    }
    
    // 只使用期望方向（仲裁器输出的方向）来判断限位方向
    uint8_t u8DesiredDir = MOTOR_DIRECTION_NONE;
    RTurn_GetDesiredDirection(pstcDev, &u8DesiredDir);
    uint8_t u8CurrentDir = RTurn_ConvertMotorDirToRTurnDir(u8DesiredDir, pstcDev->stcConfig.u8ReverseOutput);
    
    // 期望方向有效时才处理
    if (u8CurrentDir != RTURN_DIR_STOP) {
        
        uint8_t u8LimitDir = (u8CurrentDir == RTURN_DIR_FORWARD) ? RTURN_LIMIT_FORWARD : RTURN_LIMIT_REVERSE;
        
        if (!pstcDev->u8Calibrated) {
            /* ========== 未校准状态 ========== */
            if (u8CurrentDir == RTURN_DIR_FORWARD) {
                /* 打开过流：不校准、不重置角度，但锁死正转方向（影响仲裁器） */
                RTURN_OUT("LIMIT TRIGGERED (uncalibrated, FORWARD)! Lock direction, no calibration\r\n");
            } else {
                /* 关闭过流：校准成功，重置角度为下限位角度 */
                pstcDev->fCurrentAngle = pstcDev->stcConfig.fMinAngle;
                pstcDev->u8Calibrated = 1;
                RTURN_OUT("LIMIT TRIGGERED (uncalibrated, REVERSE)! Calibrated=1, Angle=%ld.%02ld deg\r\n",
                          (long)((int32_t)(pstcDev->fCurrentAngle * 100) / 100),
                          (long)((int32_t)(pstcDev->fCurrentAngle * 100) % 100));
            }
            
            /* 未校准状态下，打开和关闭过流都锁死方向（影响仲裁器） */
            pstcDev->stcLockState.u8LockedDir = u8CurrentDir;
            pstcDev->stcLockState.u8LockActive = 1;
            pstcDev->u8LimitTriggered = 1;
            
            RTurn_LimitEvent_t stcEvent;
            stcEvent.u8Direction = u8LimitDir;
            stcEvent.fAngle = pstcDev->fCurrentAngle;
            stcEvent.u8IsActive = 1;
            EventBus_Publish(TOPIC_RTURN_LIMIT, &stcEvent);
        } else {
            /* ========== 已校准状态 ========== */
            if (u8CurrentDir == RTURN_DIR_REVERSE) {
                /* 关闭时过流：重置角度为下限位角度 */
                pstcDev->fCurrentAngle = pstcDev->stcConfig.fMinAngle;
            }
            /* 打开时过流：不重置角度，保持当前角度 */
            
            pstcDev->stcLockState.u8LockedDir = u8CurrentDir;
            pstcDev->stcLockState.u8LockActive = 1;
            pstcDev->u8LimitTriggered = 1;
            
            int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 100);
            RTURN_OUT("LIMIT TRIGGERED (calibrated)! Dir=%s, Angle=%ld.%02ld deg, LockDir=%d\r\n",
                      (u8LimitDir == RTURN_LIMIT_FORWARD) ? "FORWARD" : "REVERSE",
                      (long)(s32AngleInt / 100), (long)(s32AngleInt % 100),
                      pstcDev->stcLockState.u8LockedDir);
            
            RTurn_LimitEvent_t stcEvent;
            stcEvent.u8Direction = u8LimitDir;
            stcEvent.fAngle = pstcDev->fCurrentAngle;
            stcEvent.u8IsActive = 1;
            EventBus_Publish(TOPIC_RTURN_LIMIT, &stcEvent);
        }
    }
}

static void RTurn_ClearLock(RTurn_Device_t* pstcDev) {
    if (!pstcDev) return;
    
    pstcDev->stcLockState.u8LockedDir = 0;
    pstcDev->stcLockState.u8LockActive = 0;
    pstcDev->u8LimitTriggered = 0;
    
    RTURN_OUT("Lock cleared\r\n");
}

// ========== EventBus回调函数 ==========

void RTurn_OnCurrentAlarm(void* payload) {
    Current_AlarmEvent_t* pstcEvent = (Current_AlarmEvent_t*)payload;
    
    if (!pstcEvent->u8IsActive) return;
    
    // 遍历所有设备，通过 ops.init 函数指针找到 RTurn 设备
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        DeviceNode_t* pstcNode = DeviceManager_Get(i);
        if (pstcNode && pstcNode->used && pstcNode->ops.init == RTurn_Device_Init) {
            RTurn_Device_t* pstcDev = (RTurn_Device_t*)pstcNode->private_data;
            RTurn_HandleOvercurrent(pstcDev);
            break;
        }
    }
}

// ========== 标准设备操作实现 ==========

DeviceResult_t RTurn_Device_Init(void* handle) {
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)handle;
    if (!pstcDev) return RESULT_PARAM_ERR;
    
    RTURN_DEBUG("Init: MotorHall ID=%d, Sensor ID=%d, ReverseOutput=%d\r\n",
                pstcDev->stcConfig.u8MotorHallDevId,
                pstcDev->stcConfig.u8SensorDevId,
                pstcDev->stcConfig.u8ReverseOutput);
    
    int32_t s32RatioInt = (int32_t)(pstcDev->stcConfig.fReductionRatio * 100);
    int32_t s32MaxAngleInt = (int32_t)(pstcDev->stcConfig.fMaxAngle * 100);
    int32_t s32MinAngleInt = (int32_t)(pstcDev->stcConfig.fMinAngle * 100);
    RTURN_DEBUG("Mechanical: Ratio=%ld.%02ld, MaxAngle=%ld.%02ld deg, MinAngle=%ld.%02ld deg\r\n",
                (long)(s32RatioInt / 100), (long)(s32RatioInt % 100),
                (long)(s32MaxAngleInt / 100), (long)(s32MaxAngleInt % 100),
                (long)(s32MinAngleInt / 100), (long)(s32MinAngleInt % 100));
    
    pstcDev->fCurrentAngle = 0;
    pstcDev->fCurrentSpeed = 0;
    pstcDev->u8CurrentDir = RTURN_DIR_STOP;
    pstcDev->u8Calibrated = 0;
    pstcDev->u8LimitTriggered = 0;
    pstcDev->stcLockState.u8LockedDir = 0;
    pstcDev->stcLockState.u8LockActive = 0;
    pstcDev->u32LastAngleTime = tickTimer_GetCount();
    
    DeviceNode_t* pstcMotorNode = DeviceManager_Get(pstcDev->stcConfig.u8MotorHallDevId);
    if (!pstcMotorNode || !pstcMotorNode->private_data) {
        RTURN_DEBUG("Warning: MotorHall device ID=%d not found!\r\n", 
                    pstcDev->stcConfig.u8MotorHallDevId);
    }
    
    DeviceNode_t* pstcSensorNode = DeviceManager_Get(pstcDev->stcConfig.u8SensorDevId);
    if (!pstcSensorNode || !pstcSensorNode->private_data) {
        RTURN_DEBUG("Warning: Sensor device ID=%d not found!\r\n", 
                    pstcDev->stcConfig.u8SensorDevId);
    }
    
    pstcDev->u8Initialized = 1;
    pstcDev->u32LastUpdateTime = tickTimer_GetCount();
    
    RTURN_DEBUG("Init success\r\n");
    
    return RESULT_OK;
}

DeviceResult_t RTurn_Device_Deinit(void* handle) {
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)handle;
    if (!pstcDev) return RESULT_PARAM_ERR;
    
    RTURN_DEBUG("Deinit\r\n");
    pstcDev->u8Initialized = 0;
    return RESULT_OK;
}

DeviceResult_t RTurn_Device_Read(void* handle, void* data, uint32_t size) {
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)handle;
    if (!pstcDev || !data) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;
    
    if (size == sizeof(RTurn_ReadResponse_t)) {
        RTurn_ReadResponse_t* pstcResp = (RTurn_ReadResponse_t*)data;
        pstcResp->fAngle = pstcDev->fCurrentAngle;
        pstcResp->fSpeed = pstcDev->fCurrentSpeed;
        pstcResp->u8Direction = pstcDev->u8CurrentDir;
        pstcResp->u8LockedDir = pstcDev->stcLockState.u8LockedDir;
        pstcResp->u8Calibrated = pstcDev->u8Calibrated;
        return RESULT_OK;
    }
    
    return RESULT_PARAM_ERR;
}

DeviceResult_t RTurn_Device_Write(void* handle, const void* data, uint32_t size) {
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t RTurn_Device_Control(void* handle, DeviceCommandData_t* pstcCmd) {
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)handle;
    if (!pstcDev || !pstcCmd) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;
    
    switch (pstcCmd->cmd) {
        case CMD_RTURN_GET_ANGLE:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(int32_t)) {
                *(int32_t*)pstcCmd->response = (int32_t)(pstcDev->fCurrentAngle * 100);
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_RTURN_GET_ANGLE_DEG:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(float)) {
                *(float*)pstcCmd->response = pstcDev->fCurrentAngle;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_RTURN_GET_SPEED:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(float)) {
                *(float*)pstcCmd->response = pstcDev->fCurrentSpeed;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_RTURN_GET_DIRECTION:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)pstcCmd->response = pstcDev->u8CurrentDir;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_RTURN_GET_LOCKED_DIR:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)pstcCmd->response = pstcDev->stcLockState.u8LockedDir;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_RTURN_RESET_POSITION:
            pstcDev->fCurrentAngle = 0;
            pstcDev->u8Calibrated = 1;
            RTURN_OUT("Position reset to 0, calibrated=1\r\n");
            return RESULT_OK;
            
        case CMD_RTURN_CLEAR_LOCK:
            RTurn_ClearLock(pstcDev);
            return RESULT_OK;
            
        case CMD_RTURN_GET_CALIBRATED:
            if (pstcCmd->response && pstcCmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)pstcCmd->response = pstcDev->u8Calibrated;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        default:
            RTURN_DEBUG("Unknown cmd=0x%08X\r\n", pstcCmd->cmd);
            return RESULT_ERROR;
    }
}

DeviceResult_t RTurn_Device_Update(void* handle) {
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)handle;
    if (!pstcDev || !pstcDev->u8Initialized) return RESULT_ERROR;
    
    uint32_t u32Now = tickTimer_GetCount();
    
    if (u32Now - pstcDev->u32LastUpdateTime < pstcDev->stcConfig.u16UpdateIntervalMs) {
        return RESULT_OK;
    }
    pstcDev->u32LastUpdateTime = u32Now;
    
    RTurn_UpdateAngle(pstcDev);
    
    // ========== 刷新Keil Watch调试全局变量 ==========
    g_fDbgRTurnAngle       = pstcDev->fCurrentAngle;
    g_fDbgRTurnSpeed       = pstcDev->fCurrentSpeed;
    g_u8DbgRTurnDir        = pstcDev->u8CurrentDir;
    g_u8DbgRTurnLockedDir  = pstcDev->stcLockState.u8LockedDir;
    g_u8DbgRTurnLockActive = pstcDev->stcLockState.u8LockActive;
    g_u8DbgRTurnCalibrated = pstcDev->u8Calibrated;
    g_u8DbgRTurnLimitTrig  = pstcDev->u8LimitTriggered;
    
    // 获取期望方向（仲裁器输出）并刷新到全局变量
    {
        uint8_t u8DesiredDir = MOTOR_DIRECTION_NONE;
        RTurn_GetDesiredDirection(pstcDev, &u8DesiredDir);
        g_u8DbgRTurnDesiredDir = RTurn_ConvertMotorDirToRTurnDir(u8DesiredDir, pstcDev->stcConfig.u8ReverseOutput);
    }
    
    // ========== 每2000ms打印一次锁死方向状态 ==========
    if (u32Now - s_u32LastLockPrintTime >= LOCK_PRINT_INTERVAL_MS) {
        s_u32LastLockPrintTime = u32Now;
        
        if (pstcDev->stcLockState.u8LockActive) {
            const char* pcLockDir = (pstcDev->stcLockState.u8LockedDir == RTURN_DIR_FORWARD) ? "FORWARD" :
                                    (pstcDev->stcLockState.u8LockedDir == RTURN_DIR_REVERSE) ? "REVERSE" : "NONE";
            int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 10);
            RTURN_OUT("RTurn: LOCK ACTIVE - Dir=%s, Angle=%ld.%ld deg\r\n", 
                      pcLockDir, (long)(s32AngleInt / 10), (long)(s32AngleInt % 10));
        } else {
            if (pstcDev->u8Calibrated) {
                int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 10);
                int32_t s32SpeedInt = (int32_t)(pstcDev->fCurrentSpeed * 10);
                RTURN_OUT("RTurn: No lock, Angle=%ld.%ld deg, Speed=%ld.%ld deg/s\r\n",
                          (long)(s32AngleInt / 10), (long)(s32AngleInt % 10),
                          (long)(s32SpeedInt / 10), (long)(s32SpeedInt % 10));
            } else {
                RTURN_OUT("RTurn: No lock, Not calibrated (waiting for limit trigger)\r\n");
            }
        }
    }
    
    // ========== 每4000ms打印一次当前角度 ==========
    if (pstcDev->u8Calibrated && (u32Now - s_u32LastLimitPrintTime >= LIMIT_PRINT_INTERVAL_MS)) {
        s_u32LastLimitPrintTime = u32Now;
        int32_t s32AngleInt = (int32_t)(pstcDev->fCurrentAngle * 100);
        RTURN_OUT("RTurn: Current Angle=%ld.%02ld deg, Dir=%d, Lock=%d\r\n",
                  (long)(s32AngleInt / 100), (long)(s32AngleInt % 100),
                  pstcDev->u8CurrentDir,
                  pstcDev->stcLockState.u8LockActive);
    }
    
    return RESULT_OK;
}

// ========== 圆弧转动机构特定接口 ==========

float RTurn_Device_GetAngle(RTurn_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->fCurrentAngle;
}

float RTurn_Device_GetAngleDeg(RTurn_Device_t* pstcDev) {
    return RTurn_Device_GetAngle(pstcDev);
}

float RTurn_Device_GetSpeed(RTurn_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->fCurrentSpeed;
}

uint8_t RTurn_Device_GetDirection(RTurn_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->u8CurrentDir;
}

uint8_t RTurn_Device_GetLockedDir(RTurn_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->stcLockState.u8LockedDir;
}

void RTurn_Device_ResetPosition(RTurn_Device_t* pstcDev) {
    if (!pstcDev) return;
    pstcDev->fCurrentAngle = 0;
    pstcDev->u8Calibrated = 1;
    RTURN_OUT("Position reset\r\n");
}

void RTurn_Device_ClearLock(RTurn_Device_t* pstcDev) {
    RTurn_ClearLock(pstcDev);
}

uint8_t RTurn_Device_IsCalibrated(RTurn_Device_t* pstcDev) {
    if (!pstcDev || !pstcDev->u8Initialized) return 0;
    return pstcDev->u8Calibrated;
}

RTurn_Device_t* RTurn_Device_Create(const RTurn_Config_t* pstcConfig) {
    if (!pstcConfig) return NULL;
    
    RTurn_Device_t* pstcDev = (RTurn_Device_t*)malloc(sizeof(RTurn_Device_t));
    if (!pstcDev) return NULL;
    
    memset(pstcDev, 0, sizeof(RTurn_Device_t));
    memcpy(&pstcDev->stcConfig, pstcConfig, sizeof(RTurn_Config_t));
    
    if (pstcDev->stcConfig.u16UpdateIntervalMs == 0) {
        pstcDev->stcConfig.u16UpdateIntervalMs = 50;
    }
    
    int32_t s32RatioInt = (int32_t)(pstcConfig->fReductionRatio * 100);
    RTURN_DEBUG("Create: MotorHall ID=%d, Sensor ID=%d, Ratio=%ld.%02ld, Reverse=%d\r\n",
                pstcConfig->u8MotorHallDevId,
                pstcConfig->u8SensorDevId,
                (long)(s32RatioInt / 100), (long)(s32RatioInt % 100),
                pstcConfig->u8ReverseOutput);
    
    pstcDev->u8Initialized = 0;
    return pstcDev;
}

// ========== 全局操作函数表 ==========
const DeviceOps_t g_rturn_ops = {
    .init = RTurn_Device_Init,
    .deinit = RTurn_Device_Deinit,
    .read = RTurn_Device_Read,
    .write = RTurn_Device_Write,
    .control = RTurn_Device_Control,
    .update = RTurn_Device_Update
};
