#include "dev_motor.h"
#include "dev_pwm.h"          // PWM设备层
#include "dev_voltage.h"      // 电压告警事件类型
#include "dev_sensor.h"       // 电流告警事件类型
#include "device_manager.h"
#include "TickTimer.h"
#include <string.h>
#include "rtt_log.h"
#include "dev_rturn.h"          // 添加这行，用于 RTurn_LimitEvent_t
#include "Template_Pwm.h"       // PWM配置（TMRA_4, PB6/PB7）
#include "hc32_ll_tmra.h"      // TMRA寄存器操作
#include "Pwm.h"                // PWM驱动

// 引用 main.c 中定义的PWM实例
extern pwm_t g_motor_pwm_ch1;
extern pwm_t g_motor_pwm_ch2;
extern pwm_t g_motor_pwm_ch3;
extern pwm_t g_motor_pwm_ch4;
// 在文件开头，其他静态变量定义附近添加

static MotorDir_t s_eLastArbitrationDir = DIR_NONE;  // 上次仲裁的方向
static uint16_t s_u16LastTargetDuty = 0;              // 上次的目标占空比
static bool s_bStopPolarityPending = false;           // 是否需要切换停止极性

// 在文件开头，其他 include 之后添加
// 打印间隔控制
volatile int motor_state = 0;
static uint32_t s_u32LastMotorPrintTime = 0;
#define MOTOR_PRINT_INTERVAL_MS   500

// 缓启动时间配置（毫秒）
#define MOTOR_RAMP_TIME_MS      800

// 占空比限制（2%-98%）
#define MOTOR_DUTY_MIN          2
#define MOTOR_DUTY_MAX          98

// ========== Keil Watch 调试全局变量 ==========
// 电机仲裁层调试变量 - 在 Watch 窗口添加这些变量即可实时查看电机仲裁状态
MotorDevice_t* volatile g_pMotor_Dev_Watch = NULL;      // 电机设备指针（可展开查看所有成员）
MotorDebugInfo_t volatile g_stcMotorDebug = {0};     // 调试信息副本（结构体形式）
// 为了方便查看，再导出一些单独的变量
volatile uint8_t  g_u8MotorDebug_BlockFwdCount = 0;   // 正转阻塞队列数量
volatile uint8_t  g_u8MotorDebug_BlockRevCount = 0;   // 反转阻塞队列数量
volatile uint8_t  g_u8MotorDebug_AllowFwdCount = 0;   // 正转允许队列数量
volatile uint8_t  g_u8MotorDebug_AllowRevCount = 0;   // 反转允许队列数量
volatile uint8_t  g_u8MotorDebug_ActiveDeviceId = 0;  // 当前活动设备ID
volatile uint8_t  g_u8MotorDebug_ActiveDir = 0;       // 当前活动方向(0=停止,1=正转,2=反转)
volatile uint8_t  g_u8MotorDebug_State = 0;           // 电机状态(0=IDLE,1=RAMPING,2=RUNNING)
volatile float    g_fMotorDebug_CurrentDuty = 0.0f;   // 当前占空比
volatile uint8_t  g_u8MotorDebug_ConflictFault = 0;   // 冲突故障标志

// ========== 设备ID定义 ==========
#define DEV_PWM_MOTOR_ID    1   // 电机PWM设备ID
#define DEV_MOTOR_ID        0   // 电机设备ID（与 App_Motor_Project.h 中的 ID_MOTOR 保持一致）

// ========== 设备基因定义 ==========
typedef struct {
    MotorDeviceId_t id;
    MotorPriority_t priority;
    uint32_t capability;
} MotorDeviceGene_t;

// 根据优先级模式动态定义优先级值
#if MOTOR_PRIORITY_IO_HIGH
    #define MANUAL_PRIORITY  3
    #define CAN_PRIORITY     4
#else
    #define MANUAL_PRIORITY  4
    #define CAN_PRIORITY     3
#endif

// 设备基因表
static const MotorDeviceGene_t c_device_genes[] = {
#if MOTOR_MODE_BIPOLAR
    { DEV_ID_POWER_POS,  PRIO_POWER,     CAP_ALLOW },
    { DEV_ID_POWER_NEG,  PRIO_POWER,     CAP_ALLOW },
#endif
    // { DEV_ID_LIMIT_FWD,  PRIO_LIMIT,     CAP_BLOCK },
    // { DEV_ID_LIMIT_REV,  PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_RTURN_FWD,         PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_RTURN_REV,         PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_OVERVOLTAGE_FWD,   PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_OVERVOLTAGE_REV,   PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_UNDERVOLTAGE_FWD,  PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_UNDERVOLTAGE_REV,  PRIO_LIMIT,     CAP_BLOCK },
	{ DEV_ID_OVERCUR_FWD,       PRIO_LIMIT,     CAP_BLOCK },
    // { DEV_ID_CAN,        (MotorPriority_t)CAN_PRIORITY,   MOTOR_CAN_CAPABILITY },
    { DEV_ID_IO_FWD,     (MotorPriority_t)MANUAL_PRIORITY, MOTOR_MANUAL_CAPABILITY },
    { DEV_ID_IO_REV,     (MotorPriority_t)MANUAL_PRIORITY, MOTOR_MANUAL_CAPABILITY },
    // { DEV_ID_EMERGENCY,  PRIO_EMERGENCY, CAP_BLOCK }
};

// ========== 辅助函数：限制占空比在2%-98% ==========
static uint16_t Motor_LimitDuty(uint16_t duty) {
    if (duty < MOTOR_DUTY_MIN) return MOTOR_DUTY_MIN;
    if (duty > MOTOR_DUTY_MAX) return MOTOR_DUTY_MAX;
    return duty;
}

// ========== 新增：使用PWM驱动设置四通道占空比（停止时特殊极性） ==========
static void Motor_SetStopDuty(void) {
    // 停止：PB6=50%(高有效), PB7=50%(低有效), PB8=50%(高有效), PB9=50%(低有效)
    // 由于PWM实例初始化时是低有效，停止时需要改变极性
    // 直接操作寄存器实现停止时的特殊极性
    
    stc_tmra_pwm_init_t stcPwmInit;
    (void)TMRA_PWM_StructInit(&stcPwmInit);
    uint32_t u32Period = TMRA_GetPeriodValue(CM_TMRA_4);
    uint32_t u32Compare = (u32Period * 50U) / 100U;

    /* CH1(PB6) - 高有效，50% */
    stcPwmInit.u32CompareValue        = u32Compare;
    stcPwmInit.u16StartPolarity       = TMRA_PWM_LOW;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_LOW;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_LOW;
    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH1, &stcPwmInit);
    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH1, ENABLE);

    /* CH2(PB7) - 低有效，50% */
    stcPwmInit.u32CompareValue        = u32Compare;
    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH2, &stcPwmInit);
    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH2, ENABLE);

//    /* CH3(PB8) - 高有效，50% */
//    stcPwmInit.u32CompareValue        = u32Compare;
//    stcPwmInit.u16StartPolarity       = TMRA_PWM_LOW;
//    stcPwmInit.u16StopPolarity        = TMRA_PWM_LOW;
//    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
//    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_LOW;
//    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH3, &stcPwmInit);
//    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH3, ENABLE);
//
//    /* CH4(PB9) - 低有效，50% */
//    stcPwmInit.u32CompareValue        = u32Compare;
//    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
//    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
//    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
//    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
//    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH4, &stcPwmInit);
//    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH4, ENABLE);
}

// ========== 新增：设置运行时的低有效极性（全部低有效） ==========
static void Motor_SetRunPolarity(void) {
    stc_tmra_pwm_init_t stcPwmInit;
    (void)TMRA_PWM_StructInit(&stcPwmInit);

    /* CH1(PB6) - 低有效 */
    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH1, &stcPwmInit);
    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH1, ENABLE);

    /* CH2(PB7) - 低有效 */
    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH2, &stcPwmInit);
    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH2, ENABLE);

//    /* CH3(PB8) - 低有效 */
//    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH3, &stcPwmInit);
//    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH3, ENABLE);
//
//    /* CH4(PB9) - 低有效 */
//    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH4, &stcPwmInit);
//    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH4, ENABLE);
}

// ========== 新增：使用PWM驱动设置运行时的占空比（全部低有效） ==========
static void Motor_SetRunDutyDirect(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4) {
    // 直接操作寄存器设置占空比（绕过PWM驱动的缓启动）
    uint32_t u32Period = TMRA_GetPeriodValue(CM_TMRA_4);
    TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH1, (u32Period * ch1) / 100U);
    TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH2, (u32Period * ch2) / 100U);
//    TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH3, (u32Period * ch3) / 100U);
//    TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH4, (u32Period * ch4) / 100U);
}

// ========== 修改：切换极性前先将占空比归零 ==========
static void Motor_RampForward(uint16_t target_duty) {
    uint16_t limited_duty = Motor_LimitDuty(target_duty);
    uint16_t duty_low = limited_duty;
    uint16_t duty_high = 100 - limited_duty;
    
    // 获取当前占空比
    uint16_t current_duty_ch1 = PWM_GetDuty(&g_motor_pwm_ch1);
    
    MAIN_D("[MOTOR] Ramp FWD: current=%d%%, target CH1/2=%d%%, CH3/4=%d%%, time=%dms\r\n",
           current_duty_ch1, duty_low, duty_high, MOTOR_RAMP_TIME_MS);
    
    // 如果不是从停止状态启动，或者当前占空比不是0，先快速归零
    if (current_duty_ch1 != 0) {
        // 快速将占空比降到 0（50ms 快速归零）
        PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch1, current_duty_ch1, 0, 50);
        PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch2, current_duty_ch1, 0, 50);
//        PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch3, 100 - current_duty_ch1, 0, 50);
//        PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch4, 100 - current_duty_ch1, 0, 50);

        // 等待归零完成
        tickTimer_DelayMs(50);
        current_duty_ch1 = 0;
        MAIN_D("[MOTOR] Ramp FWD: zeroed duty before polarity switch\r\n");
    }

    // 切换极性（此时占空比为0，不会冲击）
    Motor_SetRunPolarity();

    // 从 0% 缓启动到目标值
    PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch1, 0, duty_low, MOTOR_RAMP_TIME_MS);
    PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch2, 0, duty_low, MOTOR_RAMP_TIME_MS);
//    PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch3, 0, duty_high, MOTOR_RAMP_TIME_MS);
//    PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch4, 0, duty_high, MOTOR_RAMP_TIME_MS);
}

static void Motor_RampReverse(uint16_t target_duty) {
    uint16_t limited_duty = Motor_LimitDuty(target_duty);
    uint16_t duty_low = 100 - limited_duty;
    uint16_t duty_high = limited_duty;
    
    // 获取当前占空比
    uint16_t current_duty_ch1 = PWM_GetDuty(&g_motor_pwm_ch1);
    
    MAIN_D("[MOTOR] Ramp REV: current=%d%%, target CH1/2=%d%%, time=%dms\r\n",
           current_duty_ch1, duty_low, MOTOR_RAMP_TIME_MS);
    
    // 如果不是从停止状态启动，或者当前占空比不是0，先快速归零
    if (current_duty_ch1 != 0) {
        // 快速将占空比降到 0（50ms 快速归零）
        PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch1, current_duty_ch1, 0, 50);
        PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch2, current_duty_ch1, 0, 50);
//        PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch3, 100 - current_duty_ch1, 0, 50);
//        PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch4, 100 - current_duty_ch1, 0, 50);

        // 等待归零完成（简单延时，或轮询状态）
        tickTimer_DelayMs(50);
        current_duty_ch1 = 0;
    }

    // 切换极性（此时占空比为0，不会冲击）
    Motor_SetRunPolarity();

    // 从 0% 缓启动到目标值
    PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch1, 0, duty_low, MOTOR_RAMP_TIME_MS);
    PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch2, 0, duty_low, MOTOR_RAMP_TIME_MS);
//    PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch3, 0, duty_high, MOTOR_RAMP_TIME_MS);
//    PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch4, 0, duty_high, MOTOR_RAMP_TIME_MS);
}

// ========== 新增：立即设置正转（无缓启动） ==========
static void Motor_RunForwardImmediate(uint16_t duty) {
    uint16_t limited_duty = Motor_LimitDuty(duty);
    uint16_t duty_low = limited_duty;
    uint16_t duty_high = 100 - limited_duty;
    
    // 先设置运行极性
    Motor_SetRunPolarity();
    
    // 立即设置占空比
    Motor_SetRunDutyDirect(duty_low, duty_low, duty_high, duty_high);

    // 同时更新PWM驱动的当前占空比（保持同步）
    PWM_SetDuty(&g_motor_pwm_ch1, duty_low);
    PWM_SetDuty(&g_motor_pwm_ch2, duty_low);
//    PWM_SetDuty(&g_motor_pwm_ch3, duty_high);
//    PWM_SetDuty(&g_motor_pwm_ch4, duty_high);

    MAIN_D("[MOTOR] Immediate FWD: duty=%d%%, CH1/2=%d%%, CH3/4=%d%%\r\n",
           limited_duty, duty_low, duty_high);
}

// ========== 新增：立即设置反转（无缓启动） ==========
static void Motor_RunReverseImmediate(uint16_t duty) {
    uint16_t limited_duty = Motor_LimitDuty(duty);
    uint16_t duty_low = 100 - limited_duty;
    uint16_t duty_high = limited_duty;
    
    // 先设置运行极性
    Motor_SetRunPolarity();
    
    // 立即设置占空比
    Motor_SetRunDutyDirect(duty_low, duty_low, duty_high, duty_high);

    // 同时更新PWM驱动的当前占空比（保持同步）
    PWM_SetDuty(&g_motor_pwm_ch1, duty_low);
    PWM_SetDuty(&g_motor_pwm_ch2, duty_low);
//    PWM_SetDuty(&g_motor_pwm_ch3, duty_high);
//    PWM_SetDuty(&g_motor_pwm_ch4, duty_high);

    MAIN_D("[MOTOR] Immediate REV: duty=%d%%, CH1/2=%d%%, CH3/4=%d%%\r\n",
           limited_duty, duty_low, duty_high);
}

// ========== 电机仲裁结果回调函数（弱定义，用户可重写） ==========
//    GPIO_SET(PHU_PORT, PHU_PIN);
//    GPIO_SET(PHV_PORT, PHV_PIN);
__weak void Motor_OnArbitrationStop(MotorDevice_t* motor) {
    (void)motor;
    
    // 检查是否需要执行停止
    // 修改：s_eLastArbitrationDir != DIR_NONE 或者 这是第一次停止（需要初始化停止状态）
    static bool s_bFirstStop = true;
    
    if (s_eLastArbitrationDir != DIR_NONE || s_bFirstStop) {
        // 获取当前占空比
        uint16_t current_duty_ch1 = PWM_GetDuty(&g_motor_pwm_ch1);
        
        MAIN_D("[MOTOR_ARB] Stop: from dir=%d, current duty=%d%%, first_stop=%d\r\n", 
               s_eLastArbitrationDir, current_duty_ch1, s_bFirstStop);
        
        if (s_bFirstStop) {
            // 第一次停止：直接设置停止极性和50%占空比，不缓启动
            Motor_SetStopDuty();
            s_bFirstStop = false;
            MAIN_D("[MOTOR_ARB] First stop: set stop polarity and 50%% duty directly\r\n");
        } else {
            // 后续停止：使用缓启动
            PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch1, current_duty_ch1, 50, MOTOR_RAMP_TIME_MS);
            PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch2, current_duty_ch1, 50, MOTOR_RAMP_TIME_MS);
//            PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch3, 100 - current_duty_ch1, 50, MOTOR_RAMP_TIME_MS);
//            PWM_StartRamp_TargetFromStart(&g_motor_pwm_ch4, 100 - current_duty_ch1, 50, MOTOR_RAMP_TIME_MS);
            
            // 标记需要切换停止极性（缓停完成后切换）
            s_bStopPolarityPending = true;
        }
        
        // 更新方向记录
        s_eLastArbitrationDir = DIR_NONE;
        s_u16LastTargetDuty = 0;
    }
    
    motor_state = 0;
}


    // GPIO_SET(PHU_PORT, PHU_PIN);
    // GPIO_RESET(PHV_PORT, PHV_PIN);
__weak void Motor_OnArbitrationFwd(MotorDevice_t* motor, float duty) {
    (void)motor;
    uint16_t duty_percent = (uint16_t)(duty);
    
    // 检查是否需要执行缓启动
    if (s_eLastArbitrationDir != DIR_FWD || s_u16LastTargetDuty != duty_percent) {
        // 清除待处理的停止极性切换标志
        s_bStopPolarityPending = false;
        
        MAIN_D("[MOTOR_ARB] FWD: start ramp...\r\n");
        Motor_RampForward(duty_percent);
        
        s_eLastArbitrationDir = DIR_FWD;
        s_u16LastTargetDuty = duty_percent;
    }
    motor_state = 1;
}

//    GPIO_SET(PHV_PORT, PHV_PIN);
//    GPIO_RESET(PHU_PORT, PHU_PIN);
__weak void Motor_OnArbitrationRev(MotorDevice_t* motor, float duty) {
    (void)motor;
    uint16_t duty_percent = (uint16_t)(duty);
    
    // 检查是否需要执行缓启动
    if (s_eLastArbitrationDir != DIR_REV || s_u16LastTargetDuty != duty_percent) {
        // 清除待处理的停止极性切换标志
        s_bStopPolarityPending = false;
        
        MAIN_D("[MOTOR_ARB] REV: start ramp...\r\n");
        Motor_RampReverse(duty_percent);
        
        s_eLastArbitrationDir = DIR_REV;
        s_u16LastTargetDuty = duty_percent;
    }
    motor_state = 2;
}

// ========== 内部函数声明 ==========
static const MotorDeviceGene_t* Motor_GetGene(MotorDeviceId_t id);
static void Motor_CmdList_Remove(MotorCommandList_t* q, MotorDeviceId_t id);
static void Motor_CmdList_SetAllow(MotorCommandList_t* q, MotorControlCommand_t cmd);
static void Motor_CmdList_SetBlock(MotorCommandList_t* q, MotorControlCommand_t cmd);
static void Motor_ArbitrationDecision(MotorDevice_t* motor);
static void Motor_UpdateDebugInfo(MotorDevice_t* motor);

// ========== 内部函数实现 ==========

static const MotorDeviceGene_t* Motor_GetGene(MotorDeviceId_t id) {
    for(int i = 0; i < sizeof(c_device_genes)/sizeof(MotorDeviceGene_t); i++) {
        if(c_device_genes[i].id == id) return &c_device_genes[i];
    }
    return NULL;
}

static void Motor_CmdList_Remove(MotorCommandList_t* q, MotorDeviceId_t id) {
    if (id == DEV_ID_RTURN_FWD || id == DEV_ID_RTURN_REV) {
        MAIN_D("[REMOVE_DEBUG] Trying to remove id=%d from queue at 0x%p, current count=%d\r\n", 
               id, q, q->count);
        // 打印调用栈或当前位置
    }    
    for (int i = 0; i < q->count; i++) {
        if (q->commands[i].device_id == id) {
            MAIN_D("[REMOVE_DEBUG] FOUND and REMOVED id=%d from queue at 0x%p\r\n", id, q);            
            for (int j = i; j < q->count - 1; j++) {
                q->commands[j] = q->commands[j + 1];
            }
            q->count--;
            // 清空被移除的位置
            if (q->count < MAX_COMMANDS_PER_DIRECTION) {
                q->commands[q->count].device_id = DEV_ID_NONE;
                q->commands[q->count].priority = PRIO_NONE;
                q->commands[q->count].type = CMD_TYPE_NONE_USE;
                q->commands[q->count].param = 0.0f;
                q->commands[q->count].timestamp = 0;
            }
            // 实时更新 Keil Watch 独立变量
            if (q == &g_pMotor_Dev_Watch->block_fwd) {
                g_u8MotorDebug_BlockFwdCount = q->count;
            } else if (q == &g_pMotor_Dev_Watch->block_rev) {
                g_u8MotorDebug_BlockRevCount = q->count;
            } else if (q == &g_pMotor_Dev_Watch->allow_fwd) {
                g_u8MotorDebug_AllowFwdCount = q->count;
            } else if (q == &g_pMotor_Dev_Watch->allow_rev) {
                g_u8MotorDebug_AllowRevCount = q->count;
            }
            return;
        }
    }
}

static void Motor_CmdList_SetAllow(MotorCommandList_t* q, MotorControlCommand_t cmd) {
    Motor_CmdList_Remove(q, cmd.device_id);
    if (q->count >= MAX_COMMANDS_PER_DIRECTION) return;

    int idx = 0;
    for (; idx < q->count; idx++) {
        if (cmd.priority < q->commands[idx].priority) break;
    }

    for (int i = q->count; i > idx; i--) {
        q->commands[i] = q->commands[i - 1];
    }

    q->commands[idx] = cmd;
    q->count++;
    
    // 实时更新 Keil Watch 独立变量
    if (g_pMotor_Dev_Watch != NULL) {
        if (q == &g_pMotor_Dev_Watch->allow_fwd) {
            g_u8MotorDebug_AllowFwdCount = q->count;
        } else if (q == &g_pMotor_Dev_Watch->allow_rev) {
            g_u8MotorDebug_AllowRevCount = q->count;
        }
    }
}

static void Motor_CmdList_SetBlock(MotorCommandList_t* q, MotorControlCommand_t cmd) {
    for (int i = 0; i < q->count; i++) {
        if (q->commands[i].device_id == cmd.device_id) return;
    }
    if (q->count >= MAX_COMMANDS_PER_DIRECTION) return;
    q->commands[q->count++] = cmd;
    
    // 实时更新 Keil Watch 独立变量
    if (g_pMotor_Dev_Watch != NULL) {
        if (q == &g_pMotor_Dev_Watch->block_fwd) {
            g_u8MotorDebug_BlockFwdCount = q->count;
        } else if (q == &g_pMotor_Dev_Watch->block_rev) {
            g_u8MotorDebug_BlockRevCount = q->count;
        }
    }
}

static void Motor_UpdateDebugInfo(MotorDevice_t* motor) {
    MotorDebugInfo_t* d = &motor->debug_info;

    // 同步block_fwd数组
    d->block_fwd.count = motor->block_fwd.count;
    for(int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        if (i < d->block_fwd.count) {
            d->block_fwd.device_ids[i] = motor->block_fwd.commands[i].device_id;
        } else {
            d->block_fwd.device_ids[i] = DEV_ID_NONE;
        }
    }

    // 同步block_rev数组
    d->block_rev.count = motor->block_rev.count;
    for(int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        if (i < d->block_rev.count) {
            d->block_rev.device_ids[i] = motor->block_rev.commands[i].device_id;
        } else {
            d->block_rev.device_ids[i] = DEV_ID_NONE;
        }
    }

    // 同步allow_fwd数组
    d->allow_fwd.count = motor->allow_fwd.count;
    for(int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        if (i < d->allow_fwd.count) {
            d->allow_fwd.device_ids[i] = motor->allow_fwd.commands[i].device_id;
            d->allow_fwd.priorities[i] = motor->allow_fwd.commands[i].priority;
        } else {
            d->allow_fwd.device_ids[i] = DEV_ID_NONE;
            d->allow_fwd.priorities[i] = PRIO_NONE;
        }
    }

    // 同步allow_rev数组
    d->allow_rev.count = motor->allow_rev.count;
    for(int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        if (i < d->allow_rev.count) {
            d->allow_rev.device_ids[i] = motor->allow_rev.commands[i].device_id;
            d->allow_rev.priorities[i] = motor->allow_rev.commands[i].priority;
        } else {
            d->allow_rev.device_ids[i] = DEV_ID_NONE;
            d->allow_rev.priorities[i] = PRIO_NONE;
        }
    }

    d->state = motor->state;
    d->active_dir = motor->active_dir;
    d->active_device_id = motor->active_device_id;
    d->current_duty = motor->last_sent_duty;
    d->conflict_fault = motor->debug_info.conflict_fault;
    
    // ========== 同步到全局调试变量（供 Keil Watch 查看） ==========
    memcpy((void*)&g_stcMotorDebug, &motor->debug_info, sizeof(MotorDebugInfo_t));
    
    // 同步到单独变量
    g_u8MotorDebug_BlockFwdCount = motor->block_fwd.count;
    g_u8MotorDebug_BlockRevCount = motor->block_rev.count;
    g_u8MotorDebug_AllowFwdCount = motor->allow_fwd.count;
    g_u8MotorDebug_AllowRevCount = motor->allow_rev.count;
    g_u8MotorDebug_ActiveDeviceId = motor->active_device_id;
    g_u8MotorDebug_ActiveDir = motor->active_dir;
    g_u8MotorDebug_State = motor->state;
    g_fMotorDebug_CurrentDuty = motor->last_sent_duty;
    g_u8MotorDebug_ConflictFault = motor->debug_info.conflict_fault ? 1 : 0;
}

static void Motor_ArbitrationDecision(MotorDevice_t* motor) {
    if (!motor->enable) return;

    // 1. 指令候选选择
    MotorControlCommand_t* fwd_cmd = NULL;
    MotorControlCommand_t* rev_cmd = NULL;

    if(motor->block_fwd.count == 0 && motor->allow_fwd.count > 0) {
        fwd_cmd = &motor->allow_fwd.commands[0];
    }

    if(motor->block_rev.count == 0 && motor->allow_rev.count > 0) {
        rev_cmd = &motor->allow_rev.commands[0];
    }

    // 2. 冲突判定
    MotorControlCommand_t* final = NULL;
    motor->debug_info.conflict_fault = false;

    if (fwd_cmd && rev_cmd) {
        if (fwd_cmd->device_id == rev_cmd->device_id) {
            final = NULL;
            motor->debug_info.conflict_fault = true;
        } else if (fwd_cmd->priority < rev_cmd->priority) {
            final = fwd_cmd;
        } else if (rev_cmd->priority < fwd_cmd->priority) {
            final = rev_cmd;
        } else {
            final = NULL;
        }
    } else {
        final = fwd_cmd ? fwd_cmd : rev_cmd;
    }

    // 3. 执行决策 - 调试打印
    if (final) {
        MOTOR_DEBUG("Arbitration: Selected %s (prio=%d), dir=%s, duty=%.1f%%\r\n",
                    (final->device_id == DEV_ID_IO_FWD) ? "IO_FWD" :
                    (final->device_id == DEV_ID_IO_REV) ? "IO_REV" :
                    (final->device_id == DEV_ID_RTURN_FWD) ? "RTURN_FWD" :
                    (final->device_id == DEV_ID_RTURN_REV) ? "RTURN_REV" :
					(final->device_id == DEV_ID_OVERCUR_FWD) ? "OVERCUR_FWD" :
                    (final->device_id == DEV_ID_CAN) ? "CAN" :
                    (final->device_id == DEV_ID_POWER_POS) ? "POWER_POS" :
                    (final->device_id == DEV_ID_POWER_NEG) ? "POWER_NEG" : "UNKNOWN",
                    final->priority,
                    (final->type == CMD_TYPE_RUN_FWD || final->type == CMD_TYPE_RAMP_FWD) ? "FWD" : "REV",
                    final->param);
        
        motor->active_device_id = final->device_id;
        motor->active_dir = (final->type == CMD_TYPE_RUN_FWD || final->type == CMD_TYPE_RAMP_FWD) ? DIR_FWD : DIR_REV;
        motor->last_sent_duty = final->param;
        motor->state = (final->type == CMD_TYPE_RAMP_FWD || final->type == CMD_TYPE_RAMP_REV) ? MS_RAMPING : MS_RUNNING;

        // 调用仲裁结果回调
        if (motor->active_dir == DIR_FWD) {
            Motor_OnArbitrationFwd(motor, motor->last_sent_duty);
        } else {
            Motor_OnArbitrationRev(motor, motor->last_sent_duty);
        }

        // 调用PWM设备控制
        DeviceCommandData_t cmd_data;
        PWM_RampCmd_t ramp_cmd;

        if (final->type == CMD_TYPE_RAMP_FWD || final->type == CMD_TYPE_RAMP_REV) {
            cmd_data.cmd = CMD_PWM_SET_DUTY_SLOW;
            ramp_cmd.targetDuty = (uint16_t)(final->param);
            ramp_cmd.rampTimeMs = 2000;  // 2秒缓启动
            cmd_data.params = &ramp_cmd;
            cmd_data.param_size = sizeof(PWM_RampCmd_t);
        } else {
            cmd_data.cmd = CMD_PWM_SET_DUTY;
            cmd_data.params = &final->param;
            cmd_data.param_size = sizeof(uint16_t);
        }
        Device_Control(DEV_PWM_MOTOR_ID, &cmd_data);
    } else {
        MOTOR_DEBUG("Arbitration: No command selected - STOP\r\n");
        
        motor->last_sent_duty = 0.0f;
        motor->state = MS_IDLE;
        motor->active_dir = DIR_NONE;
        motor->active_device_id = DEV_ID_NONE;

        // 调用仲裁停止回调
        Motor_OnArbitrationStop(motor);

        // 停止PWM
        DeviceCommandData_t cmd_data;
        cmd_data.cmd = CMD_PWM_STOP;
        cmd_data.params = NULL;
        Device_Control(DEV_PWM_MOTOR_ID, &cmd_data);
    }

    Motor_UpdateDebugInfo(motor);
}

// ========== EventBus回调函数 ==========

void Motor_OnOvercurrent(void* payload) {
    MotorOvercurrentEvent_t* ev = (MotorOvercurrentEvent_t*)payload;
    MAIN_D("Motor: Overcurrent event! ADC=%d, Current=%d mA, Threshold=%d mA, Duration=%lu ms\r\n",
           ev->adc_id, ev->current_ma, ev->threshold_ma, ev->duration_ms);

    // 执行紧急停止
    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (node && node->private_data) {
        MotorDevice_t* motor = (MotorDevice_t*)node->private_data;
        Motor_EmergencyStop(motor);
        MAIN_D("Motor: Emergency stop due to overcurrent!\r\n");
    }
}

// ========== 电压告警回调 ==========
void Motor_OnVoltageAlarm(void* payload) {
    Voltage_AlarmEvent_t* ev = (Voltage_AlarmEvent_t*)payload;

    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    if (ev->u8IsActive) {
        // 告警触发 - 根据告警类型使用对应的设备ID向 block_fwd/block_rev 添加阻塞指令
        // 使用 PRIO_LIMIT 优先级，与限位开关同级，确保其他设备无法覆盖
        MotorDeviceId_t dev_id_fwd, dev_id_rev;
        const char* alarm_name;

        if (ev->u8AlarmType == VOLTAGE_ALARM_OVERVOLTAGE) {
            dev_id_fwd = DEV_ID_OVERVOLTAGE_FWD;
            dev_id_rev = DEV_ID_OVERVOLTAGE_REV;
            alarm_name = "OVERVOLTAGE";
        } else {
            dev_id_fwd = DEV_ID_UNDERVOLTAGE_FWD;
            dev_id_rev = DEV_ID_UNDERVOLTAGE_REV;
            alarm_name = "UNDERVOLTAGE";
        }

        MotorControlCommand_t block_fwd_cmd = {
            .device_id = dev_id_fwd,
            .priority = PRIO_LIMIT,
            .type = CMD_TYPE_BLOCK_FWD,
            .timestamp = tickTimer_GetCount()
        };
        Motor_CmdList_SetBlock(&motor->block_fwd, block_fwd_cmd);

        MotorControlCommand_t block_rev_cmd = {
            .device_id = dev_id_rev,
            .priority = PRIO_LIMIT,
            .type = CMD_TYPE_BLOCK_REV,
            .timestamp = tickTimer_GetCount()
        };
        Motor_CmdList_SetBlock(&motor->block_rev, block_rev_cmd);

        MAIN_D("Motor: %s alarm! Bus=%lu mV, Threshold=%lu mV - Block both directions!\r\n",
               alarm_name, ev->u32BusVoltageMv, ev->u32ThresholdMv);
    } else {
        // 告警解除 - 根据告警类型使用对应的设备ID从 block_fwd/block_rev 移除阻塞指令
        MotorDeviceId_t dev_id_fwd, dev_id_rev;
        const char* alarm_name;

        if (ev->u8AlarmType == VOLTAGE_ALARM_OVERVOLTAGE) {
            dev_id_fwd = DEV_ID_OVERVOLTAGE_FWD;
            dev_id_rev = DEV_ID_OVERVOLTAGE_REV;
            alarm_name = "OVERVOLTAGE";
        } else {
            dev_id_fwd = DEV_ID_UNDERVOLTAGE_FWD;
            dev_id_rev = DEV_ID_UNDERVOLTAGE_REV;
            alarm_name = "UNDERVOLTAGE";
        }

        Motor_CmdList_Remove(&motor->block_fwd, dev_id_fwd);
        Motor_CmdList_Remove(&motor->block_rev, dev_id_rev);

        MAIN_D("Motor: %s released! Bus=%lu mV - Unblock both directions\r\n",
               alarm_name, ev->u32BusVoltageMv);
    }

    Motor_ArbitrationDecision(motor);
}

void Motor_OnRTurnLimit(void* payload) {
    RTurn_LimitEvent_t* ev = (RTurn_LimitEvent_t*)payload;

    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    if (ev->u8IsActive) {
        // 限位触发，添加 BLOCK 指令（使用 RTurn 专用设备ID）
        MotorControlCommand_t cmd = {
            .device_id = (ev->u8Direction == RTURN_LIMIT_FORWARD) ? DEV_ID_RTURN_FWD : DEV_ID_RTURN_REV,
            .priority = PRIO_LIMIT,
            .timestamp = tickTimer_GetCount()
        };

        if (ev->u8Direction == RTURN_LIMIT_FORWARD) {
            cmd.type = CMD_TYPE_BLOCK_FWD;
            Motor_CmdList_SetBlock(&motor->block_fwd, cmd);
            // 同时清除 allow_fwd，确保即使 block 被移除，电机也不会继续正转
            Motor_ClearAllowFwd(motor);
            MAIN_D("Motor: RTurn forward limit triggered, block forward + clear allow_fwd\r\n");
        } else {
            cmd.type = CMD_TYPE_BLOCK_REV;
            Motor_CmdList_SetBlock(&motor->block_rev, cmd);
            // 同时清除 allow_rev，确保即使 block 被移除，电机也不会继续反转
            Motor_ClearAllowRev(motor);
            MAIN_D("Motor: RTurn reverse limit triggered, block reverse + clear allow_rev\r\n");
        }
    } else {
        // 限位解除，移除 BLOCK 指令（使用 RTurn 专用设备ID）
        MotorDeviceId_t id = (ev->u8Direction == RTURN_LIMIT_FORWARD) ? DEV_ID_RTURN_FWD : DEV_ID_RTURN_REV;
        if (ev->u8Direction == RTURN_LIMIT_FORWARD) {
            Motor_CmdList_Remove(&motor->block_fwd, id);
            MAIN_D("Motor: RTurn forward limit released\r\n");
        } else if (ev->u8Direction == RTURN_LIMIT_REVERSE) {
            Motor_CmdList_Remove(&motor->block_rev, id);
            MAIN_D("Motor: RTurn reverse limit released\r\n");
        } else {
            // 方向为0时，清除所有限位
            Motor_CmdList_Remove(&motor->block_fwd, DEV_ID_RTURN_FWD);
            Motor_CmdList_Remove(&motor->block_rev, DEV_ID_RTURN_REV);
        }
    }

    Motor_ArbitrationDecision(motor);
}

// ========== 电流告警回调 ==========
void Motor_OnCurrentAlarm(void* payload) {
    Current_AlarmEvent_t* ev = (Current_AlarmEvent_t*)payload;

    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    if (ev->u8IsActive) {
        /* 判断当前电机方向：仅正转时过流才阻塞正转
         * 反转堵转属于预期工况（如阀门关闭到位），不阻塞正转 */
        MotorDir_t eDesiredDir = Motor_GetDesiredDirection(motor);
        
        if (eDesiredDir == DIR_FWD)
        {
            // 正转时过流 → 向 block_fwd 添加 DEV_ID_OVERCUR_FWD 阻塞指令
            MotorControlCommand_t block_fwd_cmd = {
                .device_id = DEV_ID_OVERCUR_FWD,
                .priority = PRIO_LIMIT,
                .type = CMD_TYPE_BLOCK_FWD,
                .timestamp = tickTimer_GetCount()
            };
            Motor_CmdList_SetBlock(&motor->block_fwd, block_fwd_cmd);

            MAIN_D("Motor: OVERCURRENT alarm (FWD)! Current=%ld mA, Threshold=%ld mA - Block forward!\r\n",
                   (long)ev->s32CurrentMa, (long)ev->s32ThresholdMa);
        }
        else
        {
            // 反转或停止时过流 → 不阻塞正转（反转堵转属于预期工况）
            MAIN_D("Motor: OVERCURRENT alarm (dir=%d, REV/STOP is normal stall)! Current=%ld mA, Threshold=%ld mA - Skip block forward\r\n",
                   (int)eDesiredDir, (long)ev->s32CurrentMa, (long)ev->s32ThresholdMa);
        }
    } else {
        // 过流告警解除 - 从 block_fwd 移除 DEV_ID_OVERCUR_FWD
        Motor_CmdList_Remove(&motor->block_fwd, DEV_ID_OVERCUR_FWD);

        MAIN_D("Motor: OVERCURRENT released! Current=%ld mA - Unblock forward\r\n",
               (long)ev->s32CurrentMa);
    }

    Motor_ArbitrationDecision(motor);
}


void Motor_OnHardLimit(void* payload) {
    MotorLimitEvent_t* ev = (MotorLimitEvent_t*)payload;
    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    MotorDeviceId_t id = (ev->dir == DIR_FWD) ? DEV_ID_LIMIT_FWD : DEV_ID_LIMIT_REV;

    if (ev->is_active) {
        MotorControlCommand_t cmd = {
            .device_id = id,
            .timestamp = tickTimer_GetCount()
        };

        if (ev->dir == DIR_FWD) {
            cmd.type = CMD_TYPE_BLOCK_FWD;
            Motor_CmdList_SetBlock(&motor->block_fwd, cmd);
        } else {
            cmd.type = CMD_TYPE_BLOCK_REV;
            Motor_CmdList_SetBlock(&motor->block_rev, cmd);
        }
    } else {
        if (ev->dir == DIR_FWD) {
            Motor_CmdList_Remove(&motor->block_fwd, id);
        } else {
            Motor_CmdList_Remove(&motor->block_rev, id);
        }
    }

    Motor_ArbitrationDecision(motor);
}

void Motor_OnPowerEvent(void* payload) {
    MotorPowerEvent_t* ev = (MotorPowerEvent_t*)payload;
    MAIN_D("[MOTOR] OnPowerEvent: power_id=%d, is_on=%d\r\n", ev->power_id, ev->is_on);

    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

#if MOTOR_MODE_BIPOLAR
    // ========== 双极性模式 ==========
    // 电源设备提供 ALLOW 控制电机转向，不设置 BLOCK
    MotorDeviceId_t id = (ev->power_id == 0) ? DEV_ID_POWER_POS : DEV_ID_POWER_NEG;
    const MotorDeviceGene_t* gene = Motor_GetGene(id);
    if (!gene) return;

    MotorControlCommand_t cmd = {
        .device_id = id,
        .priority = gene->priority,
        .param = 85.0f,
        .timestamp = tickTimer_GetCount()
    };

    if (ev->is_on) {
        if (id == DEV_ID_POWER_POS) {
            cmd.type = CMD_TYPE_RUN_FWD;
            Motor_CmdList_SetAllow(&motor->allow_fwd, cmd);
            MAIN_D("[MOTOR] Bipolar: POWER_POS ON, added allow_fwd\r\n");
        } else {
            cmd.type = CMD_TYPE_RUN_REV;
            Motor_CmdList_SetAllow(&motor->allow_rev, cmd);
            MAIN_D("[MOTOR] Bipolar: POWER_NEG ON, added allow_rev\r\n");
        }
    } else {
        if (id == DEV_ID_POWER_POS) {
            Motor_CmdList_Remove(&motor->allow_fwd, id);
            MAIN_D("[MOTOR] Bipolar: POWER_POS OFF, removed allow_fwd\r\n");
        } else {
            Motor_CmdList_Remove(&motor->allow_rev, id);
            MAIN_D("[MOTOR] Bipolar: POWER_NEG OFF, removed allow_rev\r\n");
        }
    }
#else
    // ========== 单极性模式 ==========
    // 单极性模式下电源设备不参与仲裁，不做任何操作
    // 其他设备（如手动IO）提供某转向的ALLOW即可转
    MAIN_D("[MOTOR] Unipolar mode: power_id=%d, is_on=%d - power device not involved in arbitration\r\n",
           ev->power_id, ev->is_on);
#endif

    Motor_ArbitrationDecision(motor);
}

void Motor_OnManualIO(void* payload) {
    MotorManualIOEvent_t* ev = (MotorManualIOEvent_t*)payload;
    
    // 先检查事件数据的有效性
    if (ev == NULL) {
        MAIN_D("[MOTOR] OnManualIO: NULL payload!\r\n");
        return;
    }
    
    // 打印原始事件数据（使用十六进制打印浮点数）
    uint32_t speed_raw = *(uint32_t*)&ev->speed;
    MAIN_D("[MOTOR] OnManualIO RAW: dir=%d, type=%d, speed_raw=0x%08X\r\n", 
           ev->dir, ev->type, speed_raw);
    
    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) {
        MAIN_D("[MOTOR] OnManualIO: Device not found!\r\n");
        return;
    }

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    // 根据方向确定是哪个IO设备 - 必须初始化默认值！
    MotorDeviceId_t io_dev_id = DEV_ID_NONE;  // 初始化为 NONE
    
    if (ev->dir == DIR_FWD) {
        io_dev_id = DEV_ID_IO_FWD;
    } else if (ev->dir == DIR_REV) {
        io_dev_id = DEV_ID_IO_REV;
    } else if (ev->dir == DIR_NONE) {
        io_dev_id = DEV_ID_NONE;
    } else {
        // 无效的 dir 值
        MAIN_D("[MOTOR] OnManualIO: Invalid dir=%d, expected 0,1,2\r\n", ev->dir);
        return;
    }

    const MotorDeviceGene_t* gene = NULL;
    if (io_dev_id != DEV_ID_NONE) {
        gene = Motor_GetGene(io_dev_id);
        if (!gene) {
            MAIN_D("[MOTOR] OnManualIO: gene not found for dev_id=%d!\r\n", io_dev_id);
            return;
        }
    }
    
    // 打印调试信息 - 浮点数转换为整型
    int32_t speed_int = (int32_t)(ev->speed * 10);
    MAIN_D("[MOTOR] OnManualIO: dir=%d, type=%d, speed=%ld.%ld%%, io_dev_id=%d\r\n",
           ev->dir, ev->type, (long)(speed_int / 10), (long)(speed_int % 10), io_dev_id);
    MAIN_D("[MOTOR] OnManualIO: Before - allow_fwd.count=%d, allow_rev.count=%d, block_fwd.count=%d, block_rev.count=%d\r\n",
           motor->allow_fwd.count, motor->allow_rev.count, motor->block_fwd.count, motor->block_rev.count);

    // 处理运行指令
    if (ev->type == CMD_TYPE_RUN_FWD || ev->type == CMD_TYPE_RAMP_FWD) {
        // 检查 io_dev_id 是否正确
        if (io_dev_id != DEV_ID_IO_FWD) {
            MAIN_D("[MOTOR] ERROR: RUN_FWD but io_dev_id=%d (expected %d)\r\n", io_dev_id, DEV_ID_IO_FWD);
            return;
        }
        
        // IO_FWD按下：移除自己的BLOCK，添加自己的ALLOW
        MotorControlCommand_t cmd = {
            .device_id = io_dev_id,
            .priority = gene->priority,
            .type = ev->type,
            .param = ev->speed,
            .timestamp = tickTimer_GetCount()
        };

        Motor_CmdList_Remove(&motor->block_fwd, io_dev_id);
        Motor_CmdList_SetAllow(&motor->allow_fwd, cmd);
        
        MAIN_D("[MOTOR] IO_FWD: RUN, removed block_fwd, added allow_fwd\r\n");
    }
    else if (ev->type == CMD_TYPE_RUN_REV || ev->type == CMD_TYPE_RAMP_REV) {
        // 检查 io_dev_id 是否正确
        if (io_dev_id != DEV_ID_IO_REV) {
            MAIN_D("[MOTOR] ERROR: RUN_REV but io_dev_id=%d (expected %d)\r\n", io_dev_id, DEV_ID_IO_REV);
            return;
        }
        
        // IO_REV按下：移除自己的BLOCK，添加自己的ALLOW
        MotorControlCommand_t cmd = {
            .device_id = io_dev_id,
            .priority = gene->priority,
            .type = ev->type,
            .param = ev->speed,
            .timestamp = tickTimer_GetCount()
        };

        Motor_CmdList_Remove(&motor->block_rev, io_dev_id);
        Motor_CmdList_SetAllow(&motor->allow_rev, cmd);
        
        MAIN_D("[MOTOR] IO_REV: RUN, removed block_rev, added allow_rev\r\n");
    }
    else if (ev->type == CMD_TYPE_STOP) {
        if (ev->dir == DIR_FWD) {
            // IO_FWD停止：移除自己的ALLOW，添加自己的BLOCK
            if (gene == NULL) {
                gene = Motor_GetGene(DEV_ID_IO_FWD);
                if (!gene) {
                    MAIN_D("[MOTOR] OnManualIO: gene not found for IO_FWD!\r\n");
                    return;
                }
            }
            
            Motor_CmdList_Remove(&motor->allow_fwd, DEV_ID_IO_FWD);
            
            MotorControlCommand_t block_cmd = {
                .device_id = DEV_ID_IO_FWD,
                .priority = gene->priority,
                .type = CMD_TYPE_BLOCK_FWD,
                .timestamp = tickTimer_GetCount()
            };
            Motor_CmdList_SetBlock(&motor->block_fwd, block_cmd);
            
            MAIN_D("[MOTOR] IO_FWD: STOP, removed allow_fwd, added block_fwd\r\n");
        }
        else if (ev->dir == DIR_REV) {
            // IO_REV停止：移除自己的ALLOW，添加自己的BLOCK
            if (gene == NULL) {
                gene = Motor_GetGene(DEV_ID_IO_REV);
                if (!gene) {
                    MAIN_D("[MOTOR] OnManualIO: gene not found for IO_REV!\r\n");
                    return;
                }
            }
            
            Motor_CmdList_Remove(&motor->allow_rev, DEV_ID_IO_REV);
            
            MotorControlCommand_t block_cmd = {
                .device_id = DEV_ID_IO_REV,
                .priority = gene->priority,
                .type = CMD_TYPE_BLOCK_REV,
                .timestamp = tickTimer_GetCount()
            };
            Motor_CmdList_SetBlock(&motor->block_rev, block_cmd);
            
            MAIN_D("[MOTOR] IO_REV: STOP, removed allow_rev, added block_rev\r\n");
        }
        else {
            // 停止所有IO设备：移除各自的ALLOW，添加各自的BLOCK
            const MotorDeviceGene_t* gene_fwd = Motor_GetGene(DEV_ID_IO_FWD);
            const MotorDeviceGene_t* gene_rev = Motor_GetGene(DEV_ID_IO_REV);
            
            Motor_CmdList_Remove(&motor->allow_fwd, DEV_ID_IO_FWD);
            Motor_CmdList_Remove(&motor->allow_rev, DEV_ID_IO_REV);
            
            if (gene_fwd) {
                MotorControlCommand_t block_fwd_cmd = {
                    .device_id = DEV_ID_IO_FWD,
                    .priority = gene_fwd->priority,
                    .type = CMD_TYPE_BLOCK_FWD,
                    .timestamp = tickTimer_GetCount()
                };
                Motor_CmdList_SetBlock(&motor->block_fwd, block_fwd_cmd);
            }
            
            if (gene_rev) {
                MotorControlCommand_t block_rev_cmd = {
                    .device_id = DEV_ID_IO_REV,
                    .priority = gene_rev->priority,
                    .type = CMD_TYPE_BLOCK_REV,
                    .timestamp = tickTimer_GetCount()
                };
                Motor_CmdList_SetBlock(&motor->block_rev, block_rev_cmd);
            }
            
            MAIN_D("[MOTOR] All IO: STOP, removed all allow, added all block\r\n");
        }
    }
    else {
        MAIN_D("[MOTOR] OnManualIO: Unknown type=%d\r\n", ev->type);
        return;
    }

    Motor_ArbitrationDecision(motor);
}

void Motor_OnCANEvent(void* payload) {
    // CAN事件处理（暂时保留接口，未实现）
    // MotorCANEvent_t* ev = (MotorCANEvent_t*)payload;
    (void)payload;  // 避免未使用警告
}

// ========== 新增：电机霍尔转速反馈回调 ==========
void Motor_OnSpeedFeedback(void* payload) {
    int32_t* speed_rpm = (int32_t*)payload;

    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    // 可选：根据转速进行闭环控制
    // 例如：根据目标转速调整占空比
    if (motor->state == MS_RUNNING || motor->state == MS_RAMPING) {
        // 可以在这里实现 PID 调速
        // 目前只是接收转速数据，供调试或后续扩展使用
        (void)motor;      // 避免未使用警告
        (void)speed_rpm;  // 避免未使用警告
    }
}

// ========== 标准设备操作接口实现 ==========
DeviceResult_t Motor_Init(void* handle) {
    MotorDevice_t* motor = (MotorDevice_t*)handle;
    if (!motor) return RESULT_PARAM_ERR;

    memset(motor, 0, sizeof(MotorDevice_t));
    
    // 初始化方向跟踪变量
    s_eLastArbitrationDir = DIR_NONE;
    s_u16LastTargetDuty = 0;
    s_bStopPolarityPending = false;

    // 初始化命令列表
    for(int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        motor->allow_fwd.commands[i].device_id = DEV_ID_NONE;
        motor->allow_fwd.commands[i].priority = PRIO_NONE;
        motor->allow_rev.commands[i].device_id = DEV_ID_NONE;
        motor->allow_rev.commands[i].priority = PRIO_NONE;
        motor->block_fwd.commands[i].device_id = DEV_ID_NONE;
        motor->block_rev.commands[i].device_id = DEV_ID_NONE;
    }

    // 初始化IO设备默认状态
    MotorControlCommand_t io_fwd_block = {
        .device_id = DEV_ID_IO_FWD,
        .priority = MANUAL_PRIORITY,
        .type = CMD_TYPE_BLOCK_FWD,
        .timestamp = tickTimer_GetCount()
    };
    Motor_CmdList_SetBlock(&motor->block_fwd, io_fwd_block);
    
    MotorControlCommand_t io_rev_block = {
        .device_id = DEV_ID_IO_REV,
        .priority = MANUAL_PRIORITY,
        .type = CMD_TYPE_BLOCK_REV,
        .timestamp = tickTimer_GetCount()
    };
    Motor_CmdList_SetBlock(&motor->block_rev, io_rev_block);

    motor->enable = 1;
    motor->last_arbitration_time = tickTimer_GetCount();

    // 设置调试全局指针
    g_pMotor_Dev_Watch = motor;
    
    // 初始化调试变量
    Motor_UpdateDebugInfo(motor);

    // ========== 关键修复：同步硬件实际占空比到PWM结构体 ==========
    uint32_t period = TMRA_GetPeriodValue(CM_TMRA_4);
    if (period > 0) {
        uint32_t cmp1 = TMRA_GetCompareValue(CM_TMRA_4, TMRA_CH1);
        uint32_t cmp2 = TMRA_GetCompareValue(CM_TMRA_4, TMRA_CH2);
//        uint32_t cmp3 = TMRA_GetCompareValue(CM_TMRA_4, TMRA_CH3);
//        uint32_t cmp4 = TMRA_GetCompareValue(CM_TMRA_4, TMRA_CH4);

        uint16_t duty1 = (uint16_t)((cmp1 * 100) / period);
        uint16_t duty2 = (uint16_t)((cmp2 * 100) / period);
//        uint16_t duty3 = (uint16_t)((cmp3 * 100) / period);
//        uint16_t duty4 = (uint16_t)((cmp4 * 100) / period);

        // 直接更新PWM结构体中的占空比
        g_motor_pwm_ch1.dutyPercent = duty1;
        g_motor_pwm_ch2.dutyPercent = duty2;
//        g_motor_pwm_ch3.dutyPercent = duty3;
//        g_motor_pwm_ch4.dutyPercent = duty4;

        MAIN_D("[MOTOR] Init: synced PWM duties from hardware: CH1=%d%%, CH2=%d%%\r\n",
               duty1, duty2);
    }
    
    MOTOR_DEBUG("Motor_Init completed, g_pMotor_Dev_Watch set\r\n");
    return RESULT_OK;
}

DeviceResult_t Motor_Deinit(void* handle) {
    MotorDevice_t* motor = (MotorDevice_t*)handle;
    if (!motor) return RESULT_PARAM_ERR;

    motor->enable = 0;
    Motor_EmergencyStop(motor);
    
    g_pMotor_Dev_Watch = NULL;
    memset((void*)&g_stcMotorDebug, 0, sizeof(MotorDebugInfo_t));

    return RESULT_OK;
}

DeviceResult_t Motor_Read(void* handle, void* data, uint32_t size) {
    MotorDevice_t* motor = (MotorDevice_t*)handle;
    if (!motor || !data) return RESULT_PARAM_ERR;
    if (!motor->enable) return RESULT_ERROR;

    if (size == sizeof(MotorDebugInfo_t)) {
        memcpy(data, &motor->debug_info, sizeof(MotorDebugInfo_t));
        return RESULT_OK;
    }
    // 新增：支持读取状态结构体
    else if (size == sizeof(Motor_StateInfo_t)) {
        Motor_StateInfo_t* pstcState = (Motor_StateInfo_t*)data;
        pstcState->desired_dir = DIR_NONE;
        if (motor->state == MS_RUNNING || motor->state == MS_RAMPING) {
            pstcState->desired_dir = motor->active_dir;
        }
        pstcState->state = motor->state;
        pstcState->active_dir = motor->active_dir;
        pstcState->current_duty = motor->current_duty;
        pstcState->enable = motor->enable;
        return RESULT_OK;
    }

    return RESULT_PARAM_ERR;
}

DeviceResult_t Motor_Write(void* handle, const void* data, uint32_t size) {
    // 电机设备通常不需要直接写操作
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t Motor_Control(void* handle, DeviceCommandData_t* cmd) {
    MotorDevice_t* motor = (MotorDevice_t*)handle;
    if (!motor || !cmd) return RESULT_PARAM_ERR;

    switch(cmd->cmd) {
        case CMD_MOTOR_STOP:
            Motor_Stop(motor);
            break;
        case CMD_MOTOR_RUN_FWD:
            Motor_Start(motor, DIR_FWD);
            break;
        case CMD_MOTOR_RUN_REV:
            Motor_Start(motor, DIR_REV);
            break;
        case CMD_MOTOR_SET_SPEED:
            if (cmd->params && cmd->param_size == sizeof(float)) {
                Motor_SetSpeed(motor, *(float*)cmd->params);
            }
            break;
        case CMD_MOTOR_EMERGENCY_STOP:
            Motor_EmergencyStop(motor);
            break;
        case CMD_MOTOR_GET_DESIRED_DIR:
            if (cmd->response && cmd->response_size >= sizeof(uint8_t)) {
                MotorDir_t desired_dir = DIR_NONE;
                if (motor->state == MS_RUNNING || motor->state == MS_RAMPING) {
                    desired_dir = motor->active_dir;
                }
                *(uint8_t*)cmd->response = (uint8_t)desired_dir;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
        default:
            return RESULT_ERROR;
    }

    return RESULT_OK;
}

DeviceResult_t Motor_Update(void* handle) {
    MotorDevice_t* motor = (MotorDevice_t*)handle;
    if (!motor || !motor->enable) return RESULT_ERROR;

    uint32_t now = tickTimer_GetCount();

    // ========== 新增：更新所有PWM通道的缓启动状态 ==========
    PWM_Update(&g_motor_pwm_ch1);
    PWM_Update(&g_motor_pwm_ch2);
//    PWM_Update(&g_motor_pwm_ch3);
//    PWM_Update(&g_motor_pwm_ch4);

    // ========== 新增：检查缓停是否完成，需要切换停止极性 ==========
    if (s_bStopPolarityPending) {
        // 检查所有通道是否都完成了缓启动（状态为IDLE）
        if (PWM_GetState(&g_motor_pwm_ch1) == PWM_STATE_IDLE &&
            PWM_GetState(&g_motor_pwm_ch2) == PWM_STATE_IDLE) {
//            PWM_GetState(&g_motor_pwm_ch3) == PWM_STATE_IDLE &&
//            PWM_GetState(&g_motor_pwm_ch4) == PWM_STATE_IDLE) {
            
            // 缓停完成，切换到停止极性
            Motor_SetStopDuty();
            s_bStopPolarityPending = false;
            MAIN_D("[MOTOR] Stop ramp complete, switched to stop polarity\r\n");
        }
    }

    // 每50ms仲裁一次
    if (now - motor->last_arbitration_time >= 50) {
        Motor_ArbitrationDecision(motor);
        motor->last_arbitration_time = now;
    }

    // ========== 每2000ms打印一次电机仲裁状态 ==========
    if (now - s_u32LastMotorPrintTime >= MOTOR_PRINT_INTERVAL_MS) {
        s_u32LastMotorPrintTime = now;

        MOTOR_OUT("=== Motor Arbitration State ===\r\n");

        // 打印当前状态
        const char* state_str = (motor->state == MS_IDLE) ? "IDLE" :
                                (motor->state == MS_RAMPING) ? "RAMPING" :
                                (motor->state == MS_RUNNING) ? "RUNNING" : "LOCKED";
        const char* dir_str = (motor->active_dir == DIR_FWD) ? "FWD" :
                              (motor->active_dir == DIR_REV) ? "REV" : "NONE";
        MOTOR_OUT("  State=%s, ActiveDir=%s, Duty=%.1f%%\r\n",
                  state_str, dir_str, motor->current_duty);

        // 打印正在作用的允许指令
        MOTOR_OUT("  Active Allow FWD: ");
        if (motor->allow_fwd.count > 0) {
            for (int i = 0; i < motor->allow_fwd.count; i++) {
                const char* dev_name = "";
                switch(motor->allow_fwd.commands[i].device_id) {
                    case DEV_ID_IO_FWD: dev_name = "IO_FWD"; break;
                    case DEV_ID_IO_REV: dev_name = "IO_REV"; break;
                    case DEV_ID_CAN: dev_name = "CAN"; break;
                    case DEV_ID_POWER_POS: dev_name = "POWER_POS"; break;
                    case DEV_ID_POWER_NEG: dev_name = "POWER_NEG"; break;
                    default: dev_name = "UNKNOWN"; break;
                }
                MOTOR_OUT("%s(Prio%d) ", dev_name, motor->allow_fwd.commands[i].priority);
            }
        } else {
            MOTOR_OUT("(none)");
        }
        MOTOR_OUT("\r\n");

        MOTOR_OUT("  Active Allow REV: ");
        if (motor->allow_rev.count > 0) {
            for (int i = 0; i < motor->allow_rev.count; i++) {
                const char* dev_name = "";
                switch(motor->allow_rev.commands[i].device_id) {
                    case DEV_ID_IO_FWD: dev_name = "IO_FWD"; break;
                    case DEV_ID_IO_REV: dev_name = "IO_REV"; break;
                    case DEV_ID_CAN: dev_name = "CAN"; break;
                    case DEV_ID_POWER_POS: dev_name = "POWER_POS"; break;
                    case DEV_ID_POWER_NEG: dev_name = "POWER_NEG"; break;
                    default: dev_name = "UNKNOWN"; break;
                }
                MOTOR_OUT("%s(Prio%d) ", dev_name, motor->allow_rev.commands[i].priority);
            }
        } else {
            MOTOR_OUT("(none)");
        }
        MOTOR_OUT("\r\n");

        // 打印正在作用的阻塞指令
        MOTOR_OUT("  Active Block FWD: ");
        if (motor->block_fwd.count > 0) {
            for (int i = 0; i < motor->block_fwd.count; i++) {
                const char* dev_name = "";
                switch(motor->block_fwd.commands[i].device_id) {
                    case DEV_ID_IO_FWD: dev_name = "IO_FWD"; break;
                    case DEV_ID_IO_REV: dev_name = "IO_REV"; break;
                    case DEV_ID_CAN: dev_name = "CAN"; break;
                    case DEV_ID_LIMIT_FWD: dev_name = "LIMIT_FWD"; break;
                    case DEV_ID_LIMIT_REV: dev_name = "LIMIT_REV"; break;
                    case DEV_ID_POWER_POS: dev_name = "POWER_POS"; break;
                    case DEV_ID_POWER_NEG: dev_name = "POWER_NEG"; break;
                    case DEV_ID_RTURN_FWD: dev_name = "RTURN_FWD"; break;
                    case DEV_ID_RTURN_REV: dev_name = "RTURN_REV"; break;
                    case DEV_ID_OVERVOLTAGE_FWD: dev_name = "OVERVOLTAGE_FWD"; break;
                    case DEV_ID_OVERVOLTAGE_REV: dev_name = "OVERVOLTAGE_REV"; break;
                    case DEV_ID_UNDERVOLTAGE_FWD: dev_name = "UNDERVOLTAGE_FWD"; break;
                    case DEV_ID_UNDERVOLTAGE_REV: dev_name = "UNDERVOLTAGE_REV"; break;
                    case DEV_ID_OVERCUR_FWD: dev_name = "OVERCUR_FWD"; break;
                    default: dev_name = "UNKNOWN"; break;
                }
                MOTOR_OUT("%s ", dev_name);
            }
        } else {
            MOTOR_OUT("(none)");
        }
        MOTOR_OUT("\r\n");

        MOTOR_OUT("  Active Block REV: ");
        if (motor->block_rev.count > 0) {
            for (int i = 0; i < motor->block_rev.count; i++) {
                const char* dev_name = "";
                switch(motor->block_rev.commands[i].device_id) {
                    case DEV_ID_IO_FWD: dev_name = "IO_FWD"; break;
                    case DEV_ID_IO_REV: dev_name = "IO_REV"; break;
                    case DEV_ID_CAN: dev_name = "CAN"; break;
                    case DEV_ID_LIMIT_FWD: dev_name = "LIMIT_FWD"; break;
                    case DEV_ID_LIMIT_REV: dev_name = "LIMIT_REV"; break;
                    case DEV_ID_POWER_POS: dev_name = "POWER_POS"; break;
                    case DEV_ID_POWER_NEG: dev_name = "POWER_NEG"; break;
                    case DEV_ID_RTURN_FWD: dev_name = "RTURN_FWD"; break;
                    case DEV_ID_RTURN_REV: dev_name = "RTURN_REV"; break;
                    case DEV_ID_OVERVOLTAGE_FWD: dev_name = "OVERVOLTAGE_FWD"; break;
                    case DEV_ID_OVERVOLTAGE_REV: dev_name = "OVERVOLTAGE_REV"; break;
                    case DEV_ID_UNDERVOLTAGE_FWD: dev_name = "UNDERVOLTAGE_FWD"; break;
                    case DEV_ID_UNDERVOLTAGE_REV: dev_name = "UNDERVOLTAGE_REV"; break;
                    case DEV_ID_OVERCUR_FWD: dev_name = "OVERCUR_FWD"; break;
                    default: dev_name = "UNKNOWN"; break;
                }
                MOTOR_OUT("%s ", dev_name);
            }
        } else {
            MOTOR_OUT("(none)");
        }
        MOTOR_OUT("\r\n");

        // ========== 打印已注册但未作用的设备 ==========
        // 根据设备基因表，检查哪些设备没有在 allow 或 block 列表中
        
        // 收集已作用的设备ID
        uint8_t active_devices[16] = {0};
        uint8_t active_count = 0;
        
        // 添加 allow_fwd 中的设备
        for (int i = 0; i < motor->allow_fwd.count; i++) {
            uint8_t dev_id = motor->allow_fwd.commands[i].device_id;
            bool found = false;
            for (int j = 0; j < active_count; j++) {
                if (active_devices[j] == dev_id) { found = true; break; }
            }
            if (!found) active_devices[active_count++] = dev_id;
        }
        
        // 添加 allow_rev 中的设备
        for (int i = 0; i < motor->allow_rev.count; i++) {
            uint8_t dev_id = motor->allow_rev.commands[i].device_id;
            bool found = false;
            for (int j = 0; j < active_count; j++) {
                if (active_devices[j] == dev_id) { found = true; break; }
            }
            if (!found) active_devices[active_count++] = dev_id;
        }
        
        // 添加 block_fwd 中的设备
        for (int i = 0; i < motor->block_fwd.count; i++) {
            uint8_t dev_id = motor->block_fwd.commands[i].device_id;
            bool found = false;
            for (int j = 0; j < active_count; j++) {
                if (active_devices[j] == dev_id) { found = true; break; }
            }
            if (!found) active_devices[active_count++] = dev_id;
        }
        
        // 添加 block_rev 中的设备
        for (int i = 0; i < motor->block_rev.count; i++) {
            uint8_t dev_id = motor->block_rev.commands[i].device_id;
            bool found = false;
            for (int j = 0; j < active_count; j++) {
                if (active_devices[j] == dev_id) { found = true; break; }
            }
            if (!found) active_devices[active_count++] = dev_id;
        }
        
        // 检查基因表中的所有设备
        MOTOR_OUT("  Registered but Inactive Devices:\r\n");
        bool has_inactive = false;
        
        // 定义所有可能的设备及其名称
        struct { uint8_t id; const char* name; } all_devices[] = {
            {DEV_ID_POWER_POS, "POWER_POS"},
            {DEV_ID_POWER_NEG, "POWER_NEG"},
            {DEV_ID_LIMIT_FWD, "LIMIT_FWD"},
            {DEV_ID_LIMIT_REV, "LIMIT_REV"},
            {DEV_ID_RTURN_FWD, "RTURN_FWD"},
            {DEV_ID_RTURN_REV, "RTURN_REV"},
            {DEV_ID_CAN, "CAN"},
            {DEV_ID_IO_FWD, "IO_FWD"},
            {DEV_ID_IO_REV, "IO_REV"},
            {DEV_ID_EMERGENCY, "EMERGENCY"}
        };
        
        for (int i = 0; i < sizeof(all_devices)/sizeof(all_devices[0]); i++) {
            bool is_active = false;
            for (int j = 0; j < active_count; j++) {
                if (active_devices[j] == all_devices[i].id) {
                    is_active = true;
                    break;
                }
            }
            if (!is_active) {
                MOTOR_OUT("    %s (ID=%d) - No command\r\n", 
                          all_devices[i].name, all_devices[i].id);
                has_inactive = true;
            }
        }
        
        if (!has_inactive) {
            MOTOR_OUT("    (All registered devices are active)\r\n");
        }
        
        // 打印冲突状态
        MOTOR_OUT("  Conflict=%d\r\n",
                  motor->debug_info.conflict_fault);
        
        // 打印当前活动设备
        const char* active_dev_name = "NONE";
        switch(motor->active_device_id) {
            case DEV_ID_IO_FWD: active_dev_name = "IO_FWD"; break;
            case DEV_ID_IO_REV: active_dev_name = "IO_REV"; break;
            case DEV_ID_CAN: active_dev_name = "CAN"; break;
            case DEV_ID_POWER_POS: active_dev_name = "POWER_POS"; break;
            case DEV_ID_POWER_NEG: active_dev_name = "POWER_NEG"; break;
            case DEV_ID_LIMIT_FWD: active_dev_name = "LIMIT_FWD"; break;
            case DEV_ID_LIMIT_REV: active_dev_name = "LIMIT_REV"; break;
            case DEV_ID_OVERCUR_FWD: active_dev_name = "OVERCUR_FWD"; break;
            default: active_dev_name = "NONE"; break;
        }
        MOTOR_OUT("  Active Device: %s (ID=%d)\r\n", active_dev_name, motor->active_device_id);
    }
    
    return RESULT_OK;
}

// ========== 电机特定接口实现 ==========

void Motor_SetSpeed(MotorDevice_t* motor, float duty) {
    if (!motor || !motor->enable) return;
    
    DeviceCommandData_t cmd;
    uint16_t duty_percent = (uint16_t)(duty * 100.0f);
    cmd.cmd = CMD_PWM_SET_DUTY;
    cmd.params = &duty_percent;
    cmd.param_size = sizeof(uint16_t);
    Device_Control(DEV_PWM_MOTOR_ID, &cmd);
}

void Motor_Start(MotorDevice_t* motor, MotorDir_t dir) {
    if (!motor || !motor->enable) return;
    
    // 通过EventBus发布启动事件，让仲裁层处理
    MotorManualIOEvent_t ev = {
        .dir = dir,
        .type = CMD_TYPE_RUN_FWD,
        .speed = 50.0f
    };
    
    if (dir == DIR_FWD) {
        ev.type = CMD_TYPE_RUN_FWD;
    } else if (dir == DIR_REV) {
        ev.type = CMD_TYPE_RUN_REV;
    }
    
    EventBus_Publish(TOPIC_MANUAL_IO, &ev);
}

void Motor_Stop(MotorDevice_t* motor) {
    if (!motor) return;
    
    MotorManualIOEvent_t ev = {
        .dir = DIR_NONE,
        .type = CMD_TYPE_STOP,
        .speed = 0
    };
    
    EventBus_Publish(TOPIC_MANUAL_IO, &ev);
}

void Motor_EmergencyStop(MotorDevice_t* motor) {
    if (!motor) return;
    
    MOTOR_DEBUG("EmergencyStop: clearing all allow commands\r\n");
    
    // 紧急停止：真正清空 allow_fwd 的 commands[] 数组
    for (int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        motor->allow_fwd.commands[i].device_id = DEV_ID_NONE;
        motor->allow_fwd.commands[i].priority = PRIO_NONE;
        motor->allow_fwd.commands[i].type = CMD_TYPE_NONE_USE;
        motor->allow_fwd.commands[i].param = 0.0f;
        motor->allow_fwd.commands[i].timestamp = 0;
    }
    motor->allow_fwd.count = 0;
    
    // 紧急停止：真正清空 allow_rev 的 commands[] 数组
    for (int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        motor->allow_rev.commands[i].device_id = DEV_ID_NONE;
        motor->allow_rev.commands[i].priority = PRIO_NONE;
        motor->allow_rev.commands[i].type = CMD_TYPE_NONE_USE;
        motor->allow_rev.commands[i].param = 0.0f;
        motor->allow_rev.commands[i].timestamp = 0;
    }
    motor->allow_rev.count = 0;
    
    // 注意：不清理 block_fwd / block_rev！
    
    // 更新调试变量
    g_u8MotorDebug_AllowFwdCount = 0;
    g_u8MotorDebug_AllowRevCount = 0;
    
    Motor_ArbitrationDecision(motor);
}

void Motor_ClearAllowFwd(MotorDevice_t* motor) {
    if (!motor) return;
    
    MOTOR_DEBUG("ClearAllowFwd: clearing all allow_fwd commands\r\n");
    
    // 真正清空 commands[] 数组
    for (int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        motor->allow_fwd.commands[i].device_id = DEV_ID_NONE;
        motor->allow_fwd.commands[i].priority = PRIO_NONE;
        motor->allow_fwd.commands[i].type = CMD_TYPE_NONE_USE;
        motor->allow_fwd.commands[i].param = 0.0f;
        motor->allow_fwd.commands[i].timestamp = 0;
    }
    motor->allow_fwd.count = 0;
    
    // 更新调试变量
    g_u8MotorDebug_AllowFwdCount = 0;
    
    // 重新仲裁
    Motor_ArbitrationDecision(motor);
}

void Motor_ClearAllowRev(MotorDevice_t* motor) {
    if (!motor) return;
    
    MOTOR_DEBUG("ClearAllowRev: clearing all allow_rev commands\r\n");
    
    // 真正清空 commands[] 数组
    for (int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        motor->allow_rev.commands[i].device_id = DEV_ID_NONE;
        motor->allow_rev.commands[i].priority = PRIO_NONE;
        motor->allow_rev.commands[i].type = CMD_TYPE_NONE_USE;
        motor->allow_rev.commands[i].param = 0.0f;
        motor->allow_rev.commands[i].timestamp = 0;
    }
    motor->allow_rev.count = 0;
    
    // 更新调试变量
    g_u8MotorDebug_AllowRevCount = 0;
    
    // 重新仲裁
    Motor_ArbitrationDecision(motor);
}

const MotorDebugInfo_t* Motor_GetDebugInfo(MotorDevice_t* motor) {
    if (!motor) return NULL;
    return &motor->debug_info;
}

MotorDir_t Motor_GetDesiredDirection(MotorDevice_t* motor) {
    if (!motor) return DIR_NONE;
    // 如果当前有正在执行的命令，返回其方向
    if (motor->state == MS_RUNNING || motor->state == MS_RAMPING) {
        return motor->active_dir;
    }
    return DIR_NONE;
}

// ========== 过流告警手动清除接口 ==========
void Motor_ClearOvercurrentBlock(MotorDevice_t* motor) {
    if (!motor) return;
    
    MOTOR_DEBUG("ClearOvercurrentBlock: removing block_fwd for DEV_ID_OVERCUR_FWD\r\n");
    
    Motor_CmdList_Remove(&motor->block_fwd, DEV_ID_OVERCUR_FWD);
    
    // 重新仲裁
    Motor_ArbitrationDecision(motor);
}

// ========== 电压告警手动清除接口 ==========
void Motor_ClearVoltageBlock(MotorDevice_t* motor, uint8_t u8AlarmType) {
    if (!motor) return;
    
    MotorDeviceId_t dev_id_fwd, dev_id_rev;
    const char* alarm_name;
    
    if (u8AlarmType == VOLTAGE_ALARM_OVERVOLTAGE) {
        dev_id_fwd = DEV_ID_OVERVOLTAGE_FWD;
        dev_id_rev = DEV_ID_OVERVOLTAGE_REV;
        alarm_name = "OVERVOLTAGE";
    } else if (u8AlarmType == VOLTAGE_ALARM_UNDERVOLTAGE) {
        dev_id_fwd = DEV_ID_UNDERVOLTAGE_FWD;
        dev_id_rev = DEV_ID_UNDERVOLTAGE_REV;
        alarm_name = "UNDERVOLTAGE";
    } else {
        return;
    }
    
    MOTOR_DEBUG("ClearVoltageBlock: %s - removing block_fwd/rev\r\n", alarm_name);
    
    Motor_CmdList_Remove(&motor->block_fwd, dev_id_fwd);
    Motor_CmdList_Remove(&motor->block_rev, dev_id_rev);
    
    // 重新仲裁
    Motor_ArbitrationDecision(motor);
}
