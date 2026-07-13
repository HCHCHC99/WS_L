
  #include "main.h"
  #include "Hardware.h"
  #include "rtt_log.h"
  #include "timer6_timebase.h"
  #include "TickTimer.h"
  #include "App_Motor_Project.h"
  #include "App_Comm.h"
  #include "App_FaultHandler.h"
  #include "rtt_manager.h"
  #include "hc32_ll_utility.h"
  #include "tmr4_pwm.h"
#include "dev_comm_runner.h"
#include "Bemf.h"
#include "I.h"
#include "Usart3_Vofa.h"
#include "Usart3_Vofa_Runner.h"
#include "test_Vofa.h"
#include "../Utils/dev_pid.h"

/* Official Vofa handle (for Vofa_JustFloat / Vofa_Printf etc.) */
Vofa_HandleTypedef vofa1;

/*=============================================================================
 * Keil Watch ��改变�? (调试接口)
 *=============================================================================*/
volatile int   comm_mode        = 0;     /* 0=Stop 1=OpenFW 2=OpenRV 3=ClosedFW 4=ClosedRV 5=Calibrate 6=CalibCW 7=CalibCCW */
volatile float g_comm_duty_pct  = 80.0f; /* Duty cycle 2%~98% */

/* PID speed control �? Keil Watch variables */
volatile float g_target_rpm       = 3000.0f;

/* PID config �? all fields volatile, Keil Watch can modify at runtime */
pid_config_t g_pid_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = 0.02f,
    .ki           = 0.005f,
    .kd           = 0.0f,
    .output_min   = 2.0f,
    .output_max   = 98.0f,
    .integral_max = 50.0f,
    .update_ms    = 50,
};

/* �? PWM 全局变量 (dev_motor 模块引用, 不可删除) */
pwm_t g_motor_pwm_ch1;
pwm_t g_motor_pwm_ch2;
pwm_t g_motor_pwm_ch3;
pwm_t g_motor_pwm_ch4;

/*=============================================================================
 * Hall 映射�? (16�? × 8�?)
 *   0~5: 同� (霍尔+1=磁场CW)
 *   6~11: 偏移 (霍尔+1=磁场CCW)
 *   12: 实测校准 CW
 *   13: CCW (磁场CCW超前)
 *   14: 强拖正转 [sector+90°]
 *   15: 强拖反转 [sector-90°]
 *=============================================================================*/
int main(void)
{
    /* ---- �件初始�? ---- */
    Hardware_Init();

#if 0  /* RS485 Modbus disabled, PB12/PB13 now used for USART3 */
    /* ---- 通信�? (RS485 + Modbus RTU) ---- */
    static const App_Comm_Config_t comm_cfg = {
        .phy.baudrate     = 9600,
        .phy.dir_polarity = 0,
        .hal.rx_buf_size  = 500,
        .hal.tx_buf_size  = 500,
        .hal.rx_frame_queue_depth = 10,
        .hal.tx_queue_depth       = 10,
        .hal.frame_timeout_ms     = 0,
        .proto.node_id            = 1,
        .proto.enable_write_multi = true,
    };
    App_Comm_Init(&comm_cfg);
#endif  /* RS485 Modbus disabled */
    /* ---- USART3 + VOFA+ 初始化 ---- */
    Usart3_Vofa_Init(NULL);
    Usart3_Vofa_Runner_Init();
    Vofa_Init(&vofa1, VOFA_MODE_SKIP);

#if VOFA_TEST_ENABLE
    TestVofa_Init();
#endif

    tickTimer_DelayMs(5);

    /* ---- 换相控制器初始化 ---- */
    static const comm_runner_config_t runner_cfg = {
        .pwm_freq_hz       = 50000,

        /* Hall 传感器配�?: 3�?, PA10=U, PA9=V, PA8=W, 3对极 */
        .hall_cfg = {
            .port      = {GPIO_PORT_A, GPIO_PORT_A, GPIO_PORT_A},
            .pin       = {GPIO_PIN_10, GPIO_PIN_09, GPIO_PIN_08},
            .eirq_ch   = {EXTINT_CH10, EXTINT_CH09, EXTINT_CH08},
            .irqn      = {INT010_IRQn, INT009_IRQn, INT008_IRQn},
            .irq_src   = {INT_SRC_PORT_EIRQ10, INT_SRC_PORT_EIRQ9, INT_SRC_PORT_EIRQ8},
            .irq_priority = DDL_IRQ_PRIO_02,
            .pole_pairs   = 3,
            /* 默��场对齐�?: step0�?0x01, 磁场正向 */
            .hall_to_step = {0xFF,1,3,2,5,0,4,0xFF},
            /* on_step/on_fault �? CommRunner 内部覆写 */
            .on_step      = NULL,
            .on_fault     = NULL,
            .align_step        = 0,
            .align_duty_pct    = 80.0f,
            .align_duration_ms = 500,
            .stall_timeout_ms  = 500,
        },

        /* ��恒� (mode 1/2): 667 RPM 定� 3s 斜坡 */
        .ol_const_start_us  = 5000,
        .ol_const_target_us = 5000,
        .ol_const_ramp_ms   = 3000,

        /* 飞启�� (mode 3/4): 167�?1111 RPM, 2s 斜坡 */
        .ol_fly_start_us    = 20000,
        .ol_fly_target_us   = 3000,
        .ol_fly_ramp_ms     = 2000,

        .default_duty_pct   = 80.0f,
        .on_init_done       = NULL,
        .pid_cfg           = &g_pid_cfg,
    };
    CommRunner_Init(&runner_cfg);

    /* ---- BEMF 初始�? (PWM 已启�?, TMR4_3 正在运行) ---- */
    Bemf_Init();

    /* ---- 电流采样初始�? (ADC1_SEQ_B, PWM peak 触发, 50kHz) ---- */
    I_Init();

    /* ---- 电流零偏校准 (阻塞500ms, 电机必须静止) ---- */
    I_Calibrate();

    EventBus_Enable();

    /* ---- 主循环 ---- */
    static int   s_prev_mode     = -1;
    static float s_prev_duty     = 80.0f;

    while (1) {
//         App_Comm_Poll();

        /* Keil Watch �? CommRunner (调试�?/Modbus 下发的模式切�?) */
        if (comm_mode != s_prev_mode) {
            s_prev_mode = comm_mode;
            CommRunner_SetMode((comm_runner_mode_t)comm_mode);
        }
        if (g_comm_duty_pct != s_prev_duty) {
            s_prev_duty = g_comm_duty_pct;
            CommRunner_SetDuty(g_comm_duty_pct);
        }

        /* 驱动换相状��? */
        CommRunner_Update();

        /* CommRunner �? Keil Watch (堵转等内部触发的 STOP 同�回�) */
        {
            int actual = (int)CommRunner_GetMode();
            if (actual != comm_mode) {
                comm_mode   = actual;
                s_prev_mode = actual;
            }
        }

        /* ---- BEMF 数据读取 (�?500ms打印一次观察数�?) ---- */
#ifdef BEMF_PERIODIC_DBG
        {
            static uint32_t s_u32LastBemfPrintMs = 0;
            uint32_t u32Now = tickTimer_GetCount();
            if ((u32Now - s_u32LastBemfPrintMs) >= 500) {
                s_u32LastBemfPrintMs = u32Now;
                if (g_bemf_running) {
                    MAIN_D("BEMF: M=%umV U=%dmV V=%dmV W=%dmV cnt=%lu\r\n",
                           g_bemf_m_mv, g_bemf_u_mv, g_bemf_v_mv, g_bemf_w_mv,
                           g_bemf_sample_cnt);
                }
            }
        }
#endif

        /* ---- Target RPM: Keil Watch -> PID ---- */
        {
            static float s_last_target = 0.0f;
            if (g_target_rpm != s_last_target) {
                s_last_target = g_target_rpm;
                CommRunner_SetTargetRPM(g_target_rpm);
            }
        }

        /* ---- VOFA+ USART3: 心跳 + RX 日志 + 测试信号 ---- */
        Usart3_Vofa_FeedRx(&vofa1);   /* ring_buf -> Vofa FIFO (official API) */
        Usart3_Vofa_Runner_Run();     /* heartbeat + RX MAIN_D logging */

#if VOFA_TEST_ENABLE
        TestVofa_Run();               /* CH1 sawtooth + CH2 counter */
#endif
    }
}
