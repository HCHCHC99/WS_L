#ifndef __DEV_COMM_RUNNER_H__
#define __DEV_COMM_RUNNER_H__

#include "hall_sensor_3ch.h"
#include <stdint.h>

/*=============================================================================
 * CommRunner: �步方波换相控制器
 *
 * 封�换相状态机（停� / �� / 飞启→闭�），替代 main.c �的分散�辑�
 * 主循���调用 CommRunner_Update，模式切换调� CommRunner_SetMode�
 *=============================================================================*/

/* 换相模式 (与原� comm_mode 0~4 兼�) */
typedef enum {
    COMM_RUNNER_STOP       = 0,  /* �性滑� / 停� */
    COMM_RUNNER_OPEN_FW    = 1,  /* ��正转 (恒�斜�) */
    COMM_RUNNER_OPEN_RV    = 2,  /* ��反转 (恒�斜�) */
    COMM_RUNNER_CLOSED_FW  = 3,  /* 飞启→闭�正转 */
    COMM_RUNNER_CLOSED_RV  = 4,  /* 飞启→闭�反转 */
    COMM_RUNNER_CALIB      = 5,  /* Open-loop calibration: auto-derive Hall-to-Step 0deg table */
    COMM_RUNNER_CALIB_CW   = 6,  /* 500ms open-loop -> closed-loop CW  using calib table +5 */
    COMM_RUNNER_CALIB_CCW  = 7,  /* 500ms open-loop -> closed-loop CCW using calib table +2 */
} comm_runner_mode_t;

/*=============================================================================
 * Calibration types (visible in Keil Watch debugger)
 *=============================================================================*/

/* Calibration status */
typedef enum {
    CALIB_IDLE           = 0,  /* Not running */
    CALIB_RUNNING        = 1,  /* Sampling in progress */
    CALIB_SUCCESS        = 2,  /* Calibration complete, g_calib_table valid */
    CALIB_FAIL_TIMEOUT   = 3,  /* Did not complete in time */
    CALIB_FAIL_STALL     = 4,  /* Hall states stopped changing (motor not spinning) */
    CALIB_FAIL_MISSING   = 5,  /* Some Hall state never observed; check g_calib_error */
    CALIB_FAIL_DUPLICATE = 6,  /* Hall state mapped to multiple steps; check g_calib_error */
    CALIB_FAIL_INVALID   = 7,  /* Detected invalid Hall code 0x00 or 0x07 */
    CALIB_FAIL_AMBIGUOUS = 8,  /* Majority vote below threshold in some step */
} calib_status_t;

/* Error detail for calibration failures */
typedef struct {
    uint8_t hall_state;    /* Problematic Hall code (1-6, or 0/7 for invalid) */
    uint8_t step_a;        /* First step this Hall state mapped to */
    uint8_t step_b;        /* Conflicting step (for duplicate), 0xFF if N/A */
    uint8_t reserved;
} calib_error_detail_t;

/*=============================================================================
 * Calibration globals (volatile for Keil Watch visibility)
 *=============================================================================*/
extern volatile calib_status_t      g_calib_status;
extern volatile calib_error_detail_t g_calib_error;
extern volatile uint8_t  g_calib_table[8];             /* Hall_state -> step (0-offset); entries 0/7 = 0xFF */
extern uint8_t  g_calib_cw_table[8];          /* Calib-derived CW table: = (g_calib_table + 5) % 6 (mode 6) */
extern uint8_t  g_calib_ccw_table[8];         /* Calib-derived CCW table: = (g_calib_table + 2) % 6 (mode 7) */
extern volatile uint8_t  g_calib_valid_cycles;         /* Completed valid electrical cycles */
extern volatile uint8_t  g_calib_total_steps;          /* Total step transitions counted */
extern volatile uint8_t  g_calib_cycle_confidence[6];  /* Majority %% per step in latest cycle */
extern volatile uint8_t  g_calib_hall_obs[8];          /* Total observations per Hall code (all cycles) */
extern volatile uint8_t  g_calib_hall_seen[8];         /* 0=never_seen, 1=seen_at_least_once across cycles */
extern volatile uint8_t  g_calib_hall_first_step[8];   /* First step this Hall state was observed on (0-5, 0xFF=N/A) */
extern volatile uint8_t  g_calib_hall_first_cycle[8];  /* First cycle this Hall state was observed in (1-based, 0=N/A) */
extern volatile uint8_t  g_calib_cycle_rejected;       /* Count of cycles that failed bijection */
extern volatile uint8_t  g_calib_cycle_total;          /* Total cycles attempted (passed + rejected) */

/*=============================================================================
 * Calibration API
 *=============================================================================*/
calib_status_t CommRunner_GetCalibStatus(void);
const uint8_t* CommRunner_GetCalibTable(void);
void           CommRunner_CalibAbort(void);

/* 配置结构 */
typedef struct {
    /* PWM 频率 */
    uint16_t pwm_freq_hz;

    /* Hall 传感器配� (传给 hall_3ch_create) */
    hall_3ch_config_t hall_cfg;

    /* ��恒� (mode 1/2): 斜坡参数 */
    uint32_t ol_const_start_us;     /* 起�间� (us) */
    uint32_t ol_const_target_us;    /* �标间� (us) */
    uint32_t ol_const_ramp_ms;      /* 斜坡时长 (ms) */

    /* 飞启�� (mode 3/4): 斜坡参数 */
    uint32_t ol_fly_start_us;       /* 起�间� (us, 高扭矩低�) */
    uint32_t ol_fly_target_us;      /* �标间� (us, 高�) */
    uint32_t ol_fly_ramp_ms;        /* 斜坡时长 (ms) */

    /* 占空比默认� */
    float    default_duty_pct;

    /* PWM 初�化后回� (��): � main.c 做�� GPIO 设置 */
    void (*on_init_done)(void);
} comm_runner_config_t;

/*=============================================================================
 * API
 *=============================================================================*/

/* 初�化: 创建 PWM / Hall / 时间基准, 进入 STOP 状� */
void CommRunner_Init(const comm_runner_config_t *cfg);

/* 切换模式: 内部处理�有过� (停� / ��起� / 飞启) */
void CommRunner_SetMode(comm_runner_mode_t mode);

/* 获取当前模式 */
comm_runner_mode_t CommRunner_GetMode(void);

/* 动��置占空� (��模式下实时生�) */
void CommRunner_SetDuty(float duty_pct);

/* 获取当前占空� */
float CommRunner_GetDuty(void);

/* 主循�调用: 1ms 周期, 驱动��换相 + ��监控 */
void CommRunner_Update(void);

/* 获取滤波� RPM (来自 Hall 传感� M �) */
float CommRunner_GetRPM(void);

/* �否�在运� (��模式� Hall 处于 RUNNING 状�) */
uint8_t CommRunner_IsRunning(void);

/* �否堵� */
uint8_t CommRunner_IsStalled(void);

#endif /* __DEV_COMM_RUNNER_H__ */
