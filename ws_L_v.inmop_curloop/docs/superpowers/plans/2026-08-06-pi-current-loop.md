# PI 电流环 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有有感霍尔六步换相基础上增加 10kHz 电流 PI 内环（独立模式 mode 10），并小步增强 `dev_pid`（各环节输出可观测、I 项独立限幅、µs dt），不改变现有 mode 0~9 行为。

**Architecture:** 电流环挂载到现有 ADC1 EOCB ISR（100kHz）回调，按 10 拍抽取成 10kHz 控制率；反馈相按 `g_scope_step` 查 `dev_commutation` 固定状态表取导通相；dt 用 Timer6 µs 时间戳实测；PI 输出经新增 `Commutation_SetActiveDuty()` 直接写当前高边通道 OCCR（影子寄存器已配 PEAK 生效）。

**Tech Stack:** C (Keil MDK / ARM Compiler)，HC32F460 DDL，HC32F460 (Cortex-M4F, FPU)。

**测试约束：** 本工程无 CLI 构建（唯一构建定义是 Keil .uvprojx）。每个任务用"读回代码 + grep 符号 + 逻辑走查"验证，最终由用户在 Keil 编译烧录验证。任务里写明了每个文件、每段代码和验证命令。

---

## Task 1: dev_pid 增强（p/i/d_term、i_term_max、PID_UpdateUs）

**Files:**
- Modify: `D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\Utils\dev_pid.h`
- Modify: `D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\Utils\dev_pid.c`

- [ ] **Step 1: dev_pid.h — pid_config_t 增加 i_term_max**

在 `pid_config_t` 的 `integral_max` 之后加一行：

```c
    volatile float      integral_max;  /* Anti-windup clamp */
    volatile float      i_term_max;    /* NEW: clamp of I-term OUTPUT (0 = disabled) */
    volatile uint32_t   update_ms;     /* Minimum update interval (ms) */
```

- [ ] **Step 2: dev_pid.h — pid_state_t 增加字段 + PID_UpdateUs 声明**

`pid_state_t` 增加 4 个字段（`integral` 之后），并在 API 区加原型：

```c
typedef struct {
    pid_config_t *cfg;
    float         integral;
    float         prev_measurement;
    float         last_output;
    bool          first_sample;
    uint32_t      last_update_ms;
    /* ---- NEW ---- */
    float         p_term;          /* last P-term output (read-only observability) */
    float         i_term;          /* last I-term output (read-only observability) */
    float         d_term;          /* last D-term output (read-only observability) */
    uint32_t      last_update_us;  /* accumulated us since last executed update (PID_UpdateUs) */
} pid_state_t;
```

```c
/* Run one PID iteration with explicit us dt (for fast loops, e.g. current loop).
 * dt_us = measured time since previous call (clamped internally to 1s). */
float PID_UpdateUs(pid_state_t *pid, float setpoint, float measurement, uint32_t dt_us);
```

- [ ] **Step 3: dev_pid.c — 抽出公共计算核心 pid_compute**

在 `PID_Update` 之前新增静态函数（含 i_term_max 逻辑与 p/i/d_term 记录）：

```c
/* Core: one PID iteration with already-computed dt. Assumes cfg valid + enabled. */
static float pid_compute(pid_state_t *pid, float setpoint, float measurement, float dt_s)
{
    pid_config_t *cfg = pid->cfg;

    bool     p_valid      = cfg->p_valid;
    bool     i_valid      = cfg->i_valid;
    bool     d_valid      = cfg->d_valid;
    float    kp           = cfg->kp;
    float    ki           = cfg->ki;
    float    kd           = cfg->kd;
    float    output_min   = cfg->output_min;
    float    output_max   = cfg->output_max;
    float    integral_max = cfg->integral_max;
    float    i_term_max   = cfg->i_term_max;

    float error = setpoint - measurement;

    float p_term = 0.0f;
    if (p_valid) {
        p_term = kp * error;
    }

    float i_term = 0.0f;
    if (i_valid) {
        pid->integral += error * dt_s;
        if (pid->integral >  integral_max) pid->integral =  integral_max;
        if (pid->integral < -integral_max) pid->integral = -integral_max;
        i_term = ki * pid->integral;
        if (i_term_max > 0.0f) {
            if (i_term >  i_term_max) i_term =  i_term_max;
            if (i_term < -i_term_max) i_term = -i_term_max;
        }
    }

    float d_term = 0.0f;
    if (d_valid && !pid->first_sample && dt_s > 1e-6f) {
        d_term = kd * (pid->prev_measurement - measurement) / dt_s;
    }

    float output = p_term + i_term + d_term;
    if (output > output_max) output = output_max;
    if (output < output_min) output = output_min;

    pid->prev_measurement  = measurement;
    pid->first_sample      = false;
    pid->last_output       = output;
    pid->p_term            = p_term;
    pid->i_term            = i_term;
    pid->d_term            = d_term;
    return output;
}
```

- [ ] **Step 4: dev_pid.c — PID_Update 改为调用 pid_compute（行为不变）**

替换原 `PID_Update` 函数体为：

```c
float PID_Update(pid_state_t *pid, float setpoint, float measurement)
{
    if (!pid || !pid->cfg || !pid->cfg->enabled) {
        return 0.0f;
    }

    uint32_t update_ms = pid->cfg->update_ms;
    uint32_t now = (uint32_t)tickTimer_GetCount();

    /* --- Throttle check --- */
    if (update_ms > 0u && pid->last_update_ms != 0u) {
        uint32_t dt_ms = now - pid->last_update_ms;
        if (dt_ms < update_ms) {
            return pid->last_output;
        }
    }

    /* --- dt in seconds (clamped) --- */
    uint32_t dt_ms;
    if (pid->last_update_ms == 0u) {
        dt_ms = update_ms;
    } else {
        dt_ms = now - pid->last_update_ms;
    }
    if (dt_ms > 1000u) dt_ms = 1000u;
    float dt_s = (float)dt_ms / 1000.0f;

    float output = pid_compute(pid, setpoint, measurement, dt_s);
    pid->last_update_ms = now;
    return output;
}
```

- [ ] **Step 5: dev_pid.c — 新增 PID_UpdateUs**

在 `PID_Update` 之后新增：

```c
float PID_UpdateUs(pid_state_t *pid, float setpoint, float measurement, uint32_t dt_us)
{
    if (!pid || !pid->cfg || !pid->cfg->enabled) {
        return 0.0f;
    }

    /* --- Throttle (accumulated us). First call (first_sample) always executes. --- */
    uint32_t update_ms = pid->cfg->update_ms;
    if (update_ms > 0u && !pid->first_sample) {
        pid->last_update_us += dt_us;
        if (pid->last_update_us < update_ms * 1000u) {
            return pid->last_output;
        }
        pid->last_update_us = 0u;
    }

    /* --- dt clamp: max 1s (protects after debugger halt) --- */
    if (dt_us > 1000000u) dt_us = 1000000u;
    float dt_s = (float)dt_us / 1000000.0f;

    return pid_compute(pid, setpoint, measurement, dt_s);
}
```

- [ ] **Step 6: dev_pid.c — PID_Init / PID_Reset 清新增字段**

`PID_Init` 与 `PID_Reset` 的 `memset(pid, 0, sizeof(*pid))` 已自动清零新字段，无需额外代码；确认 `PID_Reset` 在重置 `last_output` 时同时保留 `first_sample=true`（现有代码已如此，无需改）。

- [ ] **Step 7: 验证 + 提交**

验证（读回关键符号）：

```powershell
Select-String -Path "D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\Utils\dev_pid.h" -Pattern "i_term_max","p_term","PID_UpdateUs"
Select-String -Path "D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\Utils\dev_pid.c" -Pattern "pid_compute","PID_UpdateUs"
```

提交：

```bash
git add "ws_L_v.1.0.2/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/Utils/dev_pid.h" "ws_L_v.1.0.2/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/Utils/dev_pid.c"
git commit -m "feat(pid): add per-term outputs, i_term_max, PID_UpdateUs(us dt)"
```

---

## Task 2: Commutation_GetPwmChannel + Commutation_SetActiveDuty

**Files:**
- Modify: `D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\dev_commutation.h`
- Modify: `D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\dev_commutation.c`

- [ ] **Step 1: dev_commutation.h — 新增两个原型**

在 `Commutation_GetFieldAngle` 声明之后加：

```c
/* Return the channel (0=U,1=V,2=W) that carries PWM duty in this step; 0xFF if none */
uint8_t Commutation_GetPwmChannel(uint8_t step);

/* Mid-step duty update: write OCCR of the active PWM (high-side) channel for this step.
 * Clamps 2%~98%, keeps per-channel cache consistent. Safe to call from ISR. */
void Commutation_SetActiveDuty(uint8_t step, float duty_pct);
```

- [ ] **Step 2: dev_commutation.c — 实现两个函数**

在 `Commutation_Stop` 之前新增：

```c
uint8_t Commutation_GetPwmChannel(uint8_t step)
{
    if (step > 5U) {
        return 0xFFu;
    }
    for (int ch = 0; ch < 3; ch++) {
        if (s_states[step][ch * 2U] == M_HIGH) {
            return (uint8_t)ch;
        }
    }
    return 0xFFu;
}

void Commutation_SetActiveDuty(uint8_t step, float duty_pct)
{
    uint8_t ch;

    if (step > 5U) {
        return;
    }

    if (duty_pct < COMM_DUTY_MIN_F) {
        duty_pct = COMM_DUTY_MIN_F;
    }
    if (duty_pct > COMM_DUTY_MAX_F) {
        duty_pct = COMM_DUTY_MAX_F;
    }

    ch = Commutation_GetPwmChannel(step);
    if (ch == 0xFFu) {
        return;
    }

    TMR4_PWM_SetDutyFloat((tmr4_pwm_channel_t)ch, duty_pct);
    s_last_ch_duty[ch] = duty_pct;   /* keep lazy-update cache consistent */
}
```

- [ ] **Step 3: 验证 + 提交**

```powershell
Select-String -Path "D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\dev_commutation.h" -Pattern "Commutation_GetPwmChannel","Commutation_SetActiveDuty"
Select-String -Path "D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\dev_commutation.c" -Pattern "s_last_ch_duty\[ch\] = duty_pct"
```

```bash
git add "ws_L_v.1.0.2/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/dev_commutation.h" "ws_L_v.1.0.2/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/dev_commutation.c"
git commit -m "feat(commutation): add active-channel duty update API (mid-step, ISR-safe)"
```

---

## Task 3: 电流环模块 ws/cur_loop.h/.c

**Files:**
- Create: `D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\cur_loop.h`
- Create: `D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\cur_loop.c`

- [ ] **Step 1: 创建 cur_loop.h**

```c
#ifndef __CUR_LOOP_H__
#define __CUR_LOOP_H__

#include "dev_pid.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Keil Watch: current setpoint (mA, signed) */
extern volatile float g_i_ref_ma;

/* J-Scope observability (updated in ADC ISR at control rate) */
extern volatile float g_scope_i_ref;
extern volatile float g_scope_i_fb;
extern volatile float g_scope_i_duty;
extern volatile float g_scope_i_err;

/* Current-loop PID config (volatile, Keil Watch tunable) */
extern pid_config_t g_cur_pid_cfg;

/* Register ADC EOCB callback. Call once after I_Init(). */
void CurLoop_Init(void);

void  CurLoop_SetRef(float ma);
float CurLoop_GetRef(void);

#ifdef __cplusplus
}
#endif

#endif /* __CUR_LOOP_H__ */
```

- [ ] **Step 2: 创建 cur_loop.c**

```c
/**
 *******************************************************************************
 * @file  cur_loop.c
 * @brief 10kHz current PI loop for six-step BLDC (learning)
 *
 *        Mounted on ADC1 EOCB ISR (100kHz) via I_RegisterCallback, decimated
 *        by CURLOOP_DECIMATION (10 -> 10kHz control rate).
 *        Feedback: active high-side phase current (from fixed state table,
 *        selected by g_scope_step), window-averaged over the decimation window.
 *        dt: measured from Timer6 microsecond timestamp.
 *        Output: Commutation_SetActiveDuty() -> OCCR (takes effect at next PWM peak).
 *
 *        Active only in COMM_RUNNER_CURLOOP_FW mode while hall FSM is RUNNING.
 *******************************************************************************
 */

#include "cur_loop.h"
#include "I.h"
#include "hall_sensor_3ch.h"
#include "dev_commutation.h"
#include "dev_comm_runner.h"
#include "timer6_timebase.h"

#define CURLOOP_DECIMATION   10u   /* 100kHz / 10 = 10kHz */
#define CURLOOP_DT_FIRST_US  100u  /* assumed 10kHz period for the very first call */

/* Keil Watch: current setpoint (mA) */
volatile float g_i_ref_ma = 500.0f;

/* J-Scope observability */
volatile float g_scope_i_ref  = 0.0f;
volatile float g_scope_i_fb   = 0.0f;
volatile float g_scope_i_duty = 0.0f;
volatile float g_scope_i_err  = 0.0f;

/* Current-loop PI config (Keil Watch tunable) */
pid_config_t g_cur_pid_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = 0.05f,    /* % duty per mA error */
    .ki           = 0.005f,   /* % duty per (mA*s) */
    .kd           = 0.0f,
    .output_min   = 2.0f,
    .output_max   = 98.0f,
    .integral_max = 50.0f,
    .i_term_max   = 10.0f,    /* I contribution clamped to +/-10% */
    .update_ms    = 0,        /* no throttle: run at every decimated call */
};

static pid_state_t s_pid;
static int32_t  s_sum_ma  = 0;
static uint8_t  s_cnt     = 0;
static uint64_t s_last_us = 0;
static uint8_t  s_inited  = 0;

static int16_t curloop_feedback(const stc_i_data_t *pData)
{
    uint8_t ch = Commutation_GetPwmChannel(g_scope_step);
    switch (ch) {
        case 0:  return pData->i16IU_mA;
        case 1:  return pData->i16IV_mA;
        case 2:  return pData->i16IW_mA;
        default: return 0;
    }
}

static void curloop_isr(const stc_i_data_t *pData)
{
    if (CommRunner_GetMode() != COMM_RUNNER_CURLOOP_FW ||
        !CommRunner_IsRunning()) {
        /* not active: reset window so restart starts clean */
        s_last_us = 0;
        s_cnt     = 0;
        s_sum_ma  = 0;
        return;
    }

    s_sum_ma += curloop_feedback(pData);
    s_cnt++;

    if (s_cnt >= CURLOOP_DECIMATION) {
        s_cnt = 0;
        float fb = (float)s_sum_ma / (float)CURLOOP_DECIMATION;
        s_sum_ma = 0;

        Timer6_Timebase_UpdateTimestamp();
        uint64_t now = Timer6_Timebase_GetTimestamp();
        uint32_t dt_us;
        if (s_last_us == 0) {
            dt_us = CURLOOP_DT_FIRST_US;
        } else {
            dt_us = (uint32_t)(now - s_last_us);
        }
        s_last_us = now;

        float duty = PID_UpdateUs(&s_pid, g_i_ref_ma, fb, dt_us);
        Commutation_SetActiveDuty(g_scope_step, duty);

        g_scope_i_ref  = g_i_ref_ma;
        g_scope_i_fb   = fb;
        g_scope_i_duty = duty;
        g_scope_i_err  = g_i_ref_ma - fb;
    }
}

void CurLoop_Init(void)
{
    if (s_inited) {
        return;
    }
    PID_Init(&s_pid, &g_cur_pid_cfg);
    I_RegisterCallback(curloop_isr);
    s_inited = 1;
}

void CurLoop_SetRef(float ma)
{
    g_i_ref_ma = ma;
}

float CurLoop_GetRef(void)
{
    return g_i_ref_ma;
}
```

- [ ] **Step 3: 验证 + 提交**

```powershell
Select-String -Path "D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\cur_loop.h" -Pattern "CurLoop_Init","g_i_ref_ma","g_scope_i_duty"
Select-String -Path "D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\cur_loop.c" -Pattern "PID_UpdateUs","Commutation_SetActiveDuty","I_RegisterCallback"
```

```bash
git add "ws_L_v.1.0.2/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/cur_loop.h" "ws_L_v.1.0.2/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/cur_loop.c"
git commit -m "feat(cur_loop): 10kHz current PI loop on EOCB ISR (decimated, Timer6 dt)"
```

---

## Task 4: runner mode 10 + main.c 集成

**Files:**
- Modify: `D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\dev_comm_runner.h`
- Modify: `D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\dev_comm_runner.c`
- Modify: `D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\template\source\main.c`

- [ ] **Step 1: dev_comm_runner.h — 增加 mode 10**

在 `COMM_RUNNER_PID_CCW = 9` 之后加：

```c
    COMM_RUNNER_CURLOOP_FW = 10, /* Fly-start -> Hall closed-loop + current PI (10kHz), CW */
```

- [ ] **Step 2: dev_comm_runner.c — SetMode 允许重启 + case 10**

`CommRunner_SetMode` 的"总是允许重启"判断里，在 `mode != COMM_RUNNER_PID_CCW &&` 之后加 `mode != COMM_RUNNER_CURLOOP_FW &&`。

在 switch 的 `case COMM_RUNNER_PID_CCW:` 分支之后加：

```c
    case COMM_RUNNER_CURLOOP_FW:
        if (s_hall) hall_3ch_stop(s_hall);
        calib_build_derived_tables();
        start_open_loop(s_cfg.ol_fly_start_us, s_cfg.ol_fly_target_us,
                        s_cfg.ol_fly_ramp_ms, 1);
        s_sub_phase = 0;
        MAIN_D("[CommRunner] Mode=CURLOOP_FW: fly-start -> closed-loop + current PI");
        break;
```

- [ ] **Step 3: dev_comm_runner.c — Update 增加 case 10**

在 `case COMM_RUNNER_PID_CCW:` 分支（含 PID）之后加：

```c
    /* ---- Current-loop (mode 10): current PI runs in ADC ISR (cur_loop.c) ---- */
    case COMM_RUNNER_CURLOOP_FW: {
        if (s_sub_phase == 0) {
            /* Phase 0: open-loop ramp */
            open_loop_tick(now, 1);

            uint64_t ramp_elapsed = now - s_ol_ramp_start_us;
            uint64_t ramp_total   = (uint64_t)s_ol_ramp_duration_ms * 1000UL;
            if (ramp_elapsed >= ramp_total) {
                hall_3ch_set_table(s_hall, g_calib_cw_table);
                hall_3ch_start_flying(s_hall, HALL3_DIR_FORWARD);
                s_sub_phase = 1;
                MAIN_D("[CommRunner] CURLOOP fly-start -> closed-loop (current PI active)");
            }
        } else {
            /* Phase 1: closed-loop (Hall ISR driven); duty handled by cur_loop ISR */
            hall_3ch_update(s_hall);
            if (hall_3ch_is_stalled(s_hall)) {
                MAIN_D("[CommRunner] CURLOOP stall, coast");
                CommRunner_SetMode(COMM_RUNNER_STOP);
            }
        }
        break;
    }
```

- [ ] **Step 4: main.c — include + CurLoop_Init + 注释**

`#include "I.h"` 之后加 `#include "cur_loop.h"`。

在 `I_Calibrate();` 之后加：

```c
    /* ---- 电流环初始化 (挂到 ADC1 EOCB ISR, 10kHz 抽取) ---- */
    CurLoop_Init();
```

`comm_mode` 的注释追加 mode 10：

```c
volatile int   comm_mode        = 0;     /* 0=Stop ... 9=PID_CCW 10=CurLoopFW */
```

- [ ] **Step 5: 验证 + 提交**

```powershell
Select-String -Path "D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\dev_comm_runner.h" -Pattern "CURLOOP_FW"
Select-String -Path "D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\template\source\main.c" -Pattern "cur_loop.h","CurLoop_Init"
```

```bash
git add "ws_L_v.1.0.2/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/dev_comm_runner.h" "ws_L_v.1.0.2/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/dev_comm_runner.c" "ws_L_v.1.0.2/HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/source/main.c"
git commit -m "feat(runner): add mode 10 CURLOOP_FW (current PI integration)"
```

---

## Task 5: 收尾验证 + 文档

- [ ] **Step 1: 全量 grep 一致性检查**

```powershell
Select-String -Path "D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\*.c","D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\*.h" -Pattern "CURLOOP_FW","Commutation_SetActiveDuty","Commutation_GetPwmChannel","PID_UpdateUs","g_i_ref_ma"
```

- [ ] **Step 2: 更新 CLAUDE.md 模式列表（mode 0~9 → 0~10）**

`CLAUDE.md` 的 `comm_mode` 行与模式描述处补 mode 10 说明。

- [ ] **Step 3: 提交 + 请用户在 Keil 编译验证**

```bash
git add "ws_L_v.1.0.2/CLAUDE.md"
git commit -m "docs: document mode 10 CURLOOP_FW in CLAUDE.md"
```

> ⚠️ 本工程无 CLI 构建；用户需在 Keil 打开 `template/MDK/template.uvprojx` 编译，确认 mode 0~9 回归 + mode 10 电流环行为。