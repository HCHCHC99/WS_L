# 环拓扑架构（速度/电流/位置预留）Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用配置宏 + 链路接线层，让"电流环 only / 速度环 only / 速度+电流级联"可自由切换，并预留位置环接口。每个环独立模块、输入源/输出目标可配。

**Architecture:** 四级链路 `位置环(预留) → 速度环 → 电流环 → 占空比`。速度环在主循环（慢），电流环在 EOCB ISR（快）。拓扑由 `ws/motor_config.h` 的宏决定，`dev_comm_runner` 只做接线。

**Tech Stack:** C (Keil MDK)，HC32F460，dev_pid（通用 PID）、cur_loop（电流环已模块化）。

**测试约束：** 无 CLI 构建。每任务用"读回 + grep + 逻辑走查"验证；最终由用户在 Keil 编译并跑 mode 10（电流 only）、mode 8（速度 only）、mode 11（级联）验证。

---

## Task 1: motor_config.h 增加环拓扑宏

**Files:** Modify: `D:\WS_L\ws_L_v.1.0.2\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\motor_config.h`

- [ ] **Step 1: 追加环拓扑宏（在 MOTOR_PWM_FREQ_HZ 之后）**

```c
/* ============================================================================
 * 环拓扑配置（改这里切换控制结构）
 * ==========================================================================*/
/* 位置环（预留，默认关 - 需位置反馈如编码器） */
#define MOTOR_LOOP_POSITION_ENABLE   0
#define POS_REF_EXTERNAL             0
#define MOTOR_POS_REF_SRC            POS_REF_EXTERNAL

/* 速度环 */
#define MOTOR_LOOP_SPEED_ENABLE      1
#define SPD_REF_TARGET_RPM           0
#define SPD_REF_FROM_POS             1
#define MOTOR_SPD_REF_SRC            SPD_REF_TARGET_RPM

/* 电流环 */
#define MOTOR_LOOP_CURRENT_ENABLE    1
#define CUR_REF_FIXED                0
#define CUR_REF_FROM_SPEED           1
#define MOTOR_CUR_REF_SRC            CUR_REF_FROM_SPEED

/* 占空比来源 */
#define DUTY_DIRECT                  0
#define DUTY_FROM_CURRENT            1
#define MOTOR_DUTY_SRC               DUTY_FROM_CURRENT
```

- [ ] **Step 2: 验证** `Select-String motor_config.h -Pattern "MOTOR_LOOP_SPEED_ENABLE","MOTOR_CUR_REF_SRC"`
- [ ] **Step 3: 提交** `git commit -m "feat(config): add loop topology macros (speed/current/position reserved)"`

---

## Task 2: 新建 ws/speed_loop.h/.c（速度环独立模块）

**Files:**
- Create: `...\ws\speed_loop.h`
- Create: `...\ws\speed_loop.c`

- [ ] **Step 1: speed_loop.h**

```c
#ifndef __SPEED_LOOP_H__
#define __SPEED_LOOP_H__

#include "../Utils/dev_pid.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Keil Watch: current limit when current loop is enabled (mA) */
extern volatile float g_i_ref_max_ma;

/* Speed-loop PID config (Keil Watch tunable) */
extern pid_config_t g_spd_pid_cfg;

/* J-Scope observability */
extern volatile float g_scope_spd_ref;
extern volatile float g_scope_spd_rpm;
extern volatile float g_scope_spd_err;
extern volatile float g_scope_spd_out;

void  SpeedLoop_Init(void);
void  SpeedLoop_SetTarget(float target_rpm);
/* Run one speed-loop iteration (throttled internally by update_ms).
 * Returns output: current ref (mA) when current loop enabled, else duty (%). */
float SpeedLoop_Update(float measured_rpm);
float SpeedLoop_GetOutput(void);
/* Seed output for bumpless handoff (e.g. start from measured current). */
void  SpeedLoop_Seed(float output);

#ifdef __cplusplus
}
#endif

#endif /* __SPEED_LOOP_H__ */
```

- [ ] **Step 2: speed_loop.c**

```c
#include "speed_loop.h"
#include "motor_config.h"

/* Keil Watch: current limit for cascade (mA) */
volatile float g_i_ref_max_ma = 1500.0f;

/* Default gains = legacy single-loop speed values; for cascade raise Kp/Ki */
pid_config_t g_spd_pid_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = 0.02f,
    .ki           = 0.005f,
    .kd           = 0.0f,
    .output_min   = 2.0f,    /* adjusted by topology at init */
    .output_max   = 98.0f,
    .integral_max = 200.0f,
    .i_term_max   = 0.0f,
    .update_ms    = 50,
};

volatile float g_scope_spd_ref = 0.0f;
volatile float g_scope_spd_rpm = 0.0f;
volatile float g_scope_spd_err = 0.0f;
volatile float g_scope_spd_out = 0.0f;

static pid_state_t s_spd_pid;
static float s_target_rpm = 0.0f;
static float s_output     = 0.0f;
static uint8_t s_inited   = 0;

void SpeedLoop_Init(void)
{
    if (s_inited) return;

#if MOTOR_LOOP_CURRENT_ENABLE
    /* cascade: output = current reference (mA), 0 .. limit */
    g_spd_pid_cfg.output_min = 0.0f;
    g_spd_pid_cfg.output_max = g_i_ref_max_ma;
#else
    /* speed-only: output = duty (%) */
    g_spd_pid_cfg.output_min = 2.0f;
    g_spd_pid_cfg.output_max = 98.0f;
#endif

    PID_Init(&s_spd_pid, &g_spd_pid_cfg);
    s_inited = 1;
}

void SpeedLoop_SetTarget(float target_rpm) { s_target_rpm = target_rpm; }

float SpeedLoop_Update(float measured_rpm)
{
    if (!s_inited) SpeedLoop_Init();
    g_scope_spd_rpm = measured_rpm;
    s_output = PID_Update(&s_spd_pid, s_target_rpm, measured_rpm);
    g_scope_spd_ref = s_target_rpm;
    g_scope_spd_err = s_target_rpm - measured_rpm;
    g_scope_spd_out = s_output;
    return s_output;
}

float SpeedLoop_GetOutput(void) { return s_output; }

void SpeedLoop_Seed(float output)
{
    if (!s_inited) SpeedLoop_Init();
    PID_Reset(&s_spd_pid);
    s_spd_pid.last_output = output;
    s_output = output;
}
```

- [ ] **Step 3: 验证** grep 检查符号齐全、括号配平
- [ ] **Step 4: 提交** `git commit -m "feat(speed_loop): independent speed loop module (cascade-ready)"`

---

## Task 3: cur_loop 增加外部给定入口（级联用）

**Files:** Modify: `...\ws\cur_loop.h`, `...\ws\cur_loop.c`

- [ ] **Step 1: cur_loop.h 新增**

```c
/* Cascade mode: current setpoint comes from g_cur_ref_ext_ma (no ramp) */
void CurLoop_SetExternalRef(bool enable);
extern volatile float g_cur_ref_ext_ma;
```

- [ ] **Step 2: cur_loop.c 实现**

```c
volatile float g_cur_ref_ext_ma = 0.0f;
static uint8_t s_use_ext_ref = 0;

void CurLoop_SetExternalRef(bool enable)
{
    s_use_ext_ref = enable ? 1 : 0;
}
```

PID 给定部分改为（保留原斜坡作为内部模式）：

```c
    float ref;
    if (s_use_ext_ref) {
        /* cascade: use speed-loop output directly, no ramp */
        ref = g_cur_ref_ext_ma;
    } else {
        ref = g_i_ref_ma;
        if (s_ref_ramp_active) {
            if (s_ref_ramp_start_us == 0) {
                s_ref_start = fb;
                s_ref_ramp_start_us = now;
            }
            uint64_t ramp_el = now - s_ref_ramp_start_us;
            uint64_t ramp_tot = (uint64_t)CURLOOP_REF_RAMP_MS * 1000UL;
            float ratio = (ramp_el >= ramp_tot) ? 1.0f : ((float)ramp_el / (float)ramp_tot);
            ref = s_ref_start + (g_i_ref_ma - s_ref_start) * ratio;
            if (ratio >= 1.0f) s_ref_ramp_active = 0;
        }
    }
    if (ref < 0.0f) ref = 0.0f;

    float duty = PID_UpdateUs(&s_pid, ref, fb, dt_us);
```

- [ ] **Step 3: 验证** grep `CurLoop_SetExternalRef` / `s_use_ext_ref`；括号配平
- [ ] **Step 4: 提交** `git commit -m "feat(cur_loop): external current reference entry (cascade)"`

---

## Task 4: 新建 ws/pos_loop.h/.c（位置环预留 stub）

**Files:**
- Create: `...\ws\pos_loop.h`
- Create: `...\ws\pos_loop.c`

- [ ] **Step 1: pos_loop.h**

```c
#ifndef __POS_LOOP_H__
#define __POS_LOOP_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 位置环（预留）：需位置反馈（编码器），当前为 stub。
 * 开启：MOTOR_LOOP_POSITION_ENABLE=1 并实现 PosLoop_GetPos()。 */
void  PosLoop_Init(void);
void  PosLoop_Update(float target_pos);   /* target: deg 或 counts */
float PosLoop_GetPos(void);               /* 当前位置（stub 返回 0） */
float PosLoop_GetOutput(void);            /* 输出 = 目标转速(rpm) */

#ifdef __cplusplus
}
#endif

#endif /* __POS_LOOP_H__ */
```

- [ ] **Step 2: pos_loop.c**

```c
#include "pos_loop.h"

static float s_out = 0.0f;

void PosLoop_Init(void) {}

void PosLoop_Update(float target_pos)
{
    (void)target_pos;
    s_out = 0.0f;   /* stub: no position feedback yet */
}

float PosLoop_GetPos(void) { return 0.0f; }   /* stub: implement encoder feedback */
float PosLoop_GetOutput(void) { return s_out; }
```

- [ ] **Step 3: 提交** `git commit -m "feat(pos_loop): reserved position-loop stub"`

---

## Task 5: runner 集成（mode 11 + 链路接线）

**Files:** Modify: `...\ws\dev_comm_runner.h`, `...\ws\dev_comm_runner.c`

- [ ] **Step 1: dev_comm_runner.h 增加枚举**

```c
    COMM_RUNNER_CASCADE_FW = 11, /* Macro-topology closed loop: speed/current per motor_config.h */
```

- [ ] **Step 2: dev_comm_runner.c includes + 初始化接线**

```c
#include "motor_config.h"
#include "speed_loop.h"
#include "pos_loop.h"
```

`CommRunner_Init` 末尾（`s_initialized=1` 前）：
```c
    SpeedLoop_Init();
    PosLoop_Init();
```

- [ ] **Step 3: SetMode - mode 11 与 mode 10 相同的启动流程（复制 case 10，日志改 CASCADE）**

```c
    case COMM_RUNNER_CASCADE_FW:
        if (s_hall) hall_3ch_stop(s_hall);
        calib_build_derived_tables();
        start_open_loop(s_cfg.ol_fly_start_us, s_cfg.ol_fly_target_us,
                        s_cfg.ol_fly_ramp_ms, 0);
        if (g_calib_table[1] <= 5u) {
            uint8_t hall = hall_3ch_read_raw(s_hall);
            if (hall >= 1u && hall <= 6u && g_calib_table[hall] <= 5u) {
                s_comm_step = g_calib_table[hall];
                Commutation_Step((uint8_t)s_comm_step, s_cfg.pwm_freq_hz, s_duty);
            }
        }
        s_sub_phase = 0;
        CurLoop_SetExternalRef(true);
        MAIN_D("[CommRunner] Mode=CASCADE_FW: timed open loop -> macro-topology loops");
        break;
```

- [ ] **Step 4: Update - mode 11 与 mode 10 相同的阶段机；阶段 1 里跑链路**

把 mode 10/11 的阶段 1 合并为同一处理，并加链路：
```c
        } else {
            /* Phase 1: closed-loop; chain wired by motor_config.h */
            hall_3ch_update(s_hall);
            if (hall_3ch_is_stalled(s_hall)) {
                MAIN_D("[CommRunner] CURLOOP stall, coast");
                CommRunner_SetMode(COMM_RUNNER_STOP);
                break;
            }
#if MOTOR_LOOP_POSITION_ENABLE
            PosLoop_Update(g_target_pos);   /* target from external (future) */
            float spd_ref = (MOTOR_SPD_REF_SRC == SPD_REF_FROM_POS)
                          ? PosLoop_GetOutput() : g_target_rpm;
            SpeedLoop_SetTarget(spd_ref);
#else
            SpeedLoop_SetTarget(g_target_rpm);
#endif
#if MOTOR_LOOP_SPEED_ENABLE
            float spd_out = SpeedLoop_Update(hall_3ch_get_rpm(s_hall));
#if MOTOR_LOOP_CURRENT_ENABLE
            g_cur_ref_ext_ma = spd_out;     /* speed -> current ref */
#else
            CommRunner_SetDuty(spd_out);    /* speed-only: output is duty */
#endif
#endif
        }
```
`g_target_pos` 声明（main.c 或 runner.c 顶部）：
```c
volatile float g_target_pos = 0.0f;   /* Keil Watch: reserved for position loop */
```
并在 dev_comm_runner.h 声明 extern。

- [ ] **Step 5: 模式门控 - cur_loop 的激活判断扩展 mode 11**

`CommRunner_CurLoopActive()`（或新 `CommRunner_CurrentLoopActive`）改为 mode 10 或 11 且 sub_phase==1：
```c
uint8_t CommRunner_CurrentLoopActive(void)
{
    return ((s_mode == COMM_RUNNER_CURLOOP_FW ||
             s_mode == COMM_RUNNER_CASCADE_FW) && s_sub_phase == 1) ? 1u : 0u;
}
```
cur_loop.c 的 gate 改调 `CommRunner_CurrentLoopActive()`。

- [ ] **Step 6: 验证** grep `CASCADE_FW`、`SpeedLoop_`、`CurrentLoopActive`；括号配平；README/注释同步
- [ ] **Step 7: 提交** `git commit -m "feat(runner): mode 11 cascade + chain wiring per motor_config.h"`

---

## Task 6: 文档与收尾

- [ ] **Step 1: CLAUDE.md** 模式列表补 11；cur_loop/speed_loop/pos_loop 文件清单
- [ ] **Step 2: PI电流环.md** 架构章节已写（本次计划前已追加）
- [ ] **Step 3: 提交** `git commit -m "docs: loop topology (mode 11, speed_loop, pos_loop)"`

---

## 验证清单（用户 Keil）
1. 编译通过；
2. **电流环 only**：`MOTOR_LOOP_SPEED_ENABLE=0, CURRENT=1, CUR_REF_SRC=FIXED`，跑 mode 10 —— 与现状一致；
3. **速度环 only**：`SPEED=1, CURRENT=0, DUTY_SRC=DIRECT`，跑 mode 11 —— 速度→占空比；
4. **级联**：`SPEED=1, CURRENT=1, CUR_REF_SRC=FROM_SPEED`，跑 mode 11 —— 速度→电流→占空比；
5. 堵转/限流：级联下 `g_i_ref_max_ma` 生效。