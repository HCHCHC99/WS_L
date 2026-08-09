# PI 负反馈（电流环 / 速度环）调试记录

> 记录日期：2026-08-09
> 分支：`codex/loop-architecture`
> **当前状态：电流环尚未调好**（空载大 ref 下 fb 追不上 ref；采样高频噪声仍未压到满意；Kp/Ki 未完成整定）。
> 本文记录两件事：① 当前代码里"预留的可调设置"；② 调试电流环一路走来的思路与结论。

---

## 1. 预留的可调设置（当前代码现状）

### 1.1 总频率宏（`ws/motor_config.h`）

| 宏 | 当前值 | 说明 |
|---|---|---|
| `MOTOR_PWM_FREQ_HZ` | `20000u` | PWM 频率 = ADC 采样频率 = 电流环频率（1:1）。改一个宏即可换 10k/25k/50k |

### 1.2 环拓扑宏（`ws/motor_config.h`）

| 宏 | 当前值 | 说明 |
|---|---|---|
| `MOTOR_LOOP_POSITION_ENABLE` | `0` | 位置环预留（需编码器，默认关） |
| `MOTOR_LOOP_SPEED_ENABLE` | `1` | 速度环 |
| `MOTOR_LOOP_CURRENT_ENABLE` | `1` | 电流环 |
| `MOTOR_CUR_REF_SRC` | `CUR_REF_FROM_SPEED` | 电流给定来源：级联（速度环）/ 固定 |
| `MOTOR_DUTY_SRC` | `DUTY_FROM_CURRENT` | 占空比来源 |

- 非法组合会被 `dev_comm_runner.c` 顶部的 `#error` 编译期拦截。
- 拓扑组合：电流环 only（SPEED=0,CUR=1,CUR_REF=FIXED）、速度环 only（CUR=0,DUTY=DIRECT）、级联（当前）。

### 1.3 电流环（`ws/cur_loop.c` / `cur_loop.h`）

**运行时 Keil Watch 可调（最常用）：**

| 变量 | 当前值 | 单位/含义 |
|---|---|---|
| `g_cur_pid_cfg.kp` | `0.1` | P 增益（% duty / mA） |
| `g_cur_pid_cfg.ki` | `0.1` | I 增益（% duty / (mA·s)） |
| `g_cur_pid_cfg.i_term_max` | `20` | 积分项输出钳位（±20% duty，anti-windup） |
| `g_i_ref_ma` | `150` | mode 10 固定电流给定（mA） |
| `g_cur_fb_alpha` | `0.3` | 反馈一阶平滑系数：1.0=关，0.1=强 |
| `g_cur_dbg_ms` | `20` | `[CURLOOP]` RTT 日志间隔（ms），0=关 |

**编译期参数（改代码重编译）：**

| 宏 | 当前值 | 说明 |
|---|---|---|
| `CURLOOP_WIN_SIZE` | `8`（20kHz） | 反馈滑窗点数，按频率自适应保持 ~400us（50k→20, 25k→10, 20k→8, 10k→4） |
| `CURLOOP_BLANK_SKIP` | `3` | 换相后跳过 3 拍（~135us）再填窗，避开暂态/错相样本 |
| `CURLOOP_DUTY_RATE` | `20` | 每拍最大 duty 变化（%）；20kHz 下全行程 ~250us |
| `CURLOOP_REF_RAMP_MS` | `1000` | mode 10 ref 软启动斜坡（1s） |
| `g_cur_pid_cfg.output_min/max` | `2 / 98` | 占空比钳位（预驱 2%~98%） |
| `g_cur_pid_cfg.integral_max` | `500` | 积分累积钳位（mA·s） |

**反馈链路（当前实现顺序）：**
```
ADC 采样(20k) → 选 active 相(按 g_scope_step) → 换相 blanking(3拍)
  → 8点滑窗(400us) → 一阶平滑(alpha) → PID → duty限速 → OCCR
dt：Timer6 us 时间戳实测
```

### 1.4 速度环（`ws/speed_loop.c` / `speed_loop.h`）

| 变量 | 当前值 | 说明 |
|---|---|---|
| `g_spd_pid_cfg.kp` | `0.02` | 速度环 P（偏小，级联需加大——遗留默认） |
| `g_spd_pid_cfg.ki` | `0.005` | 速度环 I |
| `g_spd_pid_cfg.kd` | `0` | D 未启用 |
| `g_spd_pid_cfg.update_ms` | `50` | 主循环 50ms 节流 |
| `g_spd_pid_cfg.integral_max` | `300000` | 积分钳位 |
| `g_i_ref_max_ma` | `1500` | 级联电流上限（=速度环输出上限，Watch 可调） |
| `g_target_rpm` | `3000`（main.c） | 速度环目标（Watch 可调） |

- 输出语义随拓扑：电流环开 → mA（0~`g_i_ref_max_ma`）；电流环关 → duty%（2~98）。
- `SpeedLoop_Seed()` 用于开环→闭环无扰交接（已接线 mode 11）。

### 1.5 观测变量（J-Scope / RTT）

**电流（全部 float，J-Scope 不回绕）：**

| 变量 | 含义 |
|---|---|
| `g_scope_i_ref` | 电流给定（PID 实际用的） |
| `g_scope_i_fb` | 平滑后反馈（PID 用的） |
| `g_scope_i_fb_raw` | 平滑前（8 点滑窗）反馈——对比滤波效果 |
| `g_scope_i_duty` / `g_scope_i_err` | 占空比 / 误差 |
| `g_scope_i_dt_us` | 控制拍间隔（us，20k 应≈50us） |
| `g_i_iu_ma / iv / iw_ma` | 三相原始电流（float） |
| `g_i_iu_filt / iv / iw_filt` | 三相 Biquad 滤波（float，80Hz@20k） |
| `g_i_uvw_ma` | 三相原始和（应≈0；诊断采样噪声） |
| `g_scope_iu_ma / iv / iw_ma` | 三相 Biquad 滤波（= g_i_*_filt） |

**速度：** `g_scope_spd_ref / rpm / err / out`
**其他：** `g_scope_step`、`g_scope_rpm`、`g_target_pos`（位置环预留）

### 1.6 运行模式与调试日志

- `comm_mode=5`：霍尔校准表；`comm_mode=10`：电流环 only；`comm_mode=11`：级联。
- `[CURLOOP]` 日志字段（全整型）：`tus dt step ref fb raw err duty i kp ki al`
  - `raw`=平滑前，`fb`=平滑后，`kp/ki`=×1000，`al`=alpha×100。

---

## 2. 电流环调试思路（过程记录）

### 阶段 A：反馈链路确认与观测（已完成）
- 把电流观测变量 int16/int32 → **float**（`g_i_*_ma/filt/uvw_ma`），解决 J-Scope int16 回绕（-2 显示 65533）；新增 `g_scope_iu/iv/iw_ma`。
- 诊断数据：**`g_i_uvw_ma`（原始三相和）±200 跳动；`g_scope_ix_ma`（80Hz 滤波）±13**。
  - 结论：三相直流/低频平衡 OK（零偏、增益没问题）；**高频采样噪声大**是 fb 抖动的主要来源之一。

### 阶段 B：抖动来源排查（已完成/部分）
1. **禁用 BEMF 实验**（`BEMF_SEQA_TRIGGER_ENABLE=0`，ADC1 只跑电流 SEQ_B）：fb 仍抖 → **排除"SEQ_A/SEQ_B 共享触发"为主因**（实验宏保留在 `Bemf.h`，可恢复）。
2. **dt 分析**：大部分 44~47us（20kHz 正常）；周期性 490~500us = 换相 blanking(3 拍)+填窗(8 拍)=11 拍×45us 的**设计空档**（非故障；但高速时会造成控制稀疏，需留意）。
3. 结论：fb 抖动 = **原始采样高频噪声 + 换相暂态**；当时只有 4 点滑窗（200us），滤波不足。

### 阶段 C：fb 滤波升级（已实施，`df3e118` / `c9ecbd4`）
- 8 点滑窗（400us）+ 换相后跳过 3 拍（blanking）+ 一阶平滑 `g_cur_fb_alpha`（默认 0.3，Watch 可调）。
- 代价：反馈延迟 ≈ 400us+170us ≈ **570us**，电流环带宽降到 ~300Hz，**Kp 需配合下调**（0.1 → 0.05~0.08 试）。

### 阶段 D：空载物理认知（关键认知，`pi_负反馈.md` 记录于此）
- 电流环是**电流伺服/执行器**，不是"增压泵"：它让 `i = ref`，但**能否达到取决于负载需求电流 ≥ ref**。
- 空载需求 ~40mA；`ref=150` 空载 → 多余转矩让电机**持续加速** → BEMF 升高 → 电流被压回 → **fb 永远 < ref（物理，不是调参问题）**。
- 验证电流环"调好了吗"的正确方法：**加载（手捏/堵转）或 ref ≤ 空载需求**，看 fb 是否快速贴上 ref 并稳定、duty 稳定。

### 阶段 E：当前状态与待办（未完成）
**未完成：**
- 电流环 Kp/Ki 尚未用"阶跃法"整定；
- fb 抖动（采样噪声）尚未压到满意；
- 空载/加载下的闭环功能尚未最终确认（待手捏/小 ref 验证）。

**待办（按顺序）：**
1. 手捏电机（负载需求 ≥150mA）或 ref=40~50，确认 fb 能否快速趋近并稳定 → 确认电流环功能；
2. `g_cur_fb_alpha` 在 0.15~0.5 间找"平滑 vs 带宽"平衡；
3. 阶跃法整定：ki=0 → kp 从 0.05 起 ×2 找临界 → 取 0.5 倍 → 加回 ki；
4. 若抖动仍大：采样点优化（PWM 峰值→谷值）或去极值平均；
5. 恢复 BEMF（`BEMF_SEQA_TRIGGER_ENABLE=1`）或明确长期禁用；
6. 电流环稳定后：调速度环（Kp/Ki 加大）→ mode 11 级联验证。

---

## 3. 关键结论 / 经验

1. **空载不能测电流环跟随**——空载平衡电流只有几十 mA，大 ref 只会让电机加速。
2. **抖动要分层**：采样噪声（→滤波 alpha/滑窗）≠ 控制放大（→Kp 太大，看 duty 是否跟着抖）≠ 物理（→负载/转速）。
3. **dt≈495us 是换相 blanking+填窗的设计空档**（11 拍×45us），不是 bug；高速时注意控制稀疏。
4. **在线调参方法**：运行中改 `g_cur_pid_cfg.kp/ki` 实时生效；判据是**阶跃响应**（上升时间/超调/振荡），不是抖动幅度；调 P 时 `ki=0`，同时看 `g_scope_i_duty` 区分噪声 vs 放大。
5. **滤波的代价是带宽/延迟**：8 点滑窗 400us + alpha 0.3 ≈570us 延迟，Kp 要相应下调。

---

## 4. 当前参数快照（提交 `de75fe5` 之后）

```
MOTOR_PWM_FREQ_HZ = 20000u
电流环: kp=0.1 ki=0.1 i_term_max=20 integral_max=500 out[2,98]
       g_i_ref_ma=150  g_cur_fb_alpha=0.3  g_cur_dbg_ms=20
       WIN_SIZE=8  BLANK_SKIP=3  DUTY_RATE=20  REF_RAMP_MS=1000
速度环: kp=0.02 ki=0.005 kd=0 update_ms=50 integral_max=300000
       g_i_ref_max_ma=1500  g_target_rpm=3000
拓扑:   POS=0 SPD=1 CUR=1 CUR_REF=FROM_SPEED DUTY=FROM_CURRENT（级联）
实验:   BEMF_SEQA_TRIGGER_ENABLE=0（BEMF 停采，待恢复/确认）
```