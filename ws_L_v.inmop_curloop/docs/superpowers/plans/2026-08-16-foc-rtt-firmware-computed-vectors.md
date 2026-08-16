# FOC RTT 固件直传电流/电压矢量与控制角机械角 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 固件在 MOTF 发送路径（主循环 1kHz 节流点）用单精度浮点算好并直传 5 个新字段——is 电流矢量幅值/相角、v 电压矢量幅值/相角、theta 控制角机械角——motor_scope 解析后仅做展示（is/v 箭头、θ 指针、数值面板直接用固件值，不再由 id/iq/vd/vq 合成）。

**Architecture:** 扩展现有 MOTF 协议：文本帧 15→20 字段、二进制帧 52→72B。固件在 `Foc_RttSend()` 节流点之后用 `sqrtf/atan2f` 计算（不进 20kHz ISR，评估开销 <1% CPU）。`motor_scope.py` 同步扩展解析与仿真；`web/app.js` 用固件值绘制 is/v 矢量与 θ 控制角指针，并新增 is/|v| 数值行。角度语义：is/v 相角为 dq 坐标系内电角度 mrad（`atan2(q,d)`），前端画布角 = 转子机械角 + dq角/极对数。

**Tech Stack:** C (Keil MDK / ARM Compiler 5/6, HC32F460 Cortex-M4F FPU @200MHz), Python 3.10, 原生 JS/Canvas。

**测试约束：** 固件无 CLI 构建（唯一构建定义是 Keil .uvprojx），固件改动用"读回代码 + grep 符号 + 逻辑走查"验证，最终由用户在 Keil 编译烧录；上位机（Python/JS）有可执行测试：文本/二进制解析、旧帧兼容、仿真连续性与端到端 sim 模式。

---

## Task 1: 固件 `ws/foc.c` — 结构体 + 计算辅助 + 文本/二进制两分支

**Files:**
- Modify: `D:\WS_L\ws_L_v.inmop_curloop\HC32F460_DDL_Rev3.3.0\projects\ev_hc32f460_lqfp100_v2\ws\foc.c`

- [ ] **Step 1: 二进制帧结构体加 5 个 int32（52→72 字节）**

把注释 `/* 二进制帧：52 字节（小端），PC 端按小端解析 */` 改为 `/* 二进制帧：72 字节（小端），PC 端按小端解析 */`。

在 `int32_t  mech_mrad;` 之后、`} foc_rtt_frame_t;` 之前加：

```c
    int32_t  is_ma;           /* sqrt(id^2+iq^2) mA（固件直传） */
    int32_t  is_angle_mrad;   /* atan2(iq,id) dq 电角度 mrad */
    int32_t  v_mv;            /* sqrt(vd^2+vq^2) mV */
    int32_t  v_angle_mrad;    /* atan2(vq,vd) dq 电角度 mrad */
    int32_t  theta_mech_mrad; /* g_foc_theta_rad / FOC_POLE_PAIRS * 1000 */
```

- [ ] **Step 2: 新增 5 个计算辅助函数（放在 `Foc_MechAngleMrad()` 之后）**

```c
/* 电流/电压矢量 + 控制角机械角：dq 合成，供 MOTF 直传（主循环调用，不进 ISR）。
 * 全单精度 sqrtf/atan2f（M4F VSQRT + 硬件 FPU，1kHz 下开销 <1% CPU）。 */
static int32_t Foc_IsMagMa(void)       { return (int32_t)sqrtf(g_foc_id_ma * g_foc_id_ma + g_foc_iq_ma * g_foc_iq_ma); }
static int32_t Foc_IsAngleMrad(void)   { return (int32_t)(atan2f(g_foc_iq_ma, g_foc_id_ma) * 1000.0f); }
static int32_t Foc_VMagMv(void)        { return (int32_t)(sqrtf(g_foc_vd * g_foc_vd + g_foc_vq * g_foc_vq) * 1000.0f); }
static int32_t Foc_VAngleMrad(void)    { return (int32_t)(atan2f(g_foc_vq, g_foc_vd) * 1000.0f); }
static int32_t Foc_ThetaMechMrad(void) { return (int32_t)(g_foc_theta_rad * (1000.0f / (float)FOC_POLE_PAIRS)); }
```

（`foc.c` 顶部已 `#include <math.h>`，`g_foc_id_ma/g_foc_iq_ma/g_foc_vd/g_foc_vq/g_foc_theta_rad` 均已存在。）

- [ ] **Step 3: 二进制分支赋值**

在 `fr.mech_mrad  = Foc_MechAngleMrad();` 之后、`SEGGER_RTT_Write(...)` 之前加：

```c
        fr.is_ma           = Foc_IsMagMa();
        fr.is_angle_mrad   = Foc_IsAngleMrad();
        fr.v_mv            = Foc_VMagMv();
        fr.v_angle_mrad    = Foc_VAngleMrad();
        fr.theta_mech_mrad = Foc_ThetaMechMrad();
```

- [ ] **Step 4: 文本分支（buf 160→224 + 格式串 + 5 个参数）**

`char buf[160];` 改为 `char buf[224];`（20 个字段最坏约 199 字节）。

格式串：
`"MOTF,%u,%u,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%u,%d\r\n"` 改为
`"MOTF,%u,%u,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%u,%d,%d,%d,%d,%d,%d\r\n"`

参数末尾 `(int)Foc_MechAngleMrad());` 改为：

```c
            (unsigned)ms,
            (int)Foc_MechAngleMrad(),
            (int)Foc_IsMagMa(),
            (int)Foc_IsAngleMrad(),
            (int)Foc_VMagMv(),
            (int)Foc_VAngleMrad(),
            (int)Foc_ThetaMechMrad());
```

- [ ] **Step 5: 读回验证**

Run（在工程根目录）:
`Select-String -Path ws\foc.c -Pattern "is_ma|is_angle_mrad|v_mv|v_angle_mrad|theta_mech_mrad|72 字节|buf\[224\]"`
Expected: 结构体 5 字段、5 个辅助函数、二进制 5 行赋值、文本格式串 20 个 `%` 参数、`buf[224]` 全部出现且无遗漏。

---

## Task 2: `tools/motor_scope/motor_scope.py` — 解析 + 仿真

**Files:**
- Modify: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope\motor_scope.py`

- [ ] **Step 1: FocFrame 加 5 字段**

在 `ms: int = 0` 之后加：

```python
    is_ma: float = 0.0           # 固件直传 is 电流矢量幅值（mA）
    is_angle_mrad: float = 0.0   # 固件直传 is 相角（dq 电角度，mrad）
    v_mv: float = 0.0            # 固件直传 v 电压矢量幅值（mV）
    v_angle_mrad: float = 0.0    # 固件直传 v 相角（dq 电角度，mrad）
    theta_mech_mrad: float = 0.0 # 固件直传控制角机械角（theta/极对数，mrad）
```

- [ ] **Step 2: to_list 追加 5 个**

`int(round(self.mech_mrad))]` 改为：

```python
                int(round(self.mech_mrad)),
                int(round(self.is_ma)), int(round(self.is_angle_mrad)),
                int(round(self.v_mv)), int(round(self.v_angle_mrad)),
                int(round(self.theta_mech_mrad))]
```

- [ ] **Step 3: parse_text_line 读 p[15..19]**

`mech_mrad=float(p[14]) if len(p) > 14 else 0,` 之后加：

```python
            is_ma=float(p[15]) if len(p) > 15 else 0,
            is_angle_mrad=float(p[16]) if len(p) > 16 else 0,
            v_mv=float(p[17]) if len(p) > 17 else 0,
            v_angle_mrad=float(p[18]) if len(p) > 18 else 0,
            theta_mech_mrad=float(p[19]) if len(p) > 19 else 0,
```

同时把 docstring `MOTF,<14 个字段>` 改为 `<19 个字段>`，模块头帧格式行补 `<is_ma>,<is_angle_mrad>,<v_mv>,<v_angle_mrad>,<theta_mech_mrad>`。

- [ ] **Step 4: 二进制格式 52→72B + parse_binary**

`_BIN_FMT = "<4sIiiiiiiiiiBBBBi"` 改为 `_BIN_FMT = "<4sIiiiiiiiiiBBBBiiiiii"     # 72 字节小端二进制帧`。

`parse_binary` 解包与返回改为：

```python
    (magic, ms, rotor, theta, iq, id_, vq, vd, spd, diff, freq,
     mode, phase, sync, rsv, mech, is_ma, is_ang, v_mv, v_ang,
     theta_mech) = struct.unpack(_BIN_FMT, buf[:_BIN_SIZE])
    return FocFrame(mode=mode, phase=phase, rotor_mrad=float(rotor),
                    theta_mrad=float(theta), iq_ma=float(iq), id_ma=float(id_),
                    vq_mv=float(vq), vd_mv=float(vd), spd_rpm=float(spd),
                    sync=sync, diff_mrad=float(diff), freq_cHz=float(freq),
                    ms=ms, mech_mrad=float(mech),
                    is_ma=float(is_ma), is_angle_mrad=float(is_ang),
                    v_mv=float(v_mv), v_angle_mrad=float(v_ang),
                    theta_mech_mrad=float(theta_mech))
```

docstring `解析 52 字节` 改为 `解析 72 字节`。

- [ ] **Step 5: SimFoc 生成 5 字段**

phase 2 块内 `f.mech_mrad = (max(0.0, th_cont - lag) / self.pp) * 1000.0` 之后加：

```python
            f.theta_mech_mrad = (th_cont / self.pp) * 1000.0   # 固件直传控制角机械角
```

phase 3 块内 `f.mech_mrad = ((th_cont + 0.25) / self.pp) * 1000.0` 之后加：

```python
            f.theta_mech_mrad = (th_cont / self.pp) * 1000.0
```

在函数末尾 `f.vd_mv = 40.0 + f.id_ma * 0.1` 之后加：

```python
        # 与固件 Foc_RttSend 同公式：is/v 幅值与 dq 相角（id/iq、vd/vq 单位分别为 mA、mV）
        f.is_ma = math.hypot(f.id_ma, f.iq_ma)
        f.is_angle_mrad = math.atan2(f.iq_ma, f.id_ma) * 1000.0
        f.v_mv = math.hypot(f.vd_mv, f.vq_mv)
        f.v_angle_mrad = math.atan2(f.vq_mv, f.vd_mv) * 1000.0
```

- [ ] **Step 6: sim_loop MOTF 日志行追加 5 字段**

`f"{int(f.mech_mrad)}")` 改为：

```python
                f"{int(f.mech_mrad)},{int(f.is_ma)},{int(f.is_angle_mrad)},"
                f"{int(f.v_mv)},{int(f.v_angle_mrad)},{int(f.theta_mech_mrad)}")
```

---

## Task 3: Python 测试（写测试 → 跑测试）

**Files:**
- Create: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope\_verify_rtt_fields.py`（临时验证脚本，跑完可删）

- [ ] **Step 1: 写验证脚本**

```python
# -*- coding: utf-8 -*-
import struct, sys, io, math
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
import motor_scope as ms

# 1) 文本帧（20 字段）
line = ("MOTF,2,3,3141,6283,-2000,-25,5000,400,3000,1,250,2000,"
        "123456,78540,2061,3217,5024,-1234,628\r\n")
fr = ms.parse_text_line(line)
assert fr is not None
assert (fr.is_ma, fr.is_angle_mrad, fr.v_mv, fr.v_angle_mrad,
        fr.theta_mech_mrad) == (2061.0, 3217.0, 5024.0, -1234.0, 628.0), fr
print("text 20-field OK:", fr)

# 2) 旧 15 字段文本帧兼容
old = ms.parse_text_line(
    "MOTF,2,3,3141,6283,-2000,-25,5000,400,3000,1,250,2000,123456,78540\r\n")
assert old is not None and old.is_ma == 0.0 and old.v_mv == 0.0
print("old 15-field compat OK")

# 3) 二进制帧 72B round-trip
assert ms._BIN_SIZE == 72, ms._BIN_SIZE
packed = struct.pack(ms._BIN_FMT, b"MOTF", 123456, 3141, 6283, -2000, -25,
                     5000, 400, 3000, 250, 2000, 2, 3, 1, 0,
                     78540, 2061, 3217, 5024, -1234, 628)
assert len(packed) == 72
fb = ms.parse_binary(packed)
assert (fb.is_ma, fb.is_angle_mrad, fb.v_mv, fb.v_angle_mrad,
        fb.theta_mech_mrad) == (2061.0, 3217.0, 5024.0, -1234.0, 628.0)
print("binary 72B OK:", fb)

# 4) to_list 长度与前端索引一致（latest[18] = theta_mech_mrad）
lst = fr.to_list()
assert len(lst) == 19 and lst[14] == 2061 and lst[18] == 628, lst
print("to_list 19 fields OK")

# 5) 仿真：is/v 幅值非负、theta_mech 连续无跳变
sim = ms.SimFoc(sample_hz=200)
prev = None
for i in range(2000):
    f = sim.next_frame()
    assert f.is_ma >= 0.0 and f.v_mv >= 0.0
    if prev and f.phase in (2, 3) and prev.phase in (2, 3):
        assert abs(f.theta_mech_mrad - prev.theta_mech_mrad) < 500.0
        assert f.theta_mech_mrad > 0.0
    prev = f
print("sim is/v/theta_mech OK")
print("ALL TESTS PASSED")
```

- [ ] **Step 2: 运行**

Run: `py _verify_rtt_fields.py`（在 `tools\motor_scope` 目录）
Expected: 打印 `ALL TESTS PASSED`，无 assert 失败。

---

## Task 4: `tools/motor_scope/web/app.js` — 展示改用固件值

**Files:**
- Modify: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope\web\app.js`

- [ ] **Step 1: poll() 映射 5 个最新字段 + history 加 thetaMechDeg**

`ms: d.latest[12], mech: d.latest[13],` 改为：

```js
        ms: d.latest[12], mech: d.latest[13],
        isMa: d.latest[14], isAng: d.latest[15],
        vMv: d.latest[16], vAng: d.latest[17], thetaMech: d.latest[18],
```

history 的 `scopeAppend({...})` 里 `mechDeg: (f[13] / 1000) * RAD2DEG,` 之后加：

```js
        thetaMechDeg: (f[18] / 1000) * RAD2DEG,   // 固件直传控制角机械角 deg
```

- [ ] **Step 2: 环形缓冲 + 入库 + 插值**

`hub.scope` 里 `mechDeg: new Float32Array(SCOPE_CAP),` 之后加：

```js
  thetaMechDeg: new Float32Array(SCOPE_CAP),
```

`scopeAppend` 里 `s.mechDeg[idx] = p.mechDeg;` 之后加：

```js
  s.thetaMechDeg[idx] = p.thetaMechDeg;
```

声明行 `let visRotorMech = 0, visRotorElec = 0, visCtrlElec = 0;` 改为：

```js
let visRotorMech = 0, visCtrlMech = 0, visRotorElec = 0, visCtrlElec = 0;
```

`advance()` 末尾 `visRotorMech = mech;` 之后加：

```js
  // 控制角机械角：固件直传（theta/极对数），直接插值/外推
  let ctrlMech = s.thetaMechDeg[iLast];
  if (span > 1e-6) {
    const dCtrlMech = s.thetaMechDeg[iLast] - s.thetaMechDeg[iPrev];
    ctrlMech = s.thetaMechDeg[iLast] + dCtrlMech / span * ext;
  }
  visCtrlMech = ctrlMech;
```

- [ ] **Step 3: drawMotor — θ 指针、is/v 箭头改用固件值**

θ 指针 `const ctrlMech = visCtrlElec / POLE_PAIRS;` 改为：

```js
  const ctrlMech = visCtrlMech;   // 固件直传控制角机械角
```

is 矢量块（删除 idX/idY/iqX/iqY 与 hypot/atan2，改用固件值）：

```js
    const isL = (f.isMa / maxA) * Lmax;                                   // 固件直传 is 幅值
    const isAng = visRotorMech + (f.isAng / 1000) * RAD2DEG / POLE_PAIRS; // dq 电角度→机械画布角
    drawVector(ctx, "#22d3ee", idL, visRotorMech, 4);          // id
    drawVector(ctx, "#fb923c", iqL, qAng, 4);                  // iq
    drawVector(ctx, "#facc15", isL, isAng, 5);                 // is
```

v 矢量块改为：

```js
    const vLmax = 100;
    const vL = f.vMv / 1000 * vLmax;                                      // 固件直传 v 幅值
    const vAng = visRotorMech + (f.vAng / 1000) * RAD2DEG / POLE_PAIRS;   // dq 电角度→机械画布角
    if (vL > 3) drawVector(ctx, "#e879f9", vL, vAng, 3, [4, 4]);
```

- [ ] **Step 4: 数值面板新增 is、|v| 两行**

`buildNumGrid` 的 rows 里：

```js
    ["iq", "iqVal", "0 mA"],
    ["id", "idVal", "0 mA"], ["is", "isVal", "0 mA"],
    ["vq", "vqVal", "0 mV"],
    ["vd", "vdVal", "0 mV"], ["|v|", "vMagVal", "0 mV"],
```

`updateNum` 里 `set("vdVal", ...)` 之后加：

```js
  set("isVal", f.isMa.toFixed(0) + " mA");
  set("vMagVal", f.vMv.toFixed(0) + " mV");
```

- [ ] **Step 5: 语法检查**

Run: `node --check web\app.js`
Expected: 退出码 0，输出 `app.js syntax OK`。

---

## Task 5: `tools/motor_scope/README.md` — 协议文档更新

**Files:**
- Modify: `D:\WS_L\ws_L_v.inmop_curloop\tools\motor_scope\README.md`

- [ ] **Step 1: 帧格式与字段表**

帧格式块末尾追加：

```
     <is_ma>,<is_angle_mrad>,<v_mv>,<v_angle_mrad>,<theta_mech_mrad>
```

字段表 `mech_mrad` 行之后追加 5 行：

```markdown
| is_ma | √(id²+iq²) | **固件直传 is 电流矢量幅值**（mA） |
| is_angle_mrad | atan2(iq,id)×1000 | **固件直传 is 相角**（dq 电角度 mrad） |
| v_mv | √(vd²+vq²)×1000 | **固件直传 v 电压矢量幅值**（mV） |
| v_angle_mrad | atan2(vq,vd)×1000 | **固件直传 v 相角**（dq 电角度 mrad） |
| theta_mech_mrad | g_foc_theta_rad/极对数×1000 | **固件直传控制角机械角**（mrad） |
```

- [ ] **Step 2: 二进制帧说明与注意点**

`52 字节小端二进制帧` 改为 `72 字节小端二进制帧`；`（末尾多一个 mech_mrad int32）` 改为 `（末尾多 mech_mrad + is/v/theta_mech 共 6 个 int32）`。

注意区补一条：

```markdown
- is/v 相角为 dq 坐标系内电角度 mrad，前端画布角 = 转子机械角 + dq角/极对数；
  固件在 `Foc_RttSend`（主循环 1kHz）用单精度 sqrtf/atan2f 计算，不进 20kHz ISR。
```

---

## Task 6: 端到端验证

**Files:**
- Run only（不改文件）

- [ ] **Step 1: 仿真端到端**

Run（在 `tools\motor_scope`）：
`py -c "import subprocess,sys,time,urllib.request,json; p=subprocess.Popen([sys.executable,'motor_scope.py','--mode','sim-foc','--rate','200','--no-browser','--port','8099']); time.sleep(4); d=json.load(urllib.request.urlopen('http://127.0.0.1:8099/data?since=0')); print(d['latest']); p.terminate()"`
Expected: `latest` 是 19 元素列表，`latest[14..18]` 为 is_ma/is_angle/v_mv/v_angle/theta_mech，ramp 阶段 theta_mech > 0。

- [ ] **Step 2: 全套回归**

Run（在 `tools\motor_scope`）：`py _verify_rtt_fields.py`
Expected: `ALL TESTS PASSED`。再跑 `node --check web\app.js`，退出码 0。

- [ ] **Step 3: 固件走查**

Run: `Select-String -Path ws\foc.c -Pattern "Foc_IsMagMa|Foc_VAngleMrad|Foc_ThetaMechMrad|72 字节|buf\[224\]"`
Expected: 全部出现。最终由用户在 Keil 编译烧录后，以 `--mode jlink` 实机验证：is/v 箭头、θ 指针、数值面板 is/|v| 与示波器机械角曲线正常。

---

## Self-Review

- Spec 覆盖：5 个字段（is_ma/is_angle_mrad/v_mv/v_angle_mrad/theta_mech_mrad）在 Task1(固件) / Task2(解析+仿真) / Task4(展示) / Task5(文档) 全部落地；开销评估已在前置讨论完成。
- 无占位符：所有步骤含完整代码与验证命令。
- 类型一致：字段名在 C/Python/JS 三端统一为 is_ma/is_angle_mrad/v_mv/v_angle_mrad/theta_mech_mrad；前端 latest 索引 [14..18]、to_list 长度 19。
- 兼容性：文本旧帧（15 字段）解析缺省 0；二进制必须三端同步更新（已注明）。
