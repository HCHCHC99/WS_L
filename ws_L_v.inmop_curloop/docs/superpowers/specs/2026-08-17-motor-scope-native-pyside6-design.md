# MotorScope 原生版（PySide6）设计文档 —— Phase 1 核心

> 状态：已与用户确认（2026-08-17）。实施前需用户复核本文件，之后转 writing-plans。

## 1. 背景与目标

现有 `tools/motor_scope/` 是"Python 后端 + 浏览器前端"：
- `motor_scope.py`：J-Link RTT 读取、MOTF 解析、本地 HTTP 服务；
- `web/`（index.html / app.js / style.css）：浏览器 Canvas 渲染界面。

用户选择**方案 C：彻底原生重写**（不依赖任何浏览器/Web 技术），目标是双击 exe 即用。

**已确认决策**：
1. GUI 框架：**PySide6**（Qt6，LGPL，免费分发）；
2. 不做仿真模式（仅实机 J-Link）；
3. **Phase 1 核心先行**：数据链路 + 电机动画 + 5 示波器 + 仪表；日志/暂停回看/重连等进 Phase 2+；
4. 打包：PyInstaller onedir → `MotorScope.exe`。

## 2. 复用的现有逻辑（勿重复实现）

`tools/motor_scope/motor_scope.py` 中的纯 Python 部分直接 import 复用（该模块 `if __name__ == "__main__"` 才启动服务，import 安全）：
- `FocFrame`（dataclass，21 字段）
- `parse_text_line()` / `parse_binary()` / `_BIN_FMT`（MOTF 文本 21 字段 / 二进制 76B）
- `RttParser`（字节流 → FocFrame，文本/二进制自动识别）
- `JLinkRttSource`（pylink 连接、RTT 控制块搜索、通道读取）——其 `open()/read()` 逻辑可复用，需适配为线程内使用
- `DataHub` 的环形缓冲思想（`history` deque + `latest`）可精简移植

**固件端无需改动**（MOTF 协议已含 cnt，文本 21 字段 / 二进制 76B）。

## 3. MOTF 协议字段（Phase 1 显示所需，索引 0..19）

| idx | 字段 | 单位/说明 |
|---|---|---|
| 0 | mode | 0停止/1开环/2电流环/3对齐 |
| 1 | phase | 0idle/1hold/2ramp/3run/4align |
| 2 | rotor_mrad | 转子电角度 mrad |
| 3 | theta_mrad | 控制角电角度 mrad |
| 4 | iq_ma / 5 id_ma | dq 电流 mA |
| 6 | vq_mv / 7 vd_mv | dq 电压 mV |
| 8 | spd_rpm | 转速 |
| 9 | sync | 1=已同步 |
| 10 | diff_mrad | 控制角-转子角偏差 |
| 11 | freq_cHz | I-F 电频率 |
| 12 | ms | 时间戳 |
| 13 | mech_mrad | 连续机械角 mrad |
| 14 | is_ma / 15 is_angle_mrad | 电流矢量幅值/相角 |
| 16 | v_mv / 17 v_angle_mrad | 电压矢量幅值/相角 |
| 18 | theta_mech_mrad | 控制角机械角 |
| 19 | cnt | 编码器原始计数 |

注：共 20 个数据字段（idx 0..19）；文本 CSV = `MOTF,` + 20 个值（21 个 token，p[1..20]）；二进制 76B `<4sIiiiiiiiiiBBBBiiiiiii`。

## 4. 架构

```
┌─ QThread（数据线程）─────────────────────────────┐
│  JLinkRttSource.open() → RTT 读取循环             │
│  → RttParser.feed() → FocFrame                    │
│  → Qt 信号 frame_received(FocFrame)  → 主线程     │
└──────────────────────────────────────────────────┘
┌─ 主线程（GUI）───────────────────────────────────┐
│  Hub（环形缓冲 + latest + 帧计数/时间戳）          │
│  QTimer(~30ms) → 各控件从 Hub 取最新数据重绘        │
│  控件：MotorView / ScopeView×5 / Gauge / NumPanel  │
│  状态栏：连接、mode/phase/sync、fps、帧龄          │
└──────────────────────────────────────────────────┘
```

- 数据线程与 GUI 通过 `QtCore.Signal` 通信，Hub 写入在主线程（槽函数内），无锁竞争；
- 断连/异常：数据线程发 `error(str)` 信号，主线程在状态栏显示（Phase 1 不自动重连）。

## 5. 界面布局（镜像 web 版）

- 主窗口 QMainWindow，中央 QWidget + 布局：
  - 左侧：`MotorView`（560×560 左右，QPainter 自绘）；
  - 右侧列：`Gauge`（转速表）+ `NumPanel`（数值网格）；
  - 下方：5 个 `ScopeView`（iq/id、角度、diff、mode、cnt），等高约 110~160px；
- 顶部状态栏：连接指示、mode/phase/sync 胶囊、fps、帧龄、RTT 通道。

### 5.1 MotorView（QPainter）
- 背景/定子 12 槽、机械角刻度；
- 转子 20 磁钢（10 对极）+ ★ 星标（按 `mech_mrad/1000` 连续机械角绘制，注意机械角 = 电角度/极对数，画布角直接用电角度/PP）；
- 白色虚线 θ 控制角指针（`theta_mech_mrad`）；
- id（青）/ iq（橙）/ is（黄）/ v（品红虚线）矢量：幅值按量程缩放，角度 = 转子机械角 + dq 相角/极对数；
- 底部读数：θe转子 / θm机械 / θe控制 / iq / id / rpm / cnt。

### 5.2 ScopeView（QPainter，通用）
- 环形缓冲（复用 web 版 SCOPE_CAP=30000 思路，Python 侧用 collections.deque(maxlen=…) 或定长数组 + head 指针）；
- 网格 + Y 轴刻度 + 时间轴（X 轴最后 1.0s 窗口，跟随实时）；
- iq/id：双曲线自动量程；角度：0~360°；diff：±π；mode：0~4 阶梯；cnt：按窗口 [min,max] 自动量程（斜率=方向）；
- 悬停显示数值（QMouseEvent → 读缓冲插值）。

### 5.3 NumPanel / Gauge
- NumPanel：转速、cnt、转子电角、控制角、机械角、iq/id/vq/vd/is/|v|、频率、偏差（与 web 版数值面板一致）；
- Gauge：半圆转速表，量程自适应。

## 6. Phase 1 范围

**做**：实机 J-Link 数据链路、上述全部显示、悬停读值、状态栏。

**不做（Phase 2+）**：
- 日志面板（MOTF / 固件日志分面板、搜索、复制、history_*.txt）；
- 暂停冻结 / 回看历史（含电机/仪表跟随）；
- 重连 / 通道切换 UI；
- 窗口宽度滑动条；
- 仿真模式（明确不做）。

## 7. 打包（PyInstaller onedir）

- `pyinstaller -D --name MotorScope main.py`；
- 附加：pylink（及其依赖）、复用模块路径（把 `tools/motor_scope/motor_scope.py` 作为数据或做成包）；
- 运行时需 SEGGER J-Link 软件（`JLinkARM.dll`）已安装（用户已有）；
- 目标 Windows 10/11 x64；预计体积几十~一百多 MB。

## 8. 错误处理

- J-Link 打开/连接失败、RTT 控制块找不到、通道不存在 → 数据线程发错误信号，主线程状态栏红字显示，程序不崩溃；
- 帧中断（帧龄 >400ms）→ 状态栏提示"数据中断"，显示冻结在最新帧；
- 解析异常 → 跳过该行（沿用现有容错）。

## 9. 验证方式（无硬件也可验证的部分）

- 解析/协议：复用 `_verify_rtt_fields.py` 的思路，构造文本 21 字段/二进制 76B 帧验证解析（可在本机跑）；
- GUI 冒烟：Phase 1 提供一个 `--mock` 选项，用脚本生成的 MOTF 帧喂给数据线程，仅用于**开发自测界面渲染**（不是用户可用的仿真模式，不影响【不做仿真】的决策；正式使用仍走 J-Link）；
- 实机：用户接 J-Link + 电机验证。

## 10. 风险

- PySide6 首次安装/体积；
- pylink 与 Qt 事件循环兼容性（数据线程独立，风险低）；
- 复用 `motor_scope.py` 时避免引入其 HTTP/线程副作用（import 安全，已确认）；
- 绘制性能：QPainter 重绘 5 示波器 + 电机，30ms 周期足够（web 版 60fps 也能跑，Qt 更稳）。
