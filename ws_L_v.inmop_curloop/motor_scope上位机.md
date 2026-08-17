# MotorScope 上位机交接（260817）

> 交接日期：2026-08-17
> 仓库：`D:\WS_L\ws_L_v.inmop_curloop`，分支 `inmop_cur_loop_rtt`
> 上位机分**两套**：
> 1. **Web 版**（`tools\motor_scope\`，浏览器 + Python HTTP 服务）——历史版本，功能齐全；
> 2. **原生版**（`tools\motor_scope_native\`，PySide6 + PyInstaller 打包 exe）——**当前主力**，已移植 web 全部核心功能。

---

## 一、原生版（当前主力）文件结构

路径：`tools\motor_scope_native\`

| 文件 | 职责 |
|---|---|
| `main.py` | 入口：argparse（--mock/--mock-rate/--device/--speed-khz/--channel/--serial/--rtt-addr/--rtt-ram-base/--rtt-ram-size）；退出时**线程停止后清空 history 文件** |
| `app.py` | 主窗口：工具栏（J-Link 序列号 / **芯片** / **SWD 速度** / RTT 通道 / **RAM 基址·大小 / RTT 地址** + 应用/刷新）、暂停/实时、窗口对数滑条(0.1~20s)、时间回看滑条、电机+仪表+5 示波器+日志分面板、状态栏（错误分级配色） |
| `data.py` | `DataThread`：J-Link RTT（复用 motor_scope 的 `JLinkRttSource`/`RttParser`/`DataHub`）或 `--mock` 自测源；`frame_received` / `log_line` / `data_error` / `data_status` 信号；jlink 模式写 `history_scope.txt`/`history_main.txt`；`set_device/set_speed/set_ram/set_channel/set_serial` |
| `hub.py` | 数据缓冲（环形 deque，20s/30000 点）+ **暂停/回看导航模型**（`follow_live` / `view_end` / `window_sec` / `view_frame` 时间插值）+ 日志缓冲（2000 条） |
| `widgets\motor_view.py` | 电机剖视图：定子 12 槽、20 磁钢（N 红/S 蓝）、星标★、d/q 轴、θ 指针、id/iq/is/v 矢量、底部读数两行 |
| `widgets\scope_view.py` | 通用示波器（cur/angle/diff/mode/cnt），标题 + Y 轴单位(旋转) + X 轴 t/s + 悬停读数 |
| `widgets\gauge.py` | 半圆转速表 |
| `widgets\num_panel.py` | 数值面板（转速/cnt/转子电角/控制角/机械角/iq/id/is/vq/vd/|v|/频率/diff） |
| `widgets\log_panel.py` | **日志分面板**：MOTF 面板（搜索/复制/清空/计数）+ 固件日志面板（MAIN_D，搜索/框选复制） |
| `README.md` / `requirements.txt` / `MotorScope.spec` | 说明 / 依赖（pyside6, pylink, pyinstaller）/ 打包配置 |
| `dist\MotorScope\` | **打包产物**（onedir），拷整文件夹即可用 |

## 二、Web 版文件结构（tools\motor_scope\）

| 文件 | 职责 |
|---|---|
| `motor_scope.py` | 核心：`FocFrame`、`parse_text_line`/`parse_binary`、`RttParser`、`DataHub`（history 文件）、`JLinkRttSource`（连接/RTT，含设备回退+降速重试）、HTTP 服务（/data /health /clearlogs /clear_history） |
| `web\index.html` / `app.js` / `style.css` | 浏览器前端（Canvas 电机动画、示波器、日志面板） |
| `启动实机模式.bat` / `清空motor_scope.bat` | 启动（清旧进程）与清数据脚本 |
| `history_scope.txt` / `history_main.txt` | MOTF 帧历史 / 固件日志（2MB 上限，启动/退出清空） |
| `_verify_rtt_fields.py` / `README.md` | 字段核对脚本 / 说明 |

> 两套共用 `motor_scope.py` 的连接与解析逻辑；原生版通过 `sys.path` 导入。

## 三、MOTF 协议（固件 `ws/foc.c` 的 `Foc_RttSend`）

- 通道 0，文本模式（默认 `FOC_RTT_RATE_HZ=1000`）：
  ```
  MOTF,<mode>,<phase>,<rotor_mrad>,<theta_mrad>,<iq_ma>,<id_ma>,
       <vq_mv>,<vd_mv>,<spd_rpm>,<sync>,<diff_mrad>,<freq_cHz>,<ms>,<mech_mrad>,
       <is_ma>,<is_angle_mrad>,<v_mv>,<v_angle_mrad>,<theta_mech_mrad>,<cnt>
  ```
  共 **21 字段**；单位：角度 mrad、电流 mA、电压 mV、转速 rpm、频率 cHz、cnt 编码器原始计数。
- 二进制模式（`FOC_RTT_RATE_HZ>2000` 自动）：**76 字节**，magic `MOTF`，`parse_binary` 解析。
- 固件发送节流（防拖慢主循环）：`MOTOR_SCOPE_LOCK_MS_FAST=30ms`、保活 300ms、spd/|vd|/|vq| 阈值触发；`g_motor_scope`（Keil Watch）=0 则完全不发。

## 四、连接与错误分级（原生版）

- 默认 `--device HC32F460`、`--speed-khz 1000`；接口显式 SWD。
- `JLinkRttSource.open()` 分三阶段，**自动容错**：
  - 阶段1 `jl.open()` → `JLinkOpenError`：未找到 J-Link/USB 占用/驱动 → **琥珀色**；
  - 阶段2a `jl.connect()` → `ChipConnectError`：设备表无 HC32F460 时**自动回退 Cortex-M4**，连接失败**逐级降速 1000→400→100kHz** 重试 → **红色**；
  - 阶段2b RTT 初始化 → `RttNoDataError`：芯片已连但找不到 RTT 控制块 → **紫色**。
- 状态栏文案即前缀（J-Link 连接失败 / 芯片未连接 / RTT 无数据），颜色三态区分；连接错误期间不会被“数据中断”覆盖。
- 工具栏可自选：芯片型号、SWD 速度、RTT 通道、RAM 基址/大小、RTT 地址(0=自动)，点“应用/刷新”重连。

## 五、功能清单（原生版已全部具备）

- **电机动画**：20 磁钢 + 星标 + d/q 轴 + θ 指针 + id/iq/is/v 矢量，实时跟随 `mech_mrad`/`theta_mech_mrad`；
- **仪表**：半圆转速表 + 数值面板；
- **5 示波器**：iq/id、转子角 vs 控制角、diff、模式 comm_mode、**ABZ cnt**（自动量程，斜率=方向）；每个都有标题 + 横/纵坐标轴；
- **暂停冻结**：暂停时缓冲写入/裁剪/最新帧全停（帧计数/帧龄继续，不误报中断）；
- **回看历史跟随**：拖动时间滑条，电机/仪表/数值面板/示波器全部按 `view_frame` 时间插值回到那一刻；窗口滑条对数 0.1~20s，暂停中缩放不跳回实时；
- **日志分面板**：MOTF 帧（搜索/复制/清空）+ 固件日志 MAIN_D（搜索/框选 Ctrl+C），自动滚动；
- **历史文件**：jlink 模式写 `history_scope.txt`/`history_main.txt`，首连清空、重连不清、退出（线程停止后）清空；
- **--mock**：无硬件自测源（200Hz 默认），生成 MOTF 帧 + 模拟日志。

## 六、数据流 / 暂停回看模型

```
DataThread (J-Link RTT 或 mock)
  ├─ frame_received(FocFrame) ──> Hub.push(fr, wall_t)   // follow_live=False 时冻结缓冲
  ├─ log_line(text) ───────────> Hub.log(text) → logs_dirty → 定时器刷 LogPanel
  └─ data_error/data_status ───> 状态栏
GUI 30ms 定时器：hub.refresh_view()（实时=latest；回看=view_end 时间插值）
  → MotorView / Gauge / NumPanel 读 hub.view_frame；ScopeView 按 view_end/window_sec 取窗口
```

- 插值：对环形缓冲做二分 + 线性插值（角度/电流/电压/转速/cnt 全字段），mode 取最近采样。
- 已知坑（已修）：**Qt 角度约定**——`QPainterPath.arcTo` 正角=视觉逆时针、`cos/sin` 正角=视觉顺时针，两者混用会导致星标与磁钢反向（commit e163dc7 已统一，cos/sin 元素用 `-mech`）。

## 七、打包与运行

```powershell
# 源码运行
py main.py --mock                        # 无硬件自测
py main.py --device HC32F460 --speed-khz 400 --serial <SN>
# 打包（onedir，覆盖 dist）
py -m PyInstaller MotorScope.spec --noconfirm
# 交付：整文件夹 tools\motor_scope_native\dist\MotorScope\ 拷到目标电脑
```

## 八、Git 提交脉络（tools/motor_scope / motor_scope_native）

- web 版功能线：MOTF 帧加 cnt → history 拆分 → 日志分面板 → 回看历史跟随 → 暂停冻结缓冲 → 锁节流 → cnt 波形 → 窗口对数滑条
- 原生版（8c29285 起）：Hub 缓冲 → 各控件 → MainWindow → README/依赖 → J-Link 序列号选择+刷新 → **米色主题 + J-Link/芯片错误分级** → 芯片未连接/RTT 无数据分开 → **暂停/回看/日志分面板/窗口滑条/重连全移植** → 连接设备回退+降速 → 工具栏自选芯片/速度/RAM → 示波器标题/坐标轴 → **Qt 角度约定统一**（星标与磁钢同向）

> 未提交的本地改动仅：`template - 副本.uvoptx`（Keil 界面状态）与 `ws/motor_config.h`（用户把 MOTOR_SCOPE_LOCK_MS_FAST 15→30），打包/交接时保持不动。