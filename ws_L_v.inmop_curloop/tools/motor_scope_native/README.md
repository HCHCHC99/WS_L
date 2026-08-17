# MotorScope 原生版（PySide6）—— Phase 1 核心

不依赖浏览器/Web 技术的原生桌面版 MotorScope：J-Link RTT 读取 + MOTF 解析 + QPainter 自绘界面（电机动画 / 5 示波器 / 转速表 / 数值面板）。

> 数据链路复用 `../motor_scope/motor_scope.py`（`JLinkRttSource`/`RttParser`/`FocFrame`），固件端 MOTF 协议零改动。

## 运行

```powershell
# 开发自测（无硬件，验证界面渲染；非仿真功能）
python main.py --mock

# 实机（默认 HC32F460 / SWD 1MHz / RTT 通道 0）
python main.py
```

### 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--mock` | 关 | 用本地生成的 MOTF 帧喂界面（仅开发自测） |
| `--mock-rate` | 200 | mock 帧率 |
| `--device` | HC32F460 | J-Link 目标 |
| `--speed-khz` | 1000 | SWD 速度 |
| `--channel` | 0 | RTT 上行通道 |
| `--rtt-addr` | 0(自动搜索) | RTT 控制块地址（找不到时在 Keil 里查 `_SEGGER_RTT` 符号地址传入） |
| `--rtt-ram-base` / `--rtt-ram-size` | 0x1FFF8000 / 0x2F000 | RAM 扫描范围（HC32F460） |

## 打包（PyInstaller onedir）

```powershell
py -m PyInstaller -D --name MotorScope --paths ..\motor_scope main.py
```

- ⚠️ `data.py` 通过 `sys.path` 引入兄弟目录的 `motor_scope.py`，PyInstaller 需用 `--paths ..\motor_scope` 让 hidden import 可解析；若仍找不到，将 `motor_scope.py` 作为数据文件或显式 hidden-import 一并打入。
- 实机模式需本机已装 SEGGER J-Link 软件（提供 `JLinkARM.dll`）。
- 生成 `dist\MotorScope\MotorScope.exe`，双击运行。

## Phase 1 范围

**已含**：J-Link 数据链路、电机动画、5 示波器（iq/id、角度、diff、mode、cnt）、转速表、数值面板、悬停读值、状态栏（连接/模式/同步/帧龄/数据中断）。

**不含（Phase 2+）**：日志面板、暂停/回看历史、重连/通道切换 UI、窗口宽度滑动条、仿真模式（`--mock` 仅开发自测）。
