# USART3 + DMA + VOFA+ 配置说明

---

## 1. 架构总览

```
┌─────────────────────────────────────────────────────┐
│ main.c                                              │
│   测试信号: CH1 锯齿波 0→100, CH2 计数器 1→200       │
│   Vofa_JustFloat() / Usart3_Vofa_SendScaled()       │
├─────────────────────────────────────────────────────┤
│ Usart3_Vofa_Runner.c  (心跳 + RX 日志 + 测试信号)     │
│ Usart3_Vofa.c         (FireWater / JustFloat / CMD) │
│ Vofa_Bridge.c         (官方 Vofa.c 桥接)             │
├─────────────────────────────────────────────────────┤
│ Usart3_IO.c           (环型缓冲 / DMA 拷贝 / JustFloat)│
├─────────────────────────────────────────────────────┤
│ Usart3_HW.c           (GPIO / USART / DMA2 CH0 / AOS)│
├─────────────────────────────────────────────────────┤
│ HC32F460 硬件                                       │
└─────────────────────────────────────────────────────┘
```

---

## 2. 硬件配置

| 项目 | 值 |
|------|-----|
| MCU | HC32F460JETA (Cortex-M4F, 168MHz) |
| USART | USART3 |
| TX 引脚 | PB13, FUNC_32 |
| RX 引脚 | PB12, FUNC_33 |
| 波特率 | **115200** |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |
| 过采样 | 8-bit |
| 时钟分频 | DIV4 (≤115200 自动选择) |

---

## 3. DMA 配置

| 项目 | 值 |
|------|-----|
| DMA 单元 | DMA2 |
| DMA 通道 | CH0 |
| 触发源 | AOS_DMA2_0 ← EVT_SRC_USART3_TI |
| 数据宽度 | 8-bit |
| BlockSize | 1 |
| TransCount | 动态设置 (每帧帧长) |
| 源地址递增 | 是 (SRC_ADDR_INC) |
| 目标地址递增 | 否 (DEST_ADDR_FIX, → USART3->TDR) |
| TC 中断 | INT042 (DMA2 TC0) |
| TC ISR | 轮询 TX_CPLT 后再关 TX（防尾字节截断） |

---

## 4. 中断配置

| IRQ | 中断源 | 用途 |
|-----|--------|------|
| INT004 | USART3_EI | RX 错误 |
| INT005 | USART3_RI | RX 接收完成（逐字节回调） |
| INT042 | DMA2_TC0 | DMA TX 传输完成 |

---

## 5. VOFA+ 协议

### JustFloat（高频数据）

- 帧格式: `[float32 LE × N] + [0x00, 0x00, 0x80, 0x7F]`
- 尾帧: `{0x00, 0x00, 0x80, 0x7F}` (IEEE754 +Infinity)
- 最大通道数: 16 (受 TX buffer 256 字节限制，实际最大 63)
- 发送方式: **DMA 非阻塞**
- 数据约定: MCU 用 `int32_t × 1000` 存储，发送前 `×0.001f` 转 float

### FireWater（文本调试）

- 帧格式: CSV 字符串 + `\r\n`
- 实现: `vsnprintf` → DMA 发送
- 场景: 低频调试打印

### 命令帧（VOFA+ → MCU）

- 帧格式: 任意数据 + `{0xAF, 0xFA}` 尾帧
- 解析: `Usart3_Vofa_ReadCmd()` 逐字节扫描
- 日志: `MAIN_D("[VOFA RX] ...")` 打印到 Segger RTT

---

## 6. 代码文件清单

```
Adp/
├── Usart3_HW.h          硬件抽象层头文件
├── Usart3_HW.c          硬件抽象层实现（GPIO/USART/DMA/AOS/IRQ）
├── Usart3_IO.h          IO 层头文件（环型缓冲 + DMA 拷贝）
├── Usart3_IO.c          IO 层实现
├── Usart3_Vofa.h        协议层头文件（JustFloat/FireWater/CMD）
├── Usart3_Vofa.c        协议层实现
├── Usart3_Vofa_Runner.h 心跳 + RX 日志 runner
├── Usart3_Vofa_Runner.c 心跳发送 + RX 打印 + 测试信号生成
├── Vofa_Bridge.h        官方 Vofa.c 桥接头文件
└── Vofa_Bridge.c        官方 Vofa.c 桥接实现（TX→DMA, RX→ring_buf）

template/source/
└── main.c               应用入口，初始化 + 主循环

Utils/
├── ring_buf.h           环型缓冲区
├── ring_buf.c
├── TickTimer.h          1ms 滴答定时器 + NonBlockingDelay
├── TickTimer.c
├── rtt_manager.h        RTT 调试日志
└── rtt_manager.c

RTT/
├── rtt_log.h            MAIN_D / MAIN_I 等日志宏
├── SEGGER_RTT.c         Segger RTT 驱动
└── SEGGER_RTT_printf.c
```

---

## 7. 测试信号

每 400ms 发送一帧 2 通道 JustFloat：

| 通道 | 行为 | 间隔 | 范围 |
|------|------|------|------|
| CH1 | 锯齿波 +0.5 | 400ms | 0 → 100 → 归零 |
| CH2 | 计数 +1 | 500ms | 1 → 200 → 归 1 |

VOFA+ 设置:
1. 协议: **JustFloat**
2. 端口: 对应串口, **115200-8-N-1**
3. 拖 CH1/CH2 到波形图

---

## 8. API 速查

### 发送 JustFloat（int32 定点，推荐）

```c
int32_t buf[] = {ia_mA, ib_mA, ic_mA, vbus_mV, speed_rpm};
Usart3_Vofa_SendScaled(buf, 5, 0.001f);  // mA→A, mV→V
```

### 发送 FireWater（调试文本）

```c
Usart3_Vofa_Printf("fault=%d, temp=%d\r\n", fault, temp);
// 或通过官方 Vofa API:
Vofa_Printf(&vofa1, "speed=%d\r\n", speed);
```

### 接收命令

```c
uint8_t cmd[32];
uint16_t n = Usart3_Vofa_ReadCmd(cmd, sizeof(cmd));
if (n >= 4 && cmd[n-2] == 0xAF && cmd[n-1] == 0xFA) {
    // 完整命令帧，cmd[0..n-3] 是数据
}
```

### 注册数据 Provider（心跳 JustFloat）

```c
void MyProvider(int32_t *buf, uint8_t *count) {
    buf[0] = g_i_u_mA;
    buf[1] = g_vbus_mV;
    *count = 2;
}
Usart3_Vofa_Runner_SetDataProvider(MyProvider);
```

---

## 9. VOFA+ 上位机操作

1. 打开 VOFA+，协议选 **JustFloat**
2. 串口设置: **115200, 8, 无校验, 1 停止位**
3. 打开串口 → 左上角出现 JustFloat 节点 → 展开看到 CH1, CH2...
4. 拖通道名到右侧波形图控件
5. 右键波形图 → Y 轴 → 调整范围（如 CH1: 0~100）

### 常见问题

| 问题 | 解决 |
|------|------|
| JustFloat 无数据显示 | 选 RawData 确认 hex 尾帧 `00 00 80 7F` 存在 |
| 通道值恒为 0 | 检查 Y 轴范围、确认波形图已点运行 |
| 数据拆分多行 | 帧间隔空闲超时 10ms，正常 |
| FireWater 有数据但 JustFloat 没有 | 检查波特率、确认协议手动选 JustFloat |

---

## 10. 已知问题

- **DMA TC ISR 轮询 TX_CPLT**: 在 ISR 中轮询约 87~174µs，对 168MHz MCU 影响可忽略。如果未来需要极致中断延迟，可考虑用 USART TC 中断（需要正确配置 IRQ137 + VSSEL137）
- **FireWater 与 JustFloat 混用**: 同一个串口上交替发文本和二进制会混淆 VOFA+ 的协议自动检测，建议手动选择协议
