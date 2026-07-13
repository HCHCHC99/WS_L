# USART3 + DMA + VOFA+ 配置说明

---

## 1. 架构总览

```
┌──────────────────────────────────────────────────────────┐
│ main.c                                                   │
│   Vofa_Init()  /  TestVofa_Run()  /  Runner_Run()        │
├──────────────────────────────────────────────────────────┤
│ test_Vofa.c           (测试信号, VOFA_TEST_ENABLE 宏开关) │
│ Usart3_Vofa_Runner.c  (心跳 + RX 日志轮询)               │
├──────────────────────────────────────────────────────────┤
│ Usart3_Vofa.c         (统一 VOFA+ 入口)                   │
│   JustFloat(SendFloats/SendScaled) + FireWater(Printf)   │
│   + 命令解析(ReadCmd/ReadLine) + Vofa.c 桥接回调          │
├──────────────────────────────────────────────────────────┤
│ Usart3_IO.c           (协议无关 IO 层)                    │
│   环型缓冲 RX + DMA-safe TX 拷贝 + busy 标志              │
├──────────────────────────────────────────────────────────┤
│ Usart3_HW.c           (可配置硬件抽象)                     │
│   所有 USART/DMA/IRQ 参数由 Usart3_HW_Config_t 结构体驱动  │
├──────────────────────────────────────────────────────────┤
│ HC32F460 硬件                                            │
└──────────────────────────────────────────────────────────┘
```

**层级依赖**：每层只调下一层，不跨层。换 USART/DMA/引脚只需改 Config 结构体。

---

## 2. 硬件配置

| 项目 | 值 |
|------|-----|
| MCU | HC32F460JETA (Cortex-M4F, 168MHz) |
| USART | USART3 (可通过 Config 结构体切换) |
| TX 引脚 | PB13, FUNC_32 |
| RX 引脚 | PB12, FUNC_33 |
| 波特率 | 115200 |
| 数据位 | 8 / 停止位 1 / 无校验 |
| 过采样 | 8-bit / 时钟 DIV4 |

---

## 3. DMA 配置

| 项目 | 值 |
|------|-----|
| DMA | DMA2 CH0 (可通过 Config 切换) |
| 触发 | AOS_DMA2_0 ← EVT_SRC_USART3_TI |
| 宽度 | 8-bit, BlockSize=1 |
| TC IRQ | INT042 (DMA2 TC0) |
| TC ISR | 先 `while(TX_CPLT)` 等移位寄存器空 → 再关 TX |

详见 `USART3_DMA.md`。

---

## 4. 中断

| IRQ | 源 | 用途 |
|-----|-----|------|
| INT004 | USART3_EI | RX 错误 |
| INT005 | USART3_RI | RX 逐字节 → ring_buf |
| INT042 | DMA2_TC0 | DMA TX 完成 → 回调 |

---

## 5. 代码文件

```
Adp/
├── Usart3_HW.h/.c          可配置硬件抽象 (Config 结构体驱动)
├── Usart3_IO.h/.c          协议无关 IO (ring_buf + DMA 拷贝)
├── Usart3_Vofa.h/.c        VOFA+ 统一入口 + Vofa.c 桥接回调
└── Usart3_Vofa_Runner.h/.c 心跳定时 + RX 日志轮询

App/
└── test_Vofa.h/.c          测试信号 (VOFA_TEST_ENABLE 宏开关)

template/source/
└── main.c                  应用入口

Utils/
├── ring_buf.h/.c           环型缓冲区
├── TickTimer.h/.c          1ms 滴答 + NonBlockingDelay
└── rtt_manager.h/.c        Segger RTT 日志

RTT/
├── rtt_log.h               MAIN_D 等日志宏
├── SEGGER_RTT.c            Segger RTT 驱动
└── SEGGER_RTT_printf.c
```

---

## 6. VOFA+ 协议

### JustFloat（高频数据，推荐）

```
帧格式: [float32 LE × N] + [0x00, 0x00, 0x80, 0x7F]
尾帧:   0x7F800000 (IEEE754 +Infinity)
最大通道: 16 (受 TX buffer 256B 限制)
发送:    DMA 非阻塞
数据约定: MCU 用 int32_t×1000，发前 ×0.001f 转 float
```

### FireWater（调试文本）

```
帧格式: CSV 字符串 + \r\n
实现:    vsnprintf → DMA
场景:    低频调试
```

### 命令帧（VOFA+ → MCU）

```
帧格式: 任意数据 + {0xAF, 0xFA} 尾帧
解析:    Usart3_Vofa_ReadCmd() 逐字节扫描
日志:    MAIN_D("[VOFA RX] ...") → Segger RTT
```

---

## 7. 换硬件指南

### 换引脚（不改代码）

```c
Usart3_HW_Config_t cfg = USART3_HW_CONFIG_DEFAULT;
cfg.tx_pin  = GPIO_PIN_06;
cfg.rx_pin  = GPIO_PIN_07;
Usart3_Vofa_Init(&cfg);
```

### 换 USART + DMA（查手册填结构体）

```c
Usart3_HW_Config_t cfg = {
    .rx_port = GPIO_PORT_A, .rx_pin = GPIO_PIN_10,
    .rx_func = GPIO_FUNC_xx,   // 查 HC32F460 手册
    .tx_port = GPIO_PORT_A, .tx_pin = GPIO_PIN_09,
    .tx_func = GPIO_FUNC_xx,
    .baudrate = 115200,
    .fcg_periph = FCG1_PERIPH_USART1,
    .usart_base = CM_USART1,
    .dma_base = CM_DMA2, .dma_ch = DMA_CH_x,
    .dma_fcg = FCG0_PERIPH_DMA2,
    .aos_target = AOS_DMA2_x, .aos_event = EVT_SRC_USART1_TI,
    .dma_tc_flag = DMA_FLAG_TC_CHx, .dma_tc_int = DMA_INT_TC_CHx,
    // IRQ 查手册
};
Usart3_Vofa_Init(&cfg);
```

**上层 IO/Vofa/Runner 代码完全不用改。**

---

## 8. API 速查

### 发送 JustFloat

```c
// int32 定点（推荐）
int32_t buf[] = {ia_mA, ib_mA, ic_mA, vbus_mV, speed_rpm};
Usart3_Vofa_SendScaled(buf, 5, 0.001f);       // mA→A, mV→V

// 直接 float
float fbuf[] = {1.5f, 2.3f, 0.8f};
Usart3_Vofa_SendFloats(fbuf, 3);
```

### 发送 FireWater

```c
Usart3_Vofa_Printf("fault=%d, temp=%d\r\n", fault, temp);
```

### 通过官方 Vofa.c

```c
Vofa_HandleTypedef vofa1;
Vofa_Init(&vofa1, VOFA_MODE_SKIP);
Vofa_JustFloat(&vofa1, floats, 5);    // 底层走 DMA
Vofa_Printf(&vofa1, "spd=%d\r\n", s);
```

### 接收命令

```c
// 从 ring_buf 直接读
uint8_t cmd[32];
uint16_t n = Usart3_Vofa_ReadCmd(cmd, sizeof(cmd));

// 或通过官方 Vofa FIFO
Usart3_Vofa_FeedRx(&vofa1);
n = Vofa_ReadCmd(&vofa1, cmd, sizeof(cmd));
```

### 注册心跳数据 Provider

```c
void MyProvider(int32_t *buf, uint8_t *count) {
    buf[0] = g_i_u_mA; buf[1] = g_vbus_mV; *count = 2;
}
Usart3_Vofa_Runner_SetDataProvider(MyProvider);
```

---

## 9. 测试信号

`test_Vofa.h` 中 `VOFA_TEST_ENABLE` 控制：

| 宏值 | 效果 |
|:---:|------|
| 1 | 每 400ms 发 CH1(锯齿波 0→100) + CH2(计数器 1→200) |
| 0 | 测试信号关闭，不影响心跳和 RX |

```c
#include "test_Vofa.h"
#if VOFA_TEST_ENABLE
TestVofa_Init();
// while(1) { TestVofa_Run(); }
#endif
```

---

## 10. VOFA+ 上位机操作

1. 协议: **JustFloat**
2. 串口: **115200-8-N-1**
3. 打开串口 → 左上角 JustFloat 节点 → 拖 CH1/CH2 到波形图
4. 右键波形图 → Y 轴调整范围

### 常见问题

| 问题 | 解决 |
|------|------|
| JustFloat 无数据 | RawData 确认 hex 尾帧 `00 00 80 7F` |
| 通道恒为 0 | 检查 Y 轴范围、波形图运行状态 |
| FireWater/JustFloat 混用 | 手动选协议，不要用自动检测 |
