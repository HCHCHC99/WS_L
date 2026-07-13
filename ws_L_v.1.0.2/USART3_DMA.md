# USART3 DMA — 根因分析与架构演进

---

## DMA 尾帧截断问题（已修复）

### 根因

DMA TC 中断触发时，最后 1~2 字节还在 USART 发送管道中：
- 倒数第 1 字节在**移位寄存器**里（正在发送）
- 倒数第 2 字节在 **TDR** 里（等待移位寄存器空闲）

ISR 立即执行 `USART_TX DISABLE` 直接掐断管道，导致尾帧字节丢失。

### 修复代码（`HW_DmaTc_ISR`）

关 TX 之前先轮询 `USART_FLAG_TX_CPLT` 等移位寄存器清空：

```c
static void HW_DmaTc_ISR(void)
{
    while (USART_GetStatus(s_stcCfg.usart_base, USART_FLAG_TX_CPLT) != SET) {
        __NOP();
    }
    USART_FuncCmd(s_stcCfg.usart_base, USART_TX, DISABLE);
    USART_ClearStatus(s_stcCfg.usart_base,
                      (USART_FLAG_TX_EMPTY | USART_FLAG_TX_CPLT));
    DMA_ChCmd(s_stcCfg.dma_base, s_stcCfg.dma_ch, DISABLE);
    DMA_ClearTransCompleteStatus(s_stcCfg.dma_base, s_stcCfg.dma_tc_flag);
    s_bTxBusy = false;
    if (s_pfnTxDoneCb) s_pfnTxDoneCb();
}
```

### 同时修复（`StartTxDma`）

发送前清除残留中断标志：

```c
DMA_ClearTransCompleteStatus(s_stcCfg.dma_base, s_stcCfg.dma_tc_flag);
NVIC_ClearPendingIRQ(s_stcCfg.dma_tc_irqn);
```

---

## 架构演进：从硬编码到可配置

### v1.0（原始）— 全部 `#define` 硬编码

```c
#define USART3_UNIT      (CM_USART3)
#define TX_DMA_UNIT      (CM_DMA2)
#define TX_DMA_CH        (DMA_CH0)
#define TX_DMA_TRIG_SEL  (AOS_DMA2_0)
#define TX_DMA_TRIG_EVT  (EVT_SRC_USART3_TI)
#define TX_DMA_TC_IRQn   (INT042_IRQn)
// ... 换 USART 或 DMA 需要改代码
```

### v2.0（当前）— `Usart3_HW_Config_t` 结构体驱动

所有硬件参数集中在一个结构体里。初始化时传入不同配置即可切换 USART/DMA/引脚，不改代码：

```c
typedef struct {
    // GPIO
    uint8_t  rx_port, rx_pin, rx_func, tx_port, tx_pin, tx_func;
    // USART
    uint32_t baudrate, fcg_periph;
    CM_USART_TypeDef *usart_base;
    // DMA
    CM_DMA_TypeDef *dma_base;
    uint8_t  dma_ch;
    uint32_t dma_fcg, aos_target, aos_event, dma_tc_flag, dma_tc_int;
    // IRQ
    IRQn_Type    rx_err_irqn, rx_full_irqn, dma_tc_irqn;
    en_int_src_t rx_err_int_src, rx_full_int_src, dma_tc_int_src;
} Usart3_HW_Config_t;
```

**使用方式**：

```c
// 默认 USART3（传 NULL）
Usart3_HW_Init(NULL);

// 自定义配置
Usart3_HW_Config_t cfg = USART3_HW_CONFIG_DEFAULT;
cfg.baudrate = 921600;
cfg.tx_pin   = GPIO_PIN_06;
Usart3_HW_Init(&cfg);
```

---

## 当前默认配置

| 项目 | 值 |
|------|-----|
| USART | USART3 |
| 引脚 | PB12=RX(FUNC_33), PB13=TX(FUNC_32) |
| 波特率 | 115200 |
| 过采样 | 8-bit |
| TX DMA | DMA2 CH0 |
| DMA 触发 | AOS_DMA2_0 ← EVT_SRC_USART3_TI |
| DMA 宽度 | 8-bit, BlockSize=1 |
| DMA TC IRQ | INT042 (DMA2 TC0) |
| RX IRQ | INT004 (EI), INT005 (RI) |

---

## 关键教训

- HC32F460 USART3 的 TX_CPLT 中断路由复杂（共享 IRQ137），不建议使用
- DMA TC ISR 中轮询 TX_CPLT 是可靠的替代方案
- **轮询必须在关 DMA 通道之前**，顺序不能错
- `USART_FLAG_TX_CPLT` 轮询后需要 `USART_ClearStatus` 清除
- 将所有硬件参数集中到 Config 结构体，换 USART/DMA/引脚只需改配置，不改逻辑代码
