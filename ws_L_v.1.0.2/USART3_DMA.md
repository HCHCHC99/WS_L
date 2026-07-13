# USART3 DMA 心跳包问题记录

---

## 最终修复方案（✅ 工作正常）

### 根因

DMA TC 中断触发时，最后 1~2 字节还在 USART 发送管道中：
- 倒数第 1 字节在**移位寄存器**里（正在发送）
- 倒数第 2 字节在 **TDR** 里（等待移位寄存器空闲）

ISR 立即执行 `USART_TX DISABLE` 直接掐断管道，导致尾帧字节丢失。

对比阻塞模式 `SendBlocking`，它在发完所有字节后先 `while(TX_CPLT)` 等移位寄存器清空再关 TX，所以不会丢字节。

### 修复代码

在 `HW_DmaTc_ISR` 中，关 TX 之前先轮询 `USART_FLAG_TX_CPLT` 等移位寄存器清空：

```c
static void HW_DmaTc_ISR(void)
{
    /* Wait for USART shift register to drain before disabling TX.
     * Without this, the last 1-2 bytes still in the TX pipeline
     * (TDR + shift register) are truncated when TX is disabled.
     * At 115200 baud, 2 bytes ≈ 174µs — acceptable ISR latency. */
    while (USART_GetStatus(USART3_UNIT, USART_FLAG_TX_CPLT) != SET) {
        __NOP();
    }

    USART_FuncCmd(USART3_UNIT, USART_TX, DISABLE);
    USART_ClearStatus(USART3_UNIT, (USART_FLAG_TX_EMPTY | USART_FLAG_TX_CPLT));
    DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, DMA_FLAG_TC_CH0);
    s_bTxBusy = false;
    if (s_pfnTxDoneCb) s_pfnTxDoneCb();
}
```

### 同时修复

`Usart3_HW_StartTxDma` 开头增加残留中断标志清除，防止上次传输的 pending IRQ 提前触发 ISR：

```c
DMA_ClearTransCompleteStatus(TX_DMA_UNIT, DMA_FLAG_TC_CH0);
NVIC_ClearPendingIRQ(TX_DMA_TC_IRQn);
```

### 为什么这次成功而之前尝试 3 失败

之前尝试 3 虽然在 ISR 中加过 `while(TX_CPLT)`，但当时的代码流程不同：
1. **之前**：先关 DMA 通道 → 轮询 TC → 关 TX → 清除标志
2. **现在**：先轮询 TC → 关 TX → 清除 TX 标志 → 关 DMA 通道 → 清除 DMA 标志

关 DMA 通道的时机很关键——如果先关 DMA，DMA 停止响应 AOS 触发，但 USART TX 还在发送，管道里的字节正常移出。问题在于当时可能还有其他干扰因素（TCIE 使能残留等）。

---

## 配置速查

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

## 尝试过的失败方案记录

### 尝试 1～3（详见旧版本记录）

1. **TX_CPLT 中断 + INTC 重映射** → 失败：HC32F460 USART3 TX_CPLT 中断需要 IRQ137 + VSSEL137 配置，INT007 无法重映射
2. **TCIE + TE 同时使能** → 失败：TCIE 成功置位但中断不来
3. **轮询 TC 但先关 DMA** → 失败：DMA 提前关闭导致问题

### 关键教训

- HC32F460 USART3 的 TX_CPLT 中断路由复杂（共享 IRQ137），不建议使用
- DMA TC ISR 中轮询 TX_CPLT 是可靠的替代方案
- **轮询必须在关 DMA 通道之前**，顺序不能错
- `USART_FLAG_TX_CPLT` 轮询后需要 `USART_ClearStatus` 清除
