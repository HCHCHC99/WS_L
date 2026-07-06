# USART3 DMA 心跳包问题记录

## 原始状态

### 配置
- USART3: PB12(RX) / PB13(TX), 921600 bps
- TX DMA: DMA2 CH0, 由 `EVT_SRC_USART3_TI` (TX empty 事件) 通过 AOS_DMA2_0 触发
- DMA TC IRQ: INT042 (DMA2 TC0)
- TX 完成处理: 在 `HW_DmaTc_ISR` 中直接关 TX

### 原始 `HW_DmaTc_ISR` 代码

```c
static void HW_DmaTc_ISR(void)
{
    USART_FuncCmd(USART3_UNIT, USART_TX, DISABLE);   // 不等移位完成, 直接关 TX
    DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, DMA_FLAG_TC_CH0);
    s_bTxBusy = false;
    if (s_pfnTxDoneCb) s_pfnTxDoneCb();
}
```

### 原始代码的行为

- **优点**: `s_bTxBusy` 能正常清零, 心跳包可以反复发送
- **缺点**: DMA TC 中断到达时 DMA 刚把最后一个字节写入 TDR, 该字节**可能还在移位寄存器中**, 立即关 TX 会导致最后一字节被截断

---

## 尝试过的修复方案（均导致 busy 卡死）

### 尝试 1: 使用 TX_CPLT 中断 + 清除 TC/TXE 标志

**思路**: 不在 DMA TC ISR 中关 TX, 改为使能 TX_CPLT 中断(`USART_INT_TX_CPLT`/TCIE), 等最后一字节移位完成后由 `HW_TxComplete_ISR` 来关 TX 并清理。

**修改**:
```c
// HW_DmaTc_ISR:
DMA_ChCmd(DISABLE);
DMA_ClearTransCompleteStatus(TC);
USART_FuncCmd(USART3, USART_INT_TX_CPLT, ENABLE);  // 使能 TCIE

// HW_TxComplete_ISR:
USART_FuncCmd(USART3, USART_TX | USART_INT_TX_CPLT, DISABLE);
USART_ClearStatus(USART3, (USART_FLAG_TX_EMPTY | USART_FLAG_TX_CPLT));
s_bTxBusy = false;
```

**结果**: ❌ 第一次心跳后 `s_bTxBusy` 永远为 1, 后续心跳全部被阻塞。

**初步推测**: HC32F460 硬件在 TE=1 时不允许修改 TCIE 位, `USART_FuncCmd(TCIE, ENABLE)` 被硬件忽略, TX_CPLT 中断永远不来。

---

### 尝试 2: TCIE 和 TE 同时使能

**思路**: 既然 TE=1 时不能改 TCIE, 那就在 `Usart3_HW_StartTxDma` 中 TE 还是 0 时同时使能 TCIE。

**修改**:
```c
// Usart3_HW_StartTxDma:
USART_FuncCmd(USART3, USART_TX | USART_INT_TX_CPLT, ENABLE);  // TE + TCIE 同时使能

// HW_DmaTc_ISR: 不再使能 TCIE (已使能)
```

**结果**: ❌ 仍然是第一次心跳后 `s_bTxBusy` 永远为 1。

**RTT 日志证据**:
```
[HW_StartTxDma] before: CR1=0xA0008024    ← TE=0, TCIE=0
[HW] DMA_TC ISR:       CR1=0xA000806C    ← TE=1, TCIE=1 ✓ (TCIE 确实被置位了!)
[HW_StartTxDma] after:  CR1=0xA000806C    ← TE=1, TCIE=1 ✓
```

TCIE **确实**被成功置位了（CR1 bit6=1），但 TX_CPLT 中断**仍然不来**。

---

### 尝试 3: 放弃 TX_CPLT 中断, 在 DMA TC ISR 中轮询 TC 标志

**思路**: 既然 TX_CPLT 中断的各种路由都有问题, 直接在 DMA TC ISR 中轮询 `USART_FLAG_TX_CPLT` 等最后一字节移位完成。

**修改**:
```c
static void HW_DmaTc_ISR(void)
{
    DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, DMA_FLAG_TC_CH0);
    // 轮询 TC 标志
    while (USART_GetStatus(USART3_UNIT, USART_FLAG_TX_CPLT) != SET) { ... }
    USART_FuncCmd(USART3_UNIT, USART_TX, DISABLE);
    USART_ClearStatus(USART3_UNIT, (USART_FLAG_TX_EMPTY | USART_FLAG_TX_CPLT));
    s_bTxBusy = false;
    if (s_pfnTxDoneCb) s_pfnTxDoneCb();
}
```

**结果**: ❌ 仍然 busy 卡死（用户反馈"还是只发了一次"）。

---

## 尝试 2 的深入分析: TCIE=1 但 TX_CPLT 中断为什么不触发？

### HC32F460 中断路由架构

通过分析 DDL 源码 `hc32_ll_interrupts.c` 和 `hc32f460_ll_interrupts_share.c`:

1. **INTC 双寄存器结构**:
   - `SEL[IRQn]` (INTSEL 寄存器): 选择中断源编号, 由 `INTC_IrqSignIn` 写入
   - `VSSEL[IRQn]` (VSSEL 寄存器): 子源使能位, 由共享分发器 `IRQ137_Handler` 读取

2. **USART3 中断路由**:
   - USART3 的 TX Complete 中断源: `INT_SRC_USART3_TCI = 291`
   - `Usart3_HW.c` 使用的 NVIC 线: `INT007_IRQn = 7`
   - 实际 USART3/4 共享的 NVIC 线: `INT137_IRQn = 137`

3. **IRQ 有效性检查**:
   - `IRQ_GRP_BASE = 32`: INTC 可重映射的 IRQ 范围是 32 及以上
   - `INT007 = 7 < 32`: 不在可重映射范围内
   - `INTC_IrqSignIn` 对 IRQ < 32 的注册**静默通过**（绕过了范围检查），但硬件可能将 SEL[7] 的写入视为无效

4. **共享分发器** `IRQ137_Handler`:
   ```c
   // 检查 VSSEL137 bit 3:
   if ((TCIE & TC) && (BIT_MASK_03 & VSSEL137)) {
       USART3_TxComplete_IrqHandler();
   }
   ```
   - `INTC_IrqSignIn` 只写了 `SEL[IRQn]`, 没写 `VSSEL[IRQn]`
   - 即使使用 IRQ137, VSSEL137 bit3 也需要单独配置

### 结论

USART3 的 TX_CPLT 中断**无法**通过 `INT007` (NVIC line 7) 触发, 因为:
- IRQ7 < 32, 是固定功能线, INTC 无法重映射
- USART3 所有中断实际通过 `INT137` 共享分发, 需要同时配置 SEL[137] 和 VSSEL137 bit3

---

## 尝试 3 失败的可能原因

虽然轮询 TC 标志不依赖 TX_CPLT 中断, 但代码中可能还有残留的 TCIE 使能逻辑或其他干扰因素。用户在尝试 2 和尝试 3 之间还有其他代码改动（`main.c` 的调试日志等）, 具体情况需要进一步隔离测试。

---

## 当前状态

**保持原始代码不变**。原始 `HW_DmaTc_ISR` 虽然可能截断最后一个字节（立即关 TX）, 但 `s_bTxBusy` 能正常清零, 心跳包可以持续发送。

### 待解决

1. USART3 TX_CPLT 中断的正确配置方式（使用 IRQ137 + VSSEL137 配置）
2. 或: 干净地实现 DMA TC ISR 中轮询 TC 方案（确保没有残留的 TCIE 使能等干扰）
