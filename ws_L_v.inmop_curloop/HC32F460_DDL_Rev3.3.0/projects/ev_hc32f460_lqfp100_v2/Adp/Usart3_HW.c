/**
 *******************************************************************************
 * @file  Usart3_HW.c
 * @brief Generic UART + DMA hardware abstraction layer implementation.
 *
 * All hardware-specific parameters come from Usart3_HW_Config_t.
 * No #define for USART/DMA/IRQ — single source of truth is the config struct.
 *******************************************************************************
 */

#include "Usart3_HW.h"
#include "rtt_manager.h"
#include <string.h>

/*=============================================================================
 * Default configs
 *============================================================================*/

/** USART3, PB12(RX)=FUNC_33, PB13(TX)=FUNC_32, DMA2 CH0, 115200 */
const Usart3_HW_Config_t USART3_HW_CONFIG_DEFAULT = {
    /* GPIO */
    .rx_port    = GPIO_PORT_B,
    .rx_pin     = GPIO_PIN_12,
    .rx_func    = GPIO_FUNC_33,
    .tx_port    = GPIO_PORT_B,
    .tx_pin     = GPIO_PIN_13,
    .tx_func    = GPIO_FUNC_32,
    /* USART */
    .baudrate   = 115200UL,
    .fcg_periph = FCG1_PERIPH_USART3,
    .usart_base = CM_USART3,
    /* DMA */
    .dma_base   = CM_DMA2,
    .dma_ch     = DMA_CH0,
    .dma_fcg    = FCG0_PERIPH_DMA2,
    .aos_target = AOS_DMA2_0,
    .aos_event  = EVT_SRC_USART3_TI,
    .dma_tc_flag = DMA_FLAG_TC_CH0,
    .dma_tc_int  = DMA_INT_TC_CH0,
    /* IRQ */
    .rx_err_irqn    = INT004_IRQn,
    .rx_err_int_src = (en_int_src_t)INT_SRC_USART3_EI,
    .rx_full_irqn   = INT005_IRQn,
    .rx_full_int_src = (en_int_src_t)INT_SRC_USART3_RI,
    .dma_tc_irqn    = INT042_IRQn,
    .dma_tc_int_src = (en_int_src_t)INT_SRC_DMA2_TC0,
};

/*=============================================================================
 * Other constants
 *============================================================================*/

/* Peripheral register unlock mask */
#define LL_PERIPH_SEL  (LL_PERIPH_GPIO | LL_PERIPH_FCG | \
                        LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_EFM | LL_PERIPH_SRAM)

/* Max single DMA transfer */
#define TX_DMA_LEN_MAX  (256U)

/*=============================================================================
 * Static — config & state
 *============================================================================*/
static Usart3_HW_Config_t   s_stcCfg;
static bool                 s_bInitialized;
static volatile bool        s_bTxBusy;

/*=============================================================================
 * Static — callbacks
 *============================================================================*/
static Usart3_HW_RxCallback_t      s_pfnRxCb;
static Usart3_HW_TxDoneCallback_t  s_pfnTxDoneCb;
static Usart3_HW_ErrorCallback_t   s_pfnErrCb;

/*=============================================================================
 * Forward declarations
 *============================================================================*/
static void HW_RxError_ISR(void);
static void HW_RxFull_ISR(void);
static void HW_DmaTc_ISR(void);

/*=============================================================================
 * Helpers
 *============================================================================*/

static void INTC_Install(const stc_irq_signin_config_t *cfg, uint32_t prio)
{
    if (cfg != NULL) {
        (void)INTC_IrqSignIn(cfg);
        NVIC_ClearPendingIRQ(cfg->enIRQn);
        NVIC_SetPriority(cfg->enIRQn, prio);
        NVIC_EnableIRQ(cfg->enIRQn);
    }
}

static void HW_ApplyConfig(const Usart3_HW_Config_t *cfg)
{
    if (cfg == NULL) {
        (void)memcpy(&s_stcCfg, &USART3_HW_CONFIG_DEFAULT,
                     sizeof(Usart3_HW_Config_t));
    } else {
        (void)memcpy(&s_stcCfg, cfg, sizeof(Usart3_HW_Config_t));
    }
}

static uint32_t HW_GetClkDiv(uint32_t baudrate)
{
    if (baudrate <= 4800UL)       return USART_CLK_DIV64;
    else if (baudrate <= 38400UL) return USART_CLK_DIV16;
    else if (baudrate <= 115200UL) return USART_CLK_DIV4;
    else                           return USART_CLK_DIV1;
}

/*=============================================================================
 * ISR callbacks
 *============================================================================*/

static void HW_RxError_ISR(void)
{
    (void)USART_ReadData(s_stcCfg.usart_base);
    USART_ClearStatus(s_stcCfg.usart_base,
        (USART_FLAG_OVERRUN | USART_FLAG_FRAME_ERR | USART_FLAG_PARITY_ERR));
    if (s_pfnErrCb) s_pfnErrCb();
}

static void HW_RxFull_ISR(void)
{
    uint8_t ch = (uint8_t)USART_ReadData(s_stcCfg.usart_base);
    if (s_pfnRxCb) s_pfnRxCb(ch);
}

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

/*=============================================================================
 * Public API — Init / DeInit
 *============================================================================*/

void Usart3_HW_Init(const Usart3_HW_Config_t *cfg)
{
    stc_usart_uart_init_t   stcUart;
    stc_irq_signin_config_t stcIrq;

    if (s_bInitialized) return;

    HW_ApplyConfig(cfg);
    s_pfnRxCb     = NULL;
    s_pfnTxDoneCb = NULL;
    s_pfnErrCb    = NULL;
    s_bTxBusy     = false;

    LL_PERIPH_WE(LL_PERIPH_SEL);

    /* GPIO pin mux */
    GPIO_SetFunc(s_stcCfg.rx_port, s_stcCfg.rx_pin, s_stcCfg.rx_func);
    GPIO_SetFunc(s_stcCfg.tx_port, s_stcCfg.tx_pin, s_stcCfg.tx_func);

    /* USART clock + init */
    FCG_Fcg1PeriphClockCmd(s_stcCfg.fcg_periph, ENABLE);
    (void)USART_UART_StructInit(&stcUart);
    stcUart.u32ClockDiv      = HW_GetClkDiv(s_stcCfg.baudrate);
    stcUart.u32Baudrate      = s_stcCfg.baudrate;
    stcUart.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    (void)USART_UART_Init(s_stcCfg.usart_base, &stcUart, NULL);

    /* RX IRQs */
    stcIrq.enIRQn      = s_stcCfg.rx_err_irqn;
    stcIrq.enIntSrc    = s_stcCfg.rx_err_int_src;
    stcIrq.pfnCallback = HW_RxError_ISR;
    INTC_Install(&stcIrq, DDL_IRQ_PRIO_DEFAULT);

    stcIrq.enIRQn      = s_stcCfg.rx_full_irqn;
    stcIrq.enIntSrc    = s_stcCfg.rx_full_int_src;
    stcIrq.pfnCallback = HW_RxFull_ISR;
    INTC_Install(&stcIrq, DDL_IRQ_PRIO_DEFAULT);

    /* TX DMA */
    {
        stc_dma_init_t stcDma;

        FCG_Fcg0PeriphClockCmd(s_stcCfg.dma_fcg, ENABLE);
        FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

        (void)DMA_StructInit(&stcDma);
        stcDma.u32IntEn       = DMA_INT_ENABLE;
        stcDma.u32BlockSize   = 1UL;
        stcDma.u32TransCount  = 0UL;
        stcDma.u32DataWidth   = DMA_DATAWIDTH_8BIT;
        stcDma.u32SrcAddr     = 0UL;
        stcDma.u32DestAddr    = (uint32_t)(&s_stcCfg.usart_base->TDR);
        stcDma.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
        stcDma.u32DestAddrInc = DMA_DEST_ADDR_FIX;
        (void)DMA_Init(s_stcCfg.dma_base, s_stcCfg.dma_ch, &stcDma);

        AOS_SetTriggerEventSrc(s_stcCfg.aos_target, s_stcCfg.aos_event);

        stcIrq.enIRQn      = s_stcCfg.dma_tc_irqn;
        stcIrq.enIntSrc    = s_stcCfg.dma_tc_int_src;
        stcIrq.pfnCallback = HW_DmaTc_ISR;
        INTC_Install(&stcIrq, DDL_IRQ_PRIO_DEFAULT);

        DMA_Cmd(s_stcCfg.dma_base, ENABLE);
        DMA_TransCompleteIntCmd(s_stcCfg.dma_base, s_stcCfg.dma_tc_int, ENABLE);
    }

    s_bInitialized = true;
    MAIN_D("[Uart_HW] Init OK: baud=%lu\r\n", s_stcCfg.baudrate);
}

void Usart3_HW_DeInit(void)
{
    if (!s_bInitialized) return;
    USART_FuncCmd(s_stcCfg.usart_base,
        (USART_RX | USART_TX | USART_INT_RX |
         USART_INT_TX_EMPTY | USART_INT_TX_CPLT), DISABLE);
    NVIC_DisableIRQ(s_stcCfg.rx_err_irqn);
    NVIC_DisableIRQ(s_stcCfg.rx_full_irqn);
    DMA_ChCmd(s_stcCfg.dma_base, s_stcCfg.dma_ch, DISABLE);
    DMA_TransCompleteIntCmd(s_stcCfg.dma_base, s_stcCfg.dma_tc_int, DISABLE);
    s_bInitialized = false;
}

/*=============================================================================
 * Public API — RX
 *============================================================================*/

void Usart3_HW_StartRx(void)
{
    USART_FuncCmd(s_stcCfg.usart_base, USART_RX | USART_INT_RX, ENABLE);
}

void Usart3_HW_StopRx(void)
{
    USART_FuncCmd(s_stcCfg.usart_base, USART_RX | USART_INT_RX, DISABLE);
}

/*=============================================================================
 * Public API — TX (DMA)
 *============================================================================*/

bool Usart3_HW_StartTxDma(const uint8_t *data, uint16_t len)
{
    if (!s_bInitialized || data == NULL || len == 0 || len > TX_DMA_LEN_MAX) {
        return false;
    }
    if (s_bTxBusy) return false;

    DMA_ClearTransCompleteStatus(s_stcCfg.dma_base, s_stcCfg.dma_tc_flag);
    NVIC_ClearPendingIRQ(s_stcCfg.dma_tc_irqn);

    DMA_SetSrcAddr(s_stcCfg.dma_base, s_stcCfg.dma_ch, (uint32_t)data);
    DMA_SetTransCount(s_stcCfg.dma_base, s_stcCfg.dma_ch, len);

    s_bTxBusy = true;
    (void)DMA_ChCmd(s_stcCfg.dma_base, s_stcCfg.dma_ch, ENABLE);
    USART_FuncCmd(s_stcCfg.usart_base, USART_TX, ENABLE);
    return true;
}

bool Usart3_HW_IsTxBusy(void)
{
    return s_bTxBusy;
}

/*=============================================================================
 * Public API — TX (blocking polling)
 *============================================================================*/

void Usart3_HW_SendBlocking(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    if (!s_bInitialized || data == NULL || len == 0) return;

    DMA_ChCmd(s_stcCfg.dma_base, s_stcCfg.dma_ch, DISABLE);
    DMA_TransCompleteIntCmd(s_stcCfg.dma_base, s_stcCfg.dma_tc_int, DISABLE);

    USART_FuncCmd(s_stcCfg.usart_base,
        (USART_TX | USART_INT_TX_EMPTY | USART_INT_TX_CPLT), DISABLE);
    USART_FuncCmd(s_stcCfg.usart_base, USART_TX, ENABLE);

    for (i = 0; i < len; i++) {
        while (USART_GetStatus(s_stcCfg.usart_base, USART_FLAG_TX_EMPTY) != SET) {
            __NOP();
        }
        USART_WriteData(s_stcCfg.usart_base, data[i]);
    }
    while (USART_GetStatus(s_stcCfg.usart_base, USART_FLAG_TX_CPLT) != SET) {
        __NOP();
    }

    USART_FuncCmd(s_stcCfg.usart_base, USART_TX, DISABLE);
    USART_ClearStatus(s_stcCfg.usart_base,
                      (USART_FLAG_TX_EMPTY | USART_FLAG_TX_CPLT));

    DMA_TransCompleteIntCmd(s_stcCfg.dma_base, s_stcCfg.dma_tc_int, ENABLE);
    s_bTxBusy = false;
}

/*=============================================================================
 * Public API — Callback registration
 *============================================================================*/

void Usart3_HW_RegisterRxCallback(Usart3_HW_RxCallback_t cb)
    { s_pfnRxCb = cb; }
void Usart3_HW_RegisterTxDoneCallback(Usart3_HW_TxDoneCallback_t cb)
    { s_pfnTxDoneCb = cb; }
void Usart3_HW_RegisterErrorCallback(Usart3_HW_ErrorCallback_t cb)
    { s_pfnErrCb = cb; }
