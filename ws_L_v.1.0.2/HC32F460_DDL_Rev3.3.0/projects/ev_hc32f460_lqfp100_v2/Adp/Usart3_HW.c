/**
 *******************************************************************************
 * @file  Usart3_HW.c
 * @brief USART3 hardware abstraction layer implementation.
 *
 * Pin-to-function mapping (verified against HC32F460 datasheet):
 *   PB12  FUNC_33 → USART3_RX
 *   PB13  FUNC_32 → USART3_TX
 *
 * DMA:  DMA2 CH0, triggered by USART3_TI via AOS_DMA2_0
 * IRQ:  INT004 (RX err), INT005 (RX full), INT042 (DMA2 TC0)
 *******************************************************************************
 */

#include "Usart3_HW.h"
#include "rtt_manager.h"
#include <string.h>

/*=============================================================================
 * Defaults
 *============================================================================*/
#define USART3_HW_DEF_RX_PORT       (GPIO_PORT_B)
#define USART3_HW_DEF_RX_PIN        (GPIO_PIN_12)
#define USART3_HW_DEF_RX_FUNC       (GPIO_FUNC_33)
#define USART3_HW_DEF_TX_PORT       (GPIO_PORT_B)
#define USART3_HW_DEF_TX_PIN        (GPIO_PIN_13)
#define USART3_HW_DEF_TX_FUNC       (GPIO_FUNC_32)
#define USART3_HW_DEF_BAUDRATE      (921600UL)

/* Peripheral register unlock mask */
#define LL_PERIPH_SEL               (LL_PERIPH_GPIO | LL_PERIPH_FCG | \
                                     LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_EFM | LL_PERIPH_SRAM)

/* USART3 base */
#define USART3_UNIT                 (CM_USART3)

/* DMA */
#define TX_DMA_UNIT                 (CM_DMA2)
#define TX_DMA_CH                   (DMA_CH0)
#define TX_DMA_TRIG_SEL             (AOS_DMA2_0)
#define TX_DMA_TRIG_EVT             (EVT_SRC_USART3_TI)
#define TX_DMA_TC_IRQn              (INT042_IRQn)   /* DMA2 CH0 TC = INT042 */
#define TX_DMA_TC_INT_SRC           (INT_SRC_DMA2_TC0)

/* USART IRQ */
#define RX_ERR_IRQn                 (INT004_IRQn)
#define RX_ERR_INT_SRC              (INT_SRC_USART3_EI)
#define RX_FULL_IRQn                (INT005_IRQn)
#define RX_FULL_INT_SRC             (INT_SRC_USART3_RI)

/* Max single DMA transfer */
#define TX_DMA_LEN_MAX              (256U)

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
 * Forward declarations (ISR handlers)
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

static void HW_ApplyDefaults(const Usart3_HW_Config_t *cfg)
{
    if (cfg == NULL) {
        s_stcCfg.baudrate = USART3_HW_DEF_BAUDRATE;
        s_stcCfg.rx_port  = USART3_HW_DEF_RX_PORT;
        s_stcCfg.rx_pin   = USART3_HW_DEF_RX_PIN;
        s_stcCfg.rx_func  = USART3_HW_DEF_RX_FUNC;
        s_stcCfg.tx_port  = USART3_HW_DEF_TX_PORT;
        s_stcCfg.tx_pin   = USART3_HW_DEF_TX_PIN;
        s_stcCfg.tx_func  = USART3_HW_DEF_TX_FUNC;
    } else {
        memcpy(&s_stcCfg, cfg, sizeof(Usart3_HW_Config_t));
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
 * ISR callbacks — delegate to registered user callbacks
 *============================================================================*/
static void HW_RxError_ISR(void)
{
    (void)USART_ReadData(USART3_UNIT);
    USART_ClearStatus(USART3_UNIT,
        (USART_FLAG_OVERRUN | USART_FLAG_FRAME_ERR | USART_FLAG_PARITY_ERR));
    if (s_pfnErrCb) s_pfnErrCb();
}

static void HW_RxFull_ISR(void)
{
    uint8_t ch = (uint8_t)USART_ReadData(USART3_UNIT);
    if (s_pfnRxCb) s_pfnRxCb(ch);
}

static void HW_DmaTc_ISR(void)
{
    USART_FuncCmd(USART3_UNIT, USART_TX, DISABLE);
    DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    DMA_ClearTransCompleteStatus(TX_DMA_UNIT, DMA_FLAG_TC_CH0);
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

    HW_ApplyDefaults(cfg);
    s_pfnRxCb    = NULL;
    s_pfnTxDoneCb = NULL;
    s_pfnErrCb   = NULL;
    s_bTxBusy    = false;

    LL_PERIPH_WE(LL_PERIPH_SEL);

    /* GPIO */
    GPIO_SetFunc(s_stcCfg.rx_port, s_stcCfg.rx_pin, s_stcCfg.rx_func);
    GPIO_SetFunc(s_stcCfg.tx_port, s_stcCfg.tx_pin, s_stcCfg.tx_func);

    /* USART clock + init */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART3, ENABLE);
    (void)USART_UART_StructInit(&stcUart);
    stcUart.u32ClockDiv      = HW_GetClkDiv(s_stcCfg.baudrate);
    stcUart.u32Baudrate      = s_stcCfg.baudrate;
    stcUart.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    (void)USART_UART_Init(USART3_UNIT, &stcUart, NULL);

    /* RX IRQ */
    stcIrq.enIRQn      = RX_ERR_IRQn;
    stcIrq.enIntSrc    = (en_int_src_t)RX_ERR_INT_SRC;
    stcIrq.pfnCallback = HW_RxError_ISR;
    INTC_Install(&stcIrq, DDL_IRQ_PRIO_DEFAULT);

    stcIrq.enIRQn      = RX_FULL_IRQn;
    stcIrq.enIntSrc    = (en_int_src_t)RX_FULL_INT_SRC;
    stcIrq.pfnCallback = HW_RxFull_ISR;
    INTC_Install(&stcIrq, DDL_IRQ_PRIO_DEFAULT);

    /* TX DMA */
    {
        stc_dma_init_t stcDma;
        FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_DMA2, ENABLE);
        FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS,  ENABLE);

        (void)DMA_StructInit(&stcDma);
        stcDma.u32IntEn       = DMA_INT_ENABLE;
        stcDma.u32BlockSize   = 1UL;
        stcDma.u32TransCount  = 0UL;
        stcDma.u32DataWidth   = DMA_DATAWIDTH_8BIT;
        stcDma.u32SrcAddr     = 0UL;  /* set per-transfer */
        stcDma.u32DestAddr    = (uint32_t)(&USART3_UNIT->TDR);
        stcDma.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
        stcDma.u32DestAddrInc = DMA_DEST_ADDR_FIX;
        (void)DMA_Init(TX_DMA_UNIT, TX_DMA_CH, &stcDma);

        AOS_SetTriggerEventSrc(TX_DMA_TRIG_SEL, TX_DMA_TRIG_EVT);

        stcIrq.enIRQn      = TX_DMA_TC_IRQn;
        stcIrq.enIntSrc    = (en_int_src_t)TX_DMA_TC_INT_SRC;
        stcIrq.pfnCallback = HW_DmaTc_ISR;
        INTC_Install(&stcIrq, DDL_IRQ_PRIO_DEFAULT);

        DMA_Cmd(TX_DMA_UNIT, ENABLE);
        DMA_TransCompleteIntCmd(TX_DMA_UNIT, DMA_INT_TC_CH0, ENABLE);
    }

    /* NOTE: Do NOT lock PWC — later inits (Bemf/Dma) need FCG register access */
    s_bInitialized = true;

    MAIN_D("[Usart3_HW] Init OK: baud=%lu\r\n", s_stcCfg.baudrate);
}

void Usart3_HW_DeInit(void)
{
    if (!s_bInitialized) return;
    USART_FuncCmd(USART3_UNIT,
        (USART_RX | USART_TX | USART_INT_RX | USART_INT_TX_EMPTY | USART_INT_TX_CPLT), DISABLE);
    NVIC_DisableIRQ(RX_ERR_IRQn);
    NVIC_DisableIRQ(RX_FULL_IRQn);
    DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    DMA_TransCompleteIntCmd(TX_DMA_UNIT, DMA_INT_TC_CH0, DISABLE);
    s_bInitialized = false;
}

/*=============================================================================
 * Public API — RX
 *============================================================================*/
void Usart3_HW_StartRx(void)
{
    USART_FuncCmd(USART3_UNIT, USART_RX | USART_INT_RX, ENABLE);
}

void Usart3_HW_StopRx(void)
{
    USART_FuncCmd(USART3_UNIT, USART_RX | USART_INT_RX, DISABLE);
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

    DMA_SetSrcAddr(TX_DMA_UNIT, TX_DMA_CH, (uint32_t)data);
    DMA_SetTransCount(TX_DMA_UNIT, TX_DMA_CH, len);

    s_bTxBusy = true;
    (void)DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, ENABLE);
    USART_FuncCmd(USART3_UNIT, USART_TX, ENABLE);
    return true;
}

bool Usart3_HW_IsTxBusy(void)
{
    return s_bTxBusy;
}

/*=============================================================================
 * Public API — TX (blocking polling, debug)
 *============================================================================*/
void Usart3_HW_SendBlocking(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    if (!s_bInitialized || data == NULL || len == 0) return;

    /* Disable DMA so it doesn't interfere */
    DMA_ChCmd(TX_DMA_UNIT, TX_DMA_CH, DISABLE);
    DMA_TransCompleteIntCmd(TX_DMA_UNIT, DMA_INT_TC_CH0, DISABLE);

    /* Enable TX in pure-polling mode */
    USART_FuncCmd(USART3_UNIT,
        (USART_TX | USART_INT_TX_EMPTY | USART_INT_TX_CPLT), DISABLE);
    USART_FuncCmd(USART3_UNIT, USART_TX, ENABLE);

    for (i = 0; i < len; i++) {
        while (USART_GetStatus(USART3_UNIT, USART_FLAG_TX_EMPTY) != SET) {
            __NOP();
        }
        USART_WriteData(USART3_UNIT, data[i]);
    }
    while (USART_GetStatus(USART3_UNIT, USART_FLAG_TX_CPLT) != SET) {
        __NOP();
    }

    USART_FuncCmd(USART3_UNIT, USART_TX, DISABLE);
    USART_ClearStatus(USART3_UNIT, (USART_FLAG_TX_EMPTY | USART_FLAG_TX_CPLT));

    /* Restore DMA */
    DMA_TransCompleteIntCmd(TX_DMA_UNIT, DMA_INT_TC_CH0, ENABLE);
    s_bTxBusy = false;
}

/*=============================================================================
 * Public API — Callback registration
 *============================================================================*/
void Usart3_HW_RegisterRxCallback(Usart3_HW_RxCallback_t cb)      { s_pfnRxCb    = cb; }
void Usart3_HW_RegisterTxDoneCallback(Usart3_HW_TxDoneCallback_t cb) { s_pfnTxDoneCb = cb; }
void Usart3_HW_RegisterErrorCallback(Usart3_HW_ErrorCallback_t cb)  { s_pfnErrCb   = cb; }
