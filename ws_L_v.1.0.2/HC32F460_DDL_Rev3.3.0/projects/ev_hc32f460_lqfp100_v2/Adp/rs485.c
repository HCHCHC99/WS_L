#include "rs485.h"
#include "Hardware.h"
#include "Gpio_io.h"

/*=============================================================================
 * USART4 pin definitions
 *============================================================================*/
#define USART_RX_PORT       (GPIO_PORT_B)
#define USART_RX_PIN        (GPIO_PIN_12)
#define USART_RX_GPIO_FUNC  (GPIO_FUNC_37)
#define USART_TX_PORT       (GPIO_PORT_B)
#define USART_TX_PIN        (GPIO_PIN_13)
#define USART_TX_GPIO_FUNC  (GPIO_FUNC_36)
#define USART_UNIT          (CM_USART4)
#define USART_FCG_ENABLE()  FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART4, ENABLE)

/* RS485 direction pin */
#define RS485_DIR_PORT      (GPIO_PORT_A)
#define RS485_DIR_PIN       (GPIO_PIN_03)

/* USART interrupt definitions */
#define USART_RX_ERR_IRQn      (INT000_IRQn)
#define USART_RX_ERR_INT_SRC   (INT_SRC_USART4_EI)
#define USART_RX_FULL_IRQn     (INT001_IRQn)
#define USART_RX_FULL_INT_SRC  (INT_SRC_USART4_RI)
#define USART_TX_EMPTY_IRQn    (INT002_IRQn)
#define USART_TX_EMPTY_INT_SRC (INT_SRC_USART4_TI)
#define USART_TX_CPLT_IRQn     (INT003_IRQn)
#define USART_TX_CPLT_INT_SRC  (INT_SRC_USART4_TCI)

/*=============================================================================
 * Baudrate clock divider auto-select
 *============================================================================*/
static uint32_t RS485_GetClkDiv(uint32_t baudrate)
{
    if (baudrate <= 4800UL)       return USART_CLK_DIV64;
    else if (baudrate <= 38400UL) return USART_CLK_DIV16;
    else if (baudrate <= 115200UL) return USART_CLK_DIV4;
    else                           return USART_CLK_DIV1;
}

/*=============================================================================
 * Static variables
 *============================================================================*/
static RS485_HW_Config_t m_stcHwConfig;
static bool m_bInitialized = false;

static RS485_RxCallback_t          m_pfnRxCallback;
static RS485_TxReadyCallback_t     m_pfnTxReadyCallback;
static RS485_TxCompleteCallback_t  m_pfnTxCompleteCallback;
static RS485_ErrorCallback_t       m_pfnErrorCallback;

/*=============================================================================
 * Static helpers
 *============================================================================*/
static void INTC_IrqInstalHandler(const stc_irq_signin_config_t *cfg, uint32_t prio)
{
    if (cfg != NULL)
    {
        INTC_IrqSignIn(cfg);
        NVIC_ClearPendingIRQ(cfg->enIRQn);
        NVIC_SetPriority(cfg->enIRQn, prio);
        NVIC_EnableIRQ(cfg->enIRQn);
    }
}

static void RS485_DIR_Init(uint8_t polarity)
{
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    Output_GPIO_Init(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_INIT_LOW);
    GPIO_SetFunc(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_FUNC_0);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
    RS485_DEBUG("DIR init, polarity=%d", polarity);
}

static void RS485_SetTxMode(void)
{
    if (m_stcHwConfig.dir_polarity == 0)
    {
        GPIO_SetPins(RS485_DIR_PORT, RS485_DIR_PIN);
    }
    else
    {
        GPIO_ResetPins(RS485_DIR_PORT, RS485_DIR_PIN);
    }
}

static void RS485_SetRxMode(void)
{
    if (m_stcHwConfig.dir_polarity == 0)
    {
        GPIO_ResetPins(RS485_DIR_PORT, RS485_DIR_PIN);
    }
    else
    {
        GPIO_SetPins(RS485_DIR_PORT, RS485_DIR_PIN);
    }
}

/*=============================================================================
 * ISR callbacks — delegate to registered handlers
 *============================================================================*/
static void USART_RxFull_IrqCallback(void)
{
    uint8_t ch = (uint8_t)USART_ReadData(USART_UNIT);
    if (m_pfnRxCallback) {
        m_pfnRxCallback(ch);
    }
}

static void USART_RxError_IrqCallback(void)
{
    uint32_t errFlag = USART_FLAG_OVERRUN | USART_FLAG_FRAME_ERR | USART_FLAG_PARITY_ERR;
    USART_ClearStatus(USART_UNIT, errFlag);
    (void)USART_ReadData(USART_UNIT);
    if (m_pfnErrorCallback) {
        m_pfnErrorCallback();
    }
    RS485_ERR_DBG("USART err");
}

static void USART_TxEmpty_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, USART_INT_TX_EMPTY, DISABLE);
    if (m_pfnTxReadyCallback) {
        m_pfnTxReadyCallback();
    }
}

static void USART_TxComplete_IrqCallback(void)
{
    USART_FuncCmd(USART_UNIT, USART_TX | USART_INT_TX_EMPTY | USART_INT_TX_CPLT, DISABLE);
    if (m_pfnTxCompleteCallback) {
        m_pfnTxCompleteCallback();
    }
    RS485_DEBUG("TX cplt");
}

/*=============================================================================
 * Public API
 *============================================================================*/

void RS485_HW_Init(const RS485_HW_Config_t *cfg)
{
    stc_usart_uart_init_t uartCfg;
    stc_irq_signin_config_t irqCfg;

    if (cfg == NULL) return;
    m_stcHwConfig = *cfg;

    m_pfnRxCallback         = NULL;
    m_pfnTxReadyCallback    = NULL;
    m_pfnTxCompleteCallback = NULL;
    m_pfnErrorCallback      = NULL;

    /* GPIO function select */
    GPIO_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_GPIO_FUNC);
    GPIO_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_GPIO_FUNC);

    /* USART init */
    USART_FCG_ENABLE();
    USART_UART_StructInit(&uartCfg);
    uartCfg.u32Baudrate      = cfg->baudrate;
    uartCfg.u32ClockDiv      = RS485_GetClkDiv(cfg->baudrate);
    uartCfg.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    USART_UART_Init(USART_UNIT, &uartCfg, NULL);

    /* Interrupt registration */
    irqCfg.enIRQn = USART_RX_ERR_IRQn;
    irqCfg.enIntSrc = USART_RX_ERR_INT_SRC;
    irqCfg.pfnCallback = USART_RxError_IrqCallback;
    INTC_IrqInstalHandler(&irqCfg, DDL_IRQ_PRIO_DEFAULT);

    irqCfg.enIRQn = USART_RX_FULL_IRQn;
    irqCfg.enIntSrc = USART_RX_FULL_INT_SRC;
    irqCfg.pfnCallback = USART_RxFull_IrqCallback;
    INTC_IrqInstalHandler(&irqCfg, DDL_IRQ_PRIO_DEFAULT);

    irqCfg.enIRQn = USART_TX_EMPTY_IRQn;
    irqCfg.enIntSrc = USART_TX_EMPTY_INT_SRC;
    irqCfg.pfnCallback = USART_TxEmpty_IrqCallback;
    INTC_IrqInstalHandler(&irqCfg, DDL_IRQ_PRIO_DEFAULT);

    irqCfg.enIRQn = USART_TX_CPLT_IRQn;
    irqCfg.enIntSrc = USART_TX_CPLT_INT_SRC;
    irqCfg.pfnCallback = USART_TxComplete_IrqCallback;
    INTC_IrqInstalHandler(&irqCfg, DDL_IRQ_PRIO_DEFAULT);

    /* RS485 direction pin init */
    RS485_DIR_Init(cfg->dir_polarity);

    /* Start in receive mode */
    USART_FuncCmd(USART_UNIT, USART_RX | USART_INT_RX, ENABLE);

    m_bInitialized = true;
    RS485_DEBUG("HW Init done, baud=%lu", cfg->baudrate);
}

void RS485_HW_DeInit(void)
{
    USART_FuncCmd(USART_UNIT, USART_RX | USART_TX | USART_INT_RX | USART_INT_TX_EMPTY | USART_INT_TX_CPLT, DISABLE);
    NVIC_DisableIRQ(USART_RX_ERR_IRQn);
    NVIC_DisableIRQ(USART_RX_FULL_IRQn);
    NVIC_DisableIRQ(USART_TX_EMPTY_IRQn);
    NVIC_DisableIRQ(USART_TX_CPLT_IRQn);

    m_pfnRxCallback         = NULL;
    m_pfnTxReadyCallback    = NULL;
    m_pfnTxCompleteCallback = NULL;
    m_pfnErrorCallback      = NULL;
    m_bInitialized          = false;
}

void RS485_HW_SetBaudrate(uint32_t baudrate)
{
    stc_usart_uart_init_t uartCfg;
    m_stcHwConfig.baudrate = baudrate;

    USART_UART_StructInit(&uartCfg);
    uartCfg.u32Baudrate      = baudrate;
    uartCfg.u32ClockDiv      = RS485_GetClkDiv(baudrate);
    uartCfg.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    USART_UART_Init(USART_UNIT, &uartCfg, NULL);

    RS485_DEBUG("Baudrate set to %lu", baudrate);
}

void RS485_HW_SendByte(uint8_t byte)
{
    USART_WriteData(USART_UNIT, byte);
}

void RS485_HW_StartTx(void)
{
    RS485_SetTxMode();
    USART_FuncCmd(USART_UNIT, USART_RX | USART_INT_RX, DISABLE);
    USART_FuncCmd(USART_UNIT, USART_TX, ENABLE);
    RS485_DEBUG("TX mode");
}

void RS485_HW_StartRx(void)
{
    USART_FuncCmd(USART_UNIT, USART_TX | USART_INT_TX_EMPTY | USART_INT_TX_CPLT, DISABLE);
    RS485_SetRxMode();
    USART_FuncCmd(USART_UNIT, USART_RX | USART_INT_RX, ENABLE);
    RS485_DEBUG("RX mode");
}

void RS485_HW_RegisterRxCallback(RS485_RxCallback_t cb)
{
    m_pfnRxCallback = cb;
}

void RS485_HW_RegisterTxReadyCallback(RS485_TxReadyCallback_t cb)
{
    m_pfnTxReadyCallback = cb;
}

void RS485_HW_RegisterTxCompleteCallback(RS485_TxCompleteCallback_t cb)
{
    m_pfnTxCompleteCallback = cb;
}

void RS485_HW_RegisterErrorCallback(RS485_ErrorCallback_t cb)
{
    m_pfnErrorCallback = cb;
}

void RS485_HW_EnableTxEmptyInt(void)
{
    USART_FuncCmd(USART_UNIT, USART_INT_TX_EMPTY, ENABLE);
}

void RS485_HW_EnableTxCompleteInt(void)
{
    USART_FuncCmd(USART_UNIT, USART_INT_TX_CPLT, ENABLE);
}
