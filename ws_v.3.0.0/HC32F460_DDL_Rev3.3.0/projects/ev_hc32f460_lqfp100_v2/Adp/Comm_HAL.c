#include "Comm_HAL.h"
#include "TickTimer.h"
#include "ring_buf.h"
#include "msg_queue.h"
#include "lock.h"
#include <string.h>

/*=============================================================================
 * Internal constants
 *============================================================================*/
#define RX_FRAME_BUF_MAX    (256U)
#define MSG_TYPE_TX_DATA    (0x01U)

/*=============================================================================
 * Static data — ring buffers
 *============================================================================*/
static uint8_t m_au8RxRingBufData[500];
static uint8_t m_au8TxRingBufData[500];
static stc_ring_buf_t m_stcRxRingBuf;
static stc_ring_buf_t m_stcTxRingBuf;

/*=============================================================================
 * Static data — message queues
 *============================================================================*/
static msg_t m_au8RxFrameQueueBuf[10];
static msg_queue_t m_stcRxFrameQueue;
static msg_t m_au8TxQueueBuf[10];
static msg_queue_t m_stcTxQueue;

/*=============================================================================
 * Static data — mutex + send state
 *============================================================================*/
static mutex_t m_stcMutex;
static volatile bool m_bTxIdle;
static uint32_t m_u32LastRxTick;
static uint8_t  m_u8FrameTimeoutMs;
/*=============================================================================
 * Forward declarations
 *============================================================================*/
static void HAL_RxCallback(uint8_t byte);
static void HAL_TxReadyCallback(void);
static void HAL_TxCompleteCallback(void);
static void HAL_ErrorCallback(void);
static void HAL_StartSend(const uint8_t *data, uint16_t len);
static void HAL_FrameParser(void);
static uint8_t HAL_CalcFrameTimeout(uint32_t baudrate);

/*=============================================================================
 * Auto-calculate frame timeout from baudrate (3.5 char times)
 *============================================================================*/
static uint8_t HAL_CalcFrameTimeout(uint32_t baudrate)
{
    if (baudrate <= 1200UL)       return 30;
    else if (baudrate <= 2400UL)  return 15;
    else if (baudrate <= 4800UL)  return 8;
    else if (baudrate <= 9600UL)  return 4;
    else                          return 3;
}

/*=============================================================================
 * RS485 HW callbacks
 *============================================================================*/
static void HAL_RxCallback(uint8_t byte)
{
    BUF_Write(&m_stcRxRingBuf, &byte, 1U);
    m_u32LastRxTick = tickTimer_GetCount();
}

static void HAL_TxReadyCallback(void)
{
    uint8_t ch;
    if (!BUF_Empty(&m_stcTxRingBuf))
    {
        BUF_Read(&m_stcTxRingBuf, &ch, 1U);
        RS485_HW_SendByte(ch);
        RS485_HW_EnableTxEmptyInt();
    }
    else
    {
        RS485_HW_EnableTxCompleteInt();
        HAL_DEBUG("TX buf empty, enable TX_CPLT");
    }
}

static void HAL_TxCompleteCallback(void)
{
    m_bTxIdle = true;
    RS485_HW_StartRx();
    Lock_Unlock(&m_stcMutex, "HAL_TxComplete");

    HAL_DEBUG("TX done, queued=%d", MsgQueue_GetCount(&m_stcTxQueue));
}

static void HAL_ErrorCallback(void)
{
    HAL_ERR("USART error");
}

/*=============================================================================
 * Frame parser: assemble raw bytes into frames based on idle timeout
 *============================================================================*/
static void HAL_FrameParser(void)
{
    uint32_t now = tickTimer_GetCount();
    uint16_t available = BUF_UsedSize(&m_stcRxRingBuf);
    uint16_t freeSpace = BUF_FreeSize(&m_stcRxRingBuf);

    if (available == 0) return;

    /* Overflow protection: discard half if nearly full */
    if (freeSpace < 64U)
    {
        HAL_WARN("RxBuf nearly full! free=%d, discarding old", freeSpace);
        uint16_t discardLen = available / 2U;
        uint8_t discardBuf[32];
        while (discardLen > 0)
        {
            uint16_t chunk = (discardLen > sizeof(discardBuf)) ? sizeof(discardBuf) : discardLen;
            __disable_irq();
            BUF_Read(&m_stcRxRingBuf, discardBuf, chunk);
            __enable_irq();
            discardLen -= chunk;
        }
        available = BUF_UsedSize(&m_stcRxRingBuf);
        if (available == 0) return;
    }

    /* Check inter-frame timeout */
    uint32_t elapsed = now - m_u32LastRxTick;
    if (elapsed >= m_u8FrameTimeoutMs)
    {
        uint8_t frameBuf[RX_FRAME_BUF_MAX];
        uint16_t frameLen = available;
        if (frameLen > RX_FRAME_BUF_MAX) frameLen = RX_FRAME_BUF_MAX;

        __disable_irq();
        BUF_Read(&m_stcRxRingBuf, frameBuf, frameLen);
        __enable_irq();

        HAL_DEBUG("Frame assembled, size=%d", frameLen);

        msg_t rxMsg;
        rxMsg.data = rxMsg.buffer;
        rxMsg.len = frameLen;
        rxMsg.type = 0;
        rxMsg.priority = MSG_PRIO_NORMAL;
        memcpy(rxMsg.buffer, frameBuf, frameLen);
        MsgQueue_Send(&m_stcRxFrameQueue, &rxMsg, false, "HAL_FrameParser");
    }
}

/*=============================================================================
 * Start sending a frame via the hardware
 *============================================================================*/
static void HAL_StartSend(const uint8_t *data, uint16_t len)
{
    uint16_t written;

    RS485_HW_StartTx();

    written = BUF_Write(&m_stcTxRingBuf, (uint8_t *)data, len);
    if (written != len)
    {
        HAL_ERR("BUF_Write failed! written=%d, expected=%d", written, len);
        RS485_HW_StartRx();
        Lock_Unlock(&m_stcMutex, "HAL_StartSend_err");
        return;
    }

    HAL_DEBUG("Sending %d bytes (written=%d)", len, written);

    /* Kick off TX: write first byte, then TXE interrupt feeds the rest */
    uint8_t first;
    BUF_Read(&m_stcTxRingBuf, &first, 1U);
    RS485_HW_SendByte(first);
    RS485_HW_EnableTxEmptyInt();
}

/*=============================================================================
 * Public API
 *============================================================================*/

void Comm_HAL_Init(const RS485_HW_Config_t *hw_cfg, const Comm_HAL_Config_t *hal_cfg)
{
    if (hw_cfg == NULL || hal_cfg == NULL) return;

    m_u8FrameTimeoutMs = (hal_cfg->frame_timeout_ms > 0)
                       ? hal_cfg->frame_timeout_ms
                       : HAL_CalcFrameTimeout(hw_cfg->baudrate);

    /* Init mutex */
    Lock_Init(&m_stcMutex, LOCK_TYPE_MUTEX, "COMM_HAL_Mutex");

    /* Init TX queue */
    queue_config_t txCfg;
    txCfg.max_size = hal_cfg->tx_queue_depth;
    txCfg.overwrite = false;
    txCfg.priority_enabled = false;
    txCfg.timeout_ms = 0;
    MsgQueue_Init(&m_stcTxQueue, m_au8TxQueueBuf, hal_cfg->tx_queue_depth, &txCfg, "COMM_HAL_TxQ");

    /* Init RX frame queue */
    queue_config_t rxCfg;
    rxCfg.max_size = hal_cfg->rx_frame_queue_depth;
    rxCfg.overwrite = false;
    rxCfg.priority_enabled = false;
    rxCfg.timeout_ms = 0;
    MsgQueue_Init(&m_stcRxFrameQueue, m_au8RxFrameQueueBuf, hal_cfg->rx_frame_queue_depth, &rxCfg, "COMM_HAL_RxQ");

    /* Init ring buffers */
    BUF_Init(&m_stcRxRingBuf, m_au8RxRingBufData, hal_cfg->rx_buf_size);
    BUF_Init(&m_stcTxRingBuf, m_au8TxRingBufData, hal_cfg->tx_buf_size);

    m_bTxIdle = true;
    m_u32LastRxTick = tickTimer_GetCount();

    /* Initialize hardware first (clears callbacks internally) */
    RS485_HW_Init(hw_cfg);

    /* Register callbacks AFTER RS485_HW_Init() so they aren't nullified */
    RS485_HW_RegisterRxCallback(HAL_RxCallback);
    RS485_HW_RegisterTxReadyCallback(HAL_TxReadyCallback);
    RS485_HW_RegisterTxCompleteCallback(HAL_TxCompleteCallback);
    RS485_HW_RegisterErrorCallback(HAL_ErrorCallback);

    HAL_DEBUG("Init done, baud=%lu, frame_timeout=%dms", hw_cfg->baudrate, m_u8FrameTimeoutMs);
}

bool Comm_HAL_Send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len > RX_FRAME_BUF_MAX) return false;

    if (Lock_TryLock(&m_stcMutex, "HAL_Send"))
    {
        m_bTxIdle = false;
        HAL_StartSend(data, len);
        return true;
    }
    else
    {
        HAL_DEBUG("Busy, queue msg (qsz=%d)", MsgQueue_GetCount(&m_stcTxQueue) + 1);

        msg_t txMsg;
        txMsg.data = txMsg.buffer;
        txMsg.len = len;
        txMsg.type = MSG_TYPE_TX_DATA;
        txMsg.priority = MSG_PRIO_NORMAL;
        memcpy(txMsg.buffer, data, len);
        return MsgQueue_Send(&m_stcTxQueue, &txMsg, false, "HAL_Send");
    }
}

bool Comm_HAL_RecvFrame(uint8_t *buf, uint16_t *len, uint16_t max_len)
{
    msg_t rxMsg;

    if (!MsgQueue_Receive(&m_stcRxFrameQueue, &rxMsg, 0, "HAL_RecvFrame"))
        return false;

    if (rxMsg.len > max_len)
        return false;

    memcpy(buf, rxMsg.data, rxMsg.len);
    *len = rxMsg.len;
    return true;
}

void Comm_HAL_Poll(void)
{
    /* 1. Frame assembly from ring buffer */
    HAL_FrameParser();

    /* 2. TX queue drain: if idle and queue has pending frames, start sending */
    if (m_bTxIdle && MsgQueue_GetCount(&m_stcTxQueue) > 0)
    {
        msg_t next;
        if (MsgQueue_Receive(&m_stcTxQueue, &next, 0, "HAL_Poll"))
        {
            HAL_DEBUG("Dequeue send, remain=%d", MsgQueue_GetCount(&m_stcTxQueue));
            if (Lock_TryLock(&m_stcMutex, "HAL_Poll"))
            {
                m_bTxIdle = false;
                HAL_StartSend(next.data, next.len);
            }
        }
    }
}

uint32_t Comm_HAL_GetIdleTime(void)
{
    if (BUF_UsedSize(&m_stcRxRingBuf) == 0) return 0;
    return tickTimer_GetCount() - m_u32LastRxTick;
}

uint16_t Comm_HAL_GetTxQueueCount(void)
{
    return MsgQueue_GetCount(&m_stcTxQueue);
}

bool Comm_HAL_IsBusy(void)
{
    return !m_bTxIdle;
}

uint8_t Comm_HAL_GetFrameTimeout(void)
{
    return m_u8FrameTimeoutMs;
}
