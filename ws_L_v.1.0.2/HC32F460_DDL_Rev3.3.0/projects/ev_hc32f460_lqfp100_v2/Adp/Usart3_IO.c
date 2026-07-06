/**
 *******************************************************************************
 * @file  Usart3_IO.c
 * @brief USART3 I/O + protocol layer.
 *
 * Owns:
 *   - Internal TX buffer (DMA-safe copy)
 *   - RX ring buffer
 *   - Busy flag
 *   - VOFA+ JustFloat frame assembly
 *
 * Delegates to Usart3_HW for register-level operations.
 *******************************************************************************
 */

#include "Usart3_IO.h"
#include "ring_buf.h"
#include "rtt_manager.h"
#include <string.h>

/*=============================================================================
 * VOFA+ JustFloat tail: 0x00 0x00 0x80 0x7F
 *============================================================================*/
#define VOFA_TAIL_LEN   (4U)
static const uint8_t s_au8VofaTail[VOFA_TAIL_LEN] = {0x00, 0x00, 0x80, 0x7F};

/*=============================================================================
 * Static — buffers & state
 *============================================================================*/
static uint8_t         s_au8TxBuf[USART3_IO_TX_MAX];
static volatile bool   s_bTxBusy;

static uint8_t         s_au8RxRing[USART3_IO_RX_BUF_SIZE];
static stc_ring_buf_t  s_stcRxRing;

static bool            s_bInitialized;

/*=============================================================================
 * HW callbacks — thin shims
 *============================================================================*/
static void IO_RxCallback(uint8_t byte)
{
    (void)BUF_Write(&s_stcRxRing, &byte, 1U);
}

static void IO_TxDoneCallback(void)
{
    s_bTxBusy = false;
}

static void IO_ErrorCallback(void)
{
    /* silent: errors are cleared at HW level */
}

/*=============================================================================
 * Public API — Init / DeInit
 *============================================================================*/
void Usart3_IO_Init(const Usart3_HW_Config_t *hw_cfg)
{
    if (s_bInitialized) return;

    /* Ring buffer */
    (void)BUF_Init(&s_stcRxRing, s_au8RxRing, sizeof(s_au8RxRing));

    /* Hardware */
    Usart3_HW_Init(hw_cfg);
    Usart3_HW_RegisterRxCallback(IO_RxCallback);
    Usart3_HW_RegisterTxDoneCallback(IO_TxDoneCallback);
    Usart3_HW_RegisterErrorCallback(IO_ErrorCallback);
    Usart3_HW_StartRx();

    s_bTxBusy     = false;
    s_bInitialized = true;
}

void Usart3_IO_DeInit(void)
{
    if (!s_bInitialized) return;
    Usart3_HW_DeInit();
    s_bInitialized = false;
    s_bTxBusy      = false;
}

/*=============================================================================
 * Public API — TX
 *============================================================================*/
bool Usart3_IO_Send(const uint8_t *data, uint16_t len)
{
    if (!s_bInitialized || data == NULL || len == 0 || len > USART3_IO_TX_MAX) {
        return false;
    }
    if (s_bTxBusy) return false;

    /* Copy to internal buffer — caller can free data immediately */
    memcpy(s_au8TxBuf, data, len);

    if (!Usart3_HW_StartTxDma(s_au8TxBuf, len)) {
        return false;
    }
    s_bTxBusy = true;
    return true;
}

void Usart3_IO_SendBlocking(const uint8_t *data, uint16_t len)
{
    if (!s_bInitialized || data == NULL || len == 0) return;
    Usart3_HW_SendBlocking(data, len);
}

bool Usart3_IO_SendFloats(const float *data, uint8_t count)
{
    uint8_t  buf[USART3_IO_TX_MAX];
    uint16_t frameLen;
    uint8_t  i;

    if (!s_bInitialized || data == NULL || count == 0) return false;

    frameLen = (uint16_t)count * sizeof(float) + VOFA_TAIL_LEN;
    if (frameLen > USART3_IO_TX_MAX) return false;

    memcpy(buf, data, (size_t)count * sizeof(float));
    for (i = 0; i < VOFA_TAIL_LEN; i++) {
        buf[(count * sizeof(float)) + i] = s_au8VofaTail[i];
    }
    return Usart3_IO_Send(buf, frameLen);
}

bool Usart3_IO_IsTxBusy(void)
{
    return s_bTxBusy;
}

/*=============================================================================
 * Public API — RX
 *============================================================================*/
uint16_t Usart3_IO_RxAvailable(void)
{
    if (!s_bInitialized) return 0;
    return BUF_UsedSize(&s_stcRxRing);
}

uint16_t Usart3_IO_Read(uint8_t *buf, uint16_t max_len)
{
    uint16_t n;
    if (!s_bInitialized || buf == NULL || max_len == 0) return 0;
    n = BUF_UsedSize(&s_stcRxRing);
    if (n > max_len) n = max_len;
    if (n > 0) (void)BUF_Read(&s_stcRxRing, buf, n);
    return n;
}
