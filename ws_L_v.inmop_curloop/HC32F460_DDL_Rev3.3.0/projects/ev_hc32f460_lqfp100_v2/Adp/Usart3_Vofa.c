/**
 *******************************************************************************
 * @file  Usart3_Vofa.c
 * @brief USART3 + VOFA+ unified protocol layer implementation.
 *
 * Layers:
 *   Usart3_Vofa  (this file)  — JustFloat / FireWater / CMD / Vofa.c bridge
 *   Usart3_IO                 — ring buffer / DMA TX copy
 *   Usart3_HW                 — GPIO / USART / DMA / AOS / IRQ
 *
 * RX data flow:
 *   USART3 RX IRQ -> HW_RxFull_ISR -> IO_RxCallback -> BUF_Write(ring_buf)
 *   Main loop -> Usart3_Vofa_ReadCmd -> Usart3_IO_Read -> BUF_Read(ring_buf)
 *
 * TX data flow (DMA, non-blocking):
 *   Usart3_Vofa_SendFloats -> Usart3_IO_Send -> memcpy(txBuf) -> DMA
 *   Usart3_Vofa_Printf -> vsnprintf -> Usart3_IO_Send -> DMA
 *   DMA done -> HW_DmaTc_ISR -> IO_TxDoneCallback -> Busy cleared
 *******************************************************************************
 */

#include "Usart3_Vofa.h"
#include "Usart3_IO.h"
#include "rtt_manager.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/*=============================================================================
 * JustFloat tail bytes: {0x00, 0x00, 0x80, 0x7F}
 *============================================================================*/
static const uint8_t s_au8JfTail[] = USART3_VOFA_JF_TAIL;

/*=============================================================================
 * VOFA+ command tail
 *============================================================================*/
static const uint8_t s_au8CmdTail[] = USART3_VOFA_CMD_TAIL;
#define CMD_TAIL_LEN            (sizeof(s_au8CmdTail))

/*=============================================================================
 * Static — state
 *============================================================================*/
static bool     s_bInitialized;
static uint8_t  s_au8TxBuf[USART3_VOFA_TX_MAX];   /* printf staging buffer */

/*=============================================================================
 * Public API — Init / DeInit
 *============================================================================*/
void Usart3_Vofa_Init(const Usart3_HW_Config_t *hw_cfg)
{
    if (s_bInitialized) return;
    Usart3_IO_Init(hw_cfg);
    s_bInitialized = true;
}

void Usart3_Vofa_DeInit(void)
{
    if (!s_bInitialized) return;
    Usart3_IO_DeInit();
    s_bInitialized = false;
}

/*=============================================================================
 * Public API — TX: JustFloat
 *============================================================================*/

/**
 * @brief  Send float array as JustFloat: [float32 x N] + [tail 4B].
 */
bool Usart3_Vofa_SendFloats(const float *data, uint8_t count)
{
    uint8_t  buf[USART3_VOFA_TX_MAX];
    uint16_t frameLen;
    uint8_t  i;

    if (!s_bInitialized || data == NULL || count == 0) return false;

    frameLen = (uint16_t)count * sizeof(float) + USART3_VOFA_JF_TAIL_LEN;
    if (frameLen > USART3_VOFA_TX_MAX) return false;

    (void)memcpy(buf, data, (size_t)count * sizeof(float));
    for (i = 0; i < USART3_VOFA_JF_TAIL_LEN; i++) {
        buf[(count * sizeof(float)) + i] = s_au8JfTail[i];
    }

    return Usart3_IO_Send(buf, frameLen);
}

/**
 * @brief  Send int32 array as scaled JustFloat.
 */
bool Usart3_Vofa_SendScaled(const int32_t *data, uint8_t count, float scale)
{
    float   afBuf[USART3_VOFA_MAX_CHANNELS];
    uint8_t i;

    if (!s_bInitialized || data == NULL || count == 0
                         || count > USART3_VOFA_MAX_CHANNELS) {
        return false;
    }
    if (Usart3_IO_IsTxBusy()) return false;

    for (i = 0; i < count; i++) {
        afBuf[i] = (float)data[i] * scale;
    }

    return Usart3_Vofa_SendFloats(afBuf, count);
}

/*=============================================================================
 * Public API — TX: FireWater / Raw
 *============================================================================*/

bool Usart3_Vofa_Printf(const char *format, ...)
{
    int      n;
    va_list  args;

    if (!s_bInitialized) return false;
    if (Usart3_IO_IsTxBusy()) return false;

    va_start(args, format);
    n = vsnprintf((char *)s_au8TxBuf, USART3_VOFA_TX_MAX, format, args);
    va_end(args);

    if (n <= 0 || (uint32_t)n > USART3_VOFA_TX_MAX) {
        return false;
    }

    return Usart3_IO_Send(s_au8TxBuf, (uint16_t)n);
}

bool Usart3_Vofa_SendRaw(const uint8_t *data, uint16_t len)
{
    if (!s_bInitialized) return false;
    return Usart3_IO_Send(data, len);
}

bool Usart3_Vofa_IsTxBusy(void)
{
    return Usart3_IO_IsTxBusy();
}

/*=============================================================================
 * Public API — RX
 *============================================================================*/

uint16_t Usart3_Vofa_RxAvailable(void)
{
    if (!s_bInitialized) return 0;
    return Usart3_IO_RxAvailable();
}

uint16_t Usart3_Vofa_ReadRaw(uint8_t *buf, uint16_t max_len)
{
    if (!s_bInitialized) return 0;
    return Usart3_IO_Read(buf, max_len);
}

uint16_t Usart3_Vofa_ReadCmd(uint8_t *buf, uint16_t max_len)
{
    uint16_t len       = 0;
    uint16_t tailMatch = 0;
    uint8_t  byte;

    if (!s_bInitialized || buf == NULL || max_len == 0) return 0;

    while (len < max_len && Usart3_IO_Read(&byte, 1U) == 1U) {
        buf[len++] = byte;

        if (byte == s_au8CmdTail[tailMatch]) {
            tailMatch++;
            if (tailMatch >= CMD_TAIL_LEN) {
                break;   /* complete command frame */
            }
        } else {
            tailMatch = 0;
        }
    }

    return len;
}

uint16_t Usart3_Vofa_ReadLine(uint8_t *buf, uint16_t max_len)
{
    uint16_t len  = 0;
    uint8_t  byte;

    if (!s_bInitialized || buf == NULL || max_len == 0) return 0;

    while (len < max_len && Usart3_IO_Read(&byte, 1U) == 1U) {
        buf[len++] = byte;
        if (byte == '\n') break;
    }

    return len;
}

/*=============================================================================
 * Official Vofa.c bridge — Vofa_SendDataCallBack / Vofa_GetDataCallBack
 *
 * These are called by the official Vofa.c (Vofa_JustFloat etc.).
 * TX: DMA with busy-wait for back-to-back calls.
 * RX: stub — use Usart3_Vofa_FeedRx() instead of Vofa_ReceiveData().
 *============================================================================*/

void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data,
                           uint16_t length)
{
    (void)handle;
    if (data == NULL || length == 0U) return;

    /* Wait for previous DMA to finish (Vofa_JustFloat calls us twice) */
    while (Usart3_IO_IsTxBusy()) {
        /* spin */
    }

    (void)Usart3_IO_Send(data, length);
}

uint8_t Vofa_GetDataCallBack(Vofa_HandleTypedef *handle)
{
    (void)handle;
    return 0U;   /* not used — RX goes through FeedRx */
}

/**
 * @brief  Move all available bytes from ring_buf into Vofa FIFO.
 *         Call in main loop before Vofa_ReadCmd / Vofa_ReadLine.
 */
void Usart3_Vofa_FeedRx(Vofa_HandleTypedef *handle)
{
    uint8_t byte;

    if (handle == NULL) return;

    while (Usart3_IO_Read(&byte, 1U) == 1U) {
        *handle->rxBuffer.wp = byte;
        handle->rxBuffer.wp++;

        if (handle->rxBuffer.wp ==
            (handle->rxBuffer.buffer + VOFA_BUFFER_SIZE)) {
            handle->rxBuffer.wp = handle->rxBuffer.buffer;
        }

        if (handle->rxBuffer.wp == handle->rxBuffer.rp) {
            handle->rxBuffer.overflow = true;
        }
    }
}
