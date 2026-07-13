/**
 *******************************************************************************
 * @file  Vofa_Bridge.c
 * @brief Implements official Vofa.c callbacks using Usart3_IO (DMA TX + ring_buf RX).
 *
 * TX: Vofa_SendDataCallBack → Usart3_IO_Send (DMA, waits if previous DMA busy)
 * RX: Vofa_Bridge_FeedRx polls ring_buf → Vofa FIFO. The ISR still writes
 *     to ring_buf (unchanged), and the main loop calls FeedRx to bridge.
 *
 * Vofa_GetDataCallBack is NOT used in normal operation — FeedRx directly
 * writes to Vofa's FIFO.  It's implemented as a stub returning 0.
 *******************************************************************************
 */

#include "Vofa_Bridge.h"
#include "Usart3_IO.h"
#include <string.h>

/*=============================================================================
 * TX callback: called by Vofa_SendData / Vofa_JustFloat / Vofa_Printf
 *
 * Vofa_JustFloat calls us TWICE per frame (float data, then tail).
 * Usart3_IO_Send is DMA-driven and non-blocking, so we spin-wait if a
 * previous transfer is still in flight.
 *============================================================================*/
void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t length)
{
    (void)handle;

    if (data == NULL || length == 0U) return;

    /* Wait for previous DMA to finish */
    while (Usart3_IO_IsTxBusy()) {
        /* spin */
    }

    (void)Usart3_IO_Send(data, length);
}

/*=============================================================================
 * RX callback: NOT used in normal flow.
 *
 * Our ISR writes bytes to Usart3_IO ring_buf (unchanged).
 * Vofa_Bridge_FeedRx() moves them from ring_buf → Vofa FIFO manually.
 * This stub exists so the linker is happy.
 *============================================================================*/
uint8_t Vofa_GetDataCallBack(Vofa_HandleTypedef *handle)
{
    (void)handle;
    return 0U;
}

/*=============================================================================
 * FeedRx: bridge ring_buf → Vofa FIFO
 *
 * Call this in main loop BEFORE Vofa_ReadCmd / Vofa_ReadLine / Vofa_ReadData.
 * Non-blocking — drains whatever bytes are available in ring_buf.
 *============================================================================*/
void Vofa_Bridge_FeedRx(Vofa_HandleTypedef *handle)
{
    uint8_t byte;

    if (handle == NULL) return;

    while (Usart3_IO_Read(&byte, 1U) == 1U) {
        /* Same logic as Vofa_ReceiveData(), but takes byte as param */
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
