/**
 *******************************************************************************
 * @file  Usart3_Vofa.c
 * @brief USART3 + VOFA+ protocol wrapper implementation.
 *
 * Layers:
 *   Usart3_Vofa  (this file)  — FireWater / command parsing / scaled JustFloat
 *   Usart3_IO                 — ring buffer / DMA TX copy / JustFloat frame
 *   Usart3_HW                 — GPIO / USART / DMA / AOS / IRQ
 *
 * RX data flow (fully interrupt-driven, no polling):
 *   USART3 RX IRQ → HW_RxFull_ISR → IO_RxCallback → BUF_Write(ring_buf)
 *   Main loop → Usart3_Vofa_ReadCmd/ReadLine → Usart3_IO_Read → BUF_Read(ring_buf)
 *
 * TX data flow (DMA, non-blocking):
 *   Usart3_Vofa_SendScaled → int32→float convert → Usart3_IO_SendFloats → DMA
 *   Usart3_Vofa_Printf → vsnprintf → Usart3_IO_Send → memcpy(txBuf) → DMA
 *   DMA done → HW_DmaTc_ISR → IO_TxDoneCallback → Busy flag cleared
 *******************************************************************************
 */

#include "Usart3_Vofa.h"
#include "Usart3_IO.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

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
 * Public API — TX
 *============================================================================*/

/**
 * @brief  Send int32 array as scaled JustFloat.
 *
 * Converts each int32 to float (int32 × scale → float), then delegates
 * to Usart3_IO_SendFloats for JustFloat framing and DMA send.
 *
 * Conversion is cheap on Cortex-M4F: single-cycle VCVT instruction.
 * No float variables needed in caller — all physical quantities stay as
 * scaled int32 (×1000 convention) in application code.
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

    /* int32 × scale → float (FPU: 1 cycle each) */
    for (i = 0; i < count; i++) {
        afBuf[i] = (float)data[i] * scale;
    }

    return Usart3_IO_SendFloats(afBuf, count);
}

/**
 * @brief  FireWater: format via vsnprintf, then DMA send.
 *
 * Blocking inside vsnprintf (CPU formats string), but the actual UART
 * transfer is DMA-driven and returns immediately.
 */
bool Usart3_Vofa_Printf(const char *format, ...)
{
    int      n;
    va_list  args;

    if (!s_bInitialized) return false;
    if (Usart3_IO_IsTxBusy()) return false;

    va_start(args, format);
    n = vsnprintf((char *)s_au8TxBuf, USART3_VOFA_TX_MAX, format, args);
    va_end(args);

    /* vsnprintf returns required length (could be > buffer) */
    if (n <= 0 || (uint32_t)n > USART3_VOFA_TX_MAX) {
        return false;
    }

    return Usart3_IO_Send(s_au8TxBuf, (uint16_t)n);
}

/**
 * @brief  Raw send: delegate to Usart3_IO.
 */
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

/**
 * @brief  Read one command frame by scanning for tail bytes {0xAF, 0xFA}.
 *
 * Reads byte-by-byte from ring_buf (via Usart3_IO_Read) so we can stop
 * as soon as the tail pattern is matched.  If max_len is reached before
 * the tail is found, returns the partial data (caller can check whether
 * the last 2 bytes match the tail).
 */
uint16_t Usart3_Vofa_ReadCmd(uint8_t *buf, uint16_t max_len)
{
    uint16_t len       = 0;
    uint16_t tailMatch = 0;
    uint8_t  byte;

    if (!s_bInitialized || buf == NULL || max_len == 0) {
        return 0;
    }

    while (len < max_len && Usart3_IO_Read(&byte, 1U) == 1U) {
        buf[len++] = byte;

        if (byte == s_au8CmdTail[tailMatch]) {
            tailMatch++;
            if (tailMatch >= CMD_TAIL_LEN) {
                /* Complete command frame received */
                break;
            }
        } else {
            tailMatch = 0;
        }
    }

    return len;
}

/**
 * @brief  Read one line (up to '\n') from ring_buf.
 *
 * Reads byte-by-byte; stops at '\n' or when buffer exhausted.
 */
uint16_t Usart3_Vofa_ReadLine(uint8_t *buf, uint16_t max_len)
{
    uint16_t len  = 0;
    uint8_t  byte;

    if (!s_bInitialized || buf == NULL || max_len == 0) {
        return 0;
    }

    while (len < max_len && Usart3_IO_Read(&byte, 1U) == 1U) {
        buf[len++] = byte;
        if (byte == '\n') {
            break;
        }
    }

    return len;
}
