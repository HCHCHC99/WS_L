/**
 *******************************************************************************
 * @file  Usart3_Vofa_Runner.c
 * @brief Periodic heartbeat TX + RX command logging.
 *
 * Heartbeat:
 *   - Every 3 s, calls data provider (if registered) and sends JustFloat.
 *
 * RX logging:
 *   - Accumulates bytes until idle -> flush as one MAIN_D line.
 *   - {0xAF, 0xFA} tail -> "CMD:" hex; otherwise -> text as-is.
 *******************************************************************************
 */

#include "Usart3_Vofa_Runner.h"
#include "Usart3_Vofa.h"
#include "TickTimer.h"
#include "rtt_manager.h"
#include <string.h>

/*=============================================================================
 * Static — RX frame assembly (accumulate until idle timeout)
 *============================================================================*/
static uint8_t  s_au8RxFrame[USART3_VOFA_RX_LOG_BUF_SIZE];
static uint16_t s_u16RxLen;
static uint64_t s_u64RxLastMs;

/*=============================================================================
 * Static helpers
 *============================================================================*/
static bool HasCmdTail(const uint8_t *buf, uint16_t len, uint16_t *pPayloadLen)
{
    if (len >= 4U && buf[len - 2U] == 0xAFU && buf[len - 1U] == 0xFAU) {
        *pPayloadLen = len - 2U;
        return true;
    }
    *pPayloadLen = len;
    return false;
}

static char HexNibble(uint8_t n)
{
    n &= 0x0FU;
    return (char)((n < 10U) ? ('0' + n) : ('A' + n - 10U));
}

static uint16_t BytesToHex(const uint8_t *src, uint16_t len,
                           char *dst, uint16_t dstMax)
{
    uint16_t i, w = 0;
    for (i = 0; i < len && (w + 2U) <= dstMax; i++) {
        dst[w++] = HexNibble(src[i] >> 4U);
        dst[w++] = HexNibble(src[i]);
    }
    return w;
}

/*=============================================================================
 * Static — heartbeat
 *============================================================================*/
static NonBlockingDelay_t          s_stcHbDelay;
static uint32_t                    s_u32HbCount;
static Usart3_Vofa_DataProvider_t  s_pfnProvider;

/*=============================================================================
 * Public API
 *============================================================================*/

void Usart3_Vofa_Runner_Init(void)
{
    nbDelay_Init(&s_stcHbDelay, USART3_VOFA_HB_INTERVAL_MS);
    nbDelay_Start(&s_stcHbDelay);
    s_u32HbCount = 0U;
    s_pfnProvider = NULL;

    MAIN_D("[VOFA Runner] Init OK, hb_interval=%ums\r\n",
           (unsigned int)USART3_VOFA_HB_INTERVAL_MS);
}

void Usart3_Vofa_Runner_SetDataProvider(Usart3_Vofa_DataProvider_t provider)
{
    s_pfnProvider = provider;
}

void Usart3_Vofa_Runner_Run(void)
{
    /*---------------------------------------------------------------------
     * Heartbeat TX (3 s)
     *-------------------------------------------------------------------*/
    if (nbDelay_IsComplete(&s_stcHbDelay)) {
        nbDelay_Start(&s_stcHbDelay);
        s_u32HbCount++;

        if (!Usart3_Vofa_IsTxBusy() && s_pfnProvider != NULL) {
            int32_t buf[USART3_VOFA_MAX_CHANNELS];
            uint8_t count = 0;

            s_pfnProvider(buf, &count);

            if (count > 0 && count <= USART3_VOFA_MAX_CHANNELS) {
                Usart3_Vofa_SendScaled(buf, count, USART3_VOFA_SCALE_MILLI);
            }
        }
    }

    /*---------------------------------------------------------------------
     * RX: accumulate bytes until idle -> flush as one MAIN_D line
     *-------------------------------------------------------------------*/
    {
        uint8_t  tmp[USART3_VOFA_RX_LOG_BUF_SIZE];
        uint16_t n = Usart3_Vofa_ReadCmd(tmp, sizeof(tmp));

        if (n > 0U) {
            uint16_t space = (uint16_t)sizeof(s_au8RxFrame) - s_u16RxLen;
            if (n > space) n = space;
            (void)memcpy(&s_au8RxFrame[s_u16RxLen], tmp, n);
            s_u16RxLen  += n;
            s_u64RxLastMs = tickTimer_GetCount();
        }

        if (s_u16RxLen > 0U
            && (tickTimer_GetCount() - s_u64RxLastMs) >= USART3_VOFA_RX_IDLE_MS) {

            uint16_t payloadLen;
            bool     isCmd = HasCmdTail(s_au8RxFrame, s_u16RxLen, &payloadLen);

            if (isCmd) {
                char     hex[USART3_VOFA_RX_LOG_BUF_SIZE * 3U];
                uint16_t hexLen;

                hexLen = BytesToHex(s_au8RxFrame, payloadLen,
                                    hex, (uint16_t)sizeof(hex) - 1U);
                hex[hexLen] = '\0';
                MAIN_D("[VOFA RX] CMD: %s\r\n", hex);
            } else {
                char     txt[USART3_VOFA_RX_LOG_BUF_SIZE + 1U];
                uint16_t i;

                for (i = 0; i < payloadLen && i < sizeof(txt) - 1U; i++) {
                    uint8_t ch = s_au8RxFrame[i];
                    txt[i] = (ch >= 0x20U && ch < 0x7FU) ? (char)ch : '.';
                }
                txt[i] = '\0';
                MAIN_D("[VOFA RX] %s\r\n", txt);
            }

            s_u16RxLen = 0U;
        }
    }
}
