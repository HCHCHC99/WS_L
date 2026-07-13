/**
 *******************************************************************************
 * @file  Usart3_Vofa_Runner.c
 * @brief Periodic heartbeat TX + RX command logging.
 *
 * Heartbeat:
 *   - FireWater text:  "hb: tick=<ms> sent=<cnt>\r\n"  every 3 s
 *   - JustFloat binary: scaled-int32 channels from user provider (optional)
 *
 * RX logging:
 *   - Accumulates bytes in s_au8RxFrame[] across multiple main-loop ticks
 *   - Flushes when idle for USART3_VOFA_RX_IDLE_MS (10ms) → one MAIN_D line
 *   - CMD tag: frame has {0xAF, 0xFA} tail → hex dump of payload
 *   - Text:   no tail → prints payload as-is (non-printable → '.')
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
 * Static helper — scan for tail {0xAF, 0xFA} in buffer
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

/*=============================================================================
 * Static — test sawtooth: ch1 0→100 step 0.5 / 400ms; ch2 1→200 step 1 / 500ms
 *============================================================================*/
#define TEST_CH1_STEP        ((int32_t)500)      /* 0.5 × 1000   */
#define TEST_CH1_MAX         ((int32_t)100000)   /* 100 × 1000   */
#define TEST_CH2_START       ((int32_t)1000)     /* 1 × 1000     */
#define TEST_CH2_STEP        ((int32_t)1000)     /* 1 × 1000     */
#define TEST_CH2_MAX         ((int32_t)200000)   /* 200 × 1000   */
#define TEST_FRAME_INTERVAL_MS (400U)            /* both ch's in one frame */

static int32_t            s_i32Sawtooth;
static int32_t            s_i32Counter;
static NonBlockingDelay_t s_stcFrameDelay;
static NonBlockingDelay_t s_stcCh2Delay;          /* CH2 independent 500ms */

/*=============================================================================
 * Static — heartbeat
 *============================================================================*/
static NonBlockingDelay_t          s_stcHbDelay;
static uint32_t                    s_u32HbCount;
static Usart3_Vofa_DataProvider_t  s_pfnProvider;

/*=============================================================================
 * Static helper — byte-to-hex
 *============================================================================*/
static char HexNibble(uint8_t nibble)
{
    nibble &= 0x0FU;
    return (char)((nibble < 10U) ? ('0' + nibble) : ('A' + nibble - 10U));
}

/**
 * @brief  Convert byte array to hex string (no null-terminator added).
 * @return  Number of hex chars written (2 × len).
 */
static uint16_t BytesToHex(const uint8_t *src, uint16_t len,
                           char *dst, uint16_t dstMax)
{
    uint16_t i;
    uint16_t w = 0;

    for (i = 0; i < len && (w + 2U) <= dstMax; i++) {
        dst[w++] = HexNibble(src[i] >> 4U);
        dst[w++] = HexNibble(src[i]);
    }
    return w;
}

/*=============================================================================
 * Public API
 *============================================================================*/

void Usart3_Vofa_Runner_Init(void)
{
    nbDelay_Init(&s_stcHbDelay, USART3_VOFA_HB_INTERVAL_MS);
    nbDelay_Start(&s_stcHbDelay);
    s_u32HbCount = 0U;
    s_pfnProvider = NULL;

    /* Test signals init: CH1=0, CH2=1 */
    s_i32Sawtooth = 0;
    s_i32Counter  = TEST_CH2_START;
    nbDelay_Init(&s_stcFrameDelay, TEST_FRAME_INTERVAL_MS);
    nbDelay_Start(&s_stcFrameDelay);
    nbDelay_Init(&s_stcCh2Delay,   500U);
    nbDelay_Start(&s_stcCh2Delay);

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
     * Test signals: 2-ch JustFloat every 400ms
     *   CH1: sawtooth, 0→100 step 0.5, wrap at 100
     *   CH2: counter,  1→200 step 1,   wrap at 200 (500ms timer)
     *-------------------------------------------------------------------*/
    /* CH2 has its own 500ms timer */
    if (nbDelay_IsComplete(&s_stcCh2Delay)) {
        nbDelay_Start(&s_stcCh2Delay);
        s_i32Counter += TEST_CH2_STEP;
        if (s_i32Counter > TEST_CH2_MAX) {
            s_i32Counter = TEST_CH2_START;
        }
    }

    /* CH1 + CH2 sent together in one 2-channel frame every 400ms */
    if (nbDelay_IsComplete(&s_stcFrameDelay)) {
        nbDelay_Start(&s_stcFrameDelay);

        s_i32Sawtooth += TEST_CH1_STEP;
        if (s_i32Sawtooth > TEST_CH1_MAX) {
            s_i32Sawtooth = 0;
        }

        if (!Usart3_Vofa_IsTxBusy()) {
            int32_t buf[2];
            buf[0] = s_i32Sawtooth;
            buf[1] = s_i32Counter;
            Usart3_Vofa_SendScaled(buf, 2U, USART3_VOFA_SCALE_MILLI);

            /* Debug: VOFA+ will see CH1/1000, CH2/1000 */
            MAIN_D("[VOFA TX] CH1_milli=%d  CH2_milli=%d\r\n",
                   (int)buf[0], (int)buf[1]);
        }
    }

    /*---------------------------------------------------------------------
     * Heartbeat TX
     *-------------------------------------------------------------------*/
    if (nbDelay_IsComplete(&s_stcHbDelay)) {
        nbDelay_Start(&s_stcHbDelay);            /* restart timer */
        s_u32HbCount++;

        /* Only send if previous TX is done (DMA free) */
        if (!Usart3_Vofa_IsTxBusy()) {

            /* ---- FireWater text heartbeat (disabled: conflicts with JustFloat) ---- */
#if 0
            Usart3_Vofa_Printf("hb: tick=%lu sent=%lu\r\n",
                               (unsigned long)tickTimer_GetCount(),
                               (unsigned long)s_u32HbCount);
#endif

            /* ---- JustFloat binary heartbeat (optional) ---- */
            if (s_pfnProvider != NULL) {
                int32_t buf[USART3_VOFA_MAX_CHANNELS];
                uint8_t count = 0;

                s_pfnProvider(buf, &count);

                if (count > 0 && count <= USART3_VOFA_MAX_CHANNELS) {
                    /* Wait for FireWater DMA to finish, then send floats */
                    while (Usart3_Vofa_IsTxBusy()) { /* spin */ }
                    Usart3_Vofa_SendScaled(buf, count,
                                           USART3_VOFA_SCALE_MILLI);
                }
            }
        }
    }

    /*---------------------------------------------------------------------
     * RX: accumulate bytes until idle → flush as one message
     *
     * Problem: at 115200 baud the main loop is faster than UART, so
     * ReadCmd often returns 1 byte per call.  Accumulating with an idle
     * timeout lets us print "1231" as one line instead of four.
     *-------------------------------------------------------------------*/
    {
        uint8_t  tmp[USART3_VOFA_RX_LOG_BUF_SIZE];
        uint16_t n = Usart3_Vofa_ReadCmd(tmp, sizeof(tmp));

        if (n > 0U) {
            /* Append to frame buffer */
            uint16_t space = (uint16_t)sizeof(s_au8RxFrame) - s_u16RxLen;
            if (n > space) n = space;
            (void)memcpy(&s_au8RxFrame[s_u16RxLen], tmp, n);
            s_u16RxLen  += n;
            s_u64RxLastMs = tickTimer_GetCount();
        }

        /* Flush when idle long enough and we have data */
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

            s_u16RxLen = 0U;   /* reset for next frame */
        }
    }
}
