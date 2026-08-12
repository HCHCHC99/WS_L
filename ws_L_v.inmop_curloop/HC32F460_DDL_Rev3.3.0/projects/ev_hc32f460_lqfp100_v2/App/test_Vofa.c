/**
 *******************************************************************************
 * @file  test_Vofa.c
 * @brief VOFA+ test signal generator.
 *
 * Sends 2-channel JustFloat every 400ms via Usart3_Vofa:
 *   CH1: sawtooth  0 -> 100  step 0.5, wrap at 100
 *   CH2: counter   1 -> 200  step 1,   wrap at 1   (500ms timer)
 *
 * Compile only when VOFA_TEST_ENABLE=1 (in test_Vofa.h).
 *******************************************************************************
 */

#include "test_Vofa.h"

#if VOFA_TEST_ENABLE

#include "Usart3_Vofa.h"
#include "TickTimer.h"

/*=============================================================================
 * Test signal parameters
 *============================================================================*/
#define TEST_CH1_STEP        ((int32_t)500)     /* 0.5 x 1000   */
#define TEST_CH1_MAX         ((int32_t)100000)  /* 100 x 1000   */
#define TEST_CH2_START       ((int32_t)1000)    /* 1 x 1000     */
#define TEST_CH2_STEP        ((int32_t)1000)    /* 1 x 1000     */
#define TEST_CH2_MAX         ((int32_t)200000)  /* 200 x 1000   */
#define TEST_FRAME_INTERVAL_MS (400U)

/*=============================================================================
 * Static — state
 *============================================================================*/
static int32_t            s_i32Sawtooth;
static int32_t            s_i32Counter;
static NonBlockingDelay_t s_stcFrameDelay;
static NonBlockingDelay_t s_stcCh2Delay;

/*=============================================================================
 * Public API
 *============================================================================*/

void TestVofa_Init(void)
{
    s_i32Sawtooth = 0;
    s_i32Counter  = TEST_CH2_START;

    nbDelay_Init(&s_stcFrameDelay, TEST_FRAME_INTERVAL_MS);
    nbDelay_Start(&s_stcFrameDelay);

    nbDelay_Init(&s_stcCh2Delay, 500U);
    nbDelay_Start(&s_stcCh2Delay);
}

void TestVofa_Run(void)
{
    /* CH2 has its own 500ms timer */
    if (nbDelay_IsComplete(&s_stcCh2Delay)) {
        nbDelay_Start(&s_stcCh2Delay);
        s_i32Counter += TEST_CH2_STEP;
        if (s_i32Counter > TEST_CH2_MAX) {
            s_i32Counter = TEST_CH2_START;
        }
    }

    /* CH1 + CH2 sent together every 400ms */
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
        }
    }
}

#endif /* VOFA_TEST_ENABLE */
