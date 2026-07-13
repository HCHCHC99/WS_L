/**
 *******************************************************************************
 * @file  Usart3_Vofa_Runner.h
 * @brief Periodic heartbeat TX + RX command logging via MAIN_D.
 *
 * Runs in the main loop (non-blocking).  Two jobs:
 *   1. Heartbeat: every 3 seconds, send status via Usart3_Vofa (FireWater
 *      text + optional JustFloat scaled data from a user-registered provider).
 *   2. RX logging:  when a complete command frame {0xAF, 0xFA} arrives,
 *      dump the payload bytes via MAIN_D (Segger RTT).
 *
 * Usage:
 * @code
 *   // Init once
 *   Usart3_Vofa_Init(NULL);
 *   Usart3_Vofa_Runner_Init();
 *   Usart3_Vofa_Runner_SetDataProvider(my_provider);  // optional
 *
 *   // In main loop
 *   while (1) {
 *       CommRunner_Update();
 *       Usart3_Vofa_Runner_Run();   // <-- add this line
 *   }
 * @endcode
 *******************************************************************************
 */

#ifndef __USART3_VOFA_RUNNER_H__
#define __USART3_VOFA_RUNNER_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/** Heartbeat interval (ms) */
#define USART3_VOFA_HB_INTERVAL_MS   (3000U)

/** Max RX command payload to log */
#define USART3_VOFA_RX_LOG_BUF_SIZE  (64U)

/** RX frame idle timeout (ms): flush after this much silence */
#define USART3_VOFA_RX_IDLE_MS       (10U)

/*******************************************************************************
 * Data provider callback (for JustFloat heartbeat data)
 ******************************************************************************/

/**
 * @brief  Callback to fill scaled-int32 data for JustFloat heartbeat.
 * @param  buf    Output: int32 array to fill (up to USART3_VOFA_MAX_CHANNELS)
 * @param  count  Output: number of channels filled (set to 0 to skip)
 *
 * Example:
 * @code
 *   void my_provider(int32_t *buf, uint8_t *count) {
 *       buf[0] = g_motor_mode;     // mode (unscaled, use scale=1.0f)
 *       buf[1] = g_vbus_mV;        // mV → scale=0.001f → V
 *       buf[2] = g_speed_rpm;
 *       *count = 3;
 *   }
 * @endcode
 */
typedef void (*Usart3_Vofa_DataProvider_t)(int32_t *buf, uint8_t *count);

/*******************************************************************************
 * Public API
 ******************************************************************************/

/**
 * @brief  Initialize heartbeat timer.  Call once after Usart3_Vofa_Init().
 */
void Usart3_Vofa_Runner_Init(void);

/**
 * @brief  Main loop tick.  Call as fast as practical (non-blocking).
 *
 *         - Checks heartbeat timer; sends if elapsed
 *         - Checks RX ring_buf; prints received commands via MAIN_D
 */
void Usart3_Vofa_Runner_Run(void);

/**
 * @brief  Register a data provider for the JustFloat heartbeat frame.
 * @param  provider  Callback, or NULL to disable JustFloat heartbeat.
 */
void Usart3_Vofa_Runner_SetDataProvider(Usart3_Vofa_DataProvider_t provider);

#ifdef __cplusplus
}
#endif

#endif /* __USART3_VOFA_RUNNER_H__ */
