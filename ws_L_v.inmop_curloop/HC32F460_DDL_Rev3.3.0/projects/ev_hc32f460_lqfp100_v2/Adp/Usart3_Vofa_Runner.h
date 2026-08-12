/**
 *******************************************************************************
 * @file  Usart3_Vofa_Runner.h
 * @brief Periodic heartbeat TX + RX command logging via MAIN_D.
 *
 * Runs in main loop (non-blocking). Two jobs:
 *   1. Heartbeat: every 3 seconds, send status via Usart3_Vofa.
 *   2. RX logging: accumulate bytes with idle timeout, print via MAIN_D.
 *
 * Usage:
 * @code
 *   Usart3_Vofa_Init(NULL);
 *   Usart3_Vofa_Runner_Init();
 *
 *   while (1) {
 *       Usart3_Vofa_Runner_Run();
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

#define USART3_VOFA_HB_INTERVAL_MS   (3000U)
#define USART3_VOFA_RX_LOG_BUF_SIZE  (64U)
#define USART3_VOFA_RX_IDLE_MS       (10U)

/*******************************************************************************
 * Data provider callback (for heartbeat JustFloat data)
 ******************************************************************************/

typedef void (*Usart3_Vofa_DataProvider_t)(int32_t *buf, uint8_t *count);

/*******************************************************************************
 * Public API
 ******************************************************************************/

void Usart3_Vofa_Runner_Init(void);
void Usart3_Vofa_Runner_Run(void);
void Usart3_Vofa_Runner_SetDataProvider(Usart3_Vofa_DataProvider_t provider);

#ifdef __cplusplus
}
#endif

#endif /* __USART3_VOFA_RUNNER_H__ */
