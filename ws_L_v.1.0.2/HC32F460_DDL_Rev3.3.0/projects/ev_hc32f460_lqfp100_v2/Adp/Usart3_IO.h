/**
 *******************************************************************************
 * @file  Usart3_IO.h
 * @brief USART3 I/O + protocol layer (ring buffer, DMA send, VOFA+ JustFloat)
 *
 * Depends on Usart3_HW.h for hardware access.
 * Does NOT touch USART registers directly.
 *
 * Typical usage:
 * @code
 *   Usart3_IO_Init(NULL);                           // use default 921600
 *   Usart3_IO_SendBlocking((uint8_t*)"hello\r\n", 7); // startup test
 *
 *   float buf[9] = { ... };                         // 9-channel motor data
 *   if (!Usart3_IO_IsTxBusy())
 *       Usart3_IO_SendFloats(buf, 9);               // VOFA+ JustFloat frame
 * @endcode
 *******************************************************************************
 */

#ifndef __USART3_IO_H__
#define __USART3_IO_H__

#include <stdint.h>
#include <stdbool.h>
#include "Usart3_HW.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/
#define USART3_IO_TX_MAX        (256U)   /* max single DMA frame */
#define USART3_IO_RX_BUF_SIZE   (128U)   /* RX ring buffer */

/*******************************************************************************
 * Public API
 ******************************************************************************/

/**
 * @brief  Initialize IO layer: HW init + RX ring buffer + enable RX.
 * @param  hw_cfg  NULL = use defaults (921600, PB12 RX, PB13 TX)
 */
void Usart3_IO_Init(const Usart3_HW_Config_t *hw_cfg);

/**
 * @brief  De-initialize IO + HW layers.
 */
void Usart3_IO_DeInit(void);

/* ---- TX ---- */

/**
 * @brief  Send raw bytes via DMA (non-blocking).
 * @note   Copies data internally — caller can reuse buffer immediately.
 */
bool Usart3_IO_Send(const uint8_t *data, uint16_t len);

/**
 * @brief  Send raw bytes via polling (BLOCKING, debug only).
 * @note   Bypasses DMA. Use only for startup tests.
 */
void Usart3_IO_SendBlocking(const uint8_t *data, uint16_t len);

/**
 * @brief  Send float array in VOFA+ JustFloat format.
 *         Frame: [float32 × count] + tail [0x00 0x00 0x80 0x7F]
 * @note   Non-blocking (uses DMA).
 */
bool Usart3_IO_SendFloats(const float *data, uint8_t count);

/**
 * @brief  Check if DMA TX is in progress.
 */
bool Usart3_IO_IsTxBusy(void);

/* ---- RX ---- */

/**
 * @brief  Number of bytes available in RX ring buffer.
 */
uint16_t Usart3_IO_RxAvailable(void);

/**
 * @brief  Read received bytes (non-blocking, partial read OK).
 * @return Number of bytes actually read.
 */
uint16_t Usart3_IO_Read(uint8_t *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* __USART3_IO_H__ */
