/**
 *******************************************************************************
 * @file  Usart3_Vofa.h
 * @brief USART3 + VOFA+ unified protocol layer.
 *
 * Single entry point for all VOFA+ functionality:
 *   - JustFloat (SendFloats / SendScaled)
 *   - FireWater (Printf)
 *   - Raw TX/RX
 *   - Command-frame RX ({0xAF, 0xFA} tail)
 *   - Line-based RX
 *   - Official Vofa.c bridge (Vofa_SendDataCallBack / Vofa_GetDataCallBack)
 *
 * Sits on Usart3_IO (DMA TX + ring_buf RX).
 * Does NOT touch USART/DMA registers directly.
 *******************************************************************************
 */

#ifndef __USART3_VOFA_H__
#define __USART3_VOFA_H__

#include <stdint.h>
#include <stdbool.h>
#include "Usart3_HW.h"
#include "Vofa.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/** Max single TX frame (matches Usart3_IO TX buffer) */
#define USART3_VOFA_TX_MAX         (256U)

/** Max channels in one JustFloat frame (limited by TX buffer) */
#define USART3_VOFA_MAX_CHANNELS   (16U)

/** Default scale: int32 x 0.001 (mV->V, mA->A, mdeg->deg, ...) */
#define USART3_VOFA_SCALE_MILLI    (0.001f)

/** VOFA+ command tail bytes */
#define USART3_VOFA_CMD_TAIL       {0xAF, 0xFA}

/** JustFloat frame tail (IEEE754 +Infinity) */
#define USART3_VOFA_JF_TAIL        {0x00, 0x00, 0x80, 0x7F}
#define USART3_VOFA_JF_TAIL_LEN    (4U)

/*******************************************************************************
 * Public API — Init / DeInit
 ******************************************************************************/

void Usart3_Vofa_Init(const Usart3_HW_Config_t *hw_cfg);
void Usart3_Vofa_DeInit(void);

/*******************************************************************************
 * Public API — TX
 ******************************************************************************/

/** JustFloat: send raw float array (DMA non-blocking) */
bool Usart3_Vofa_SendFloats(const float *data, uint8_t count);

/** JustFloat: send int32 array with on-the-fly scale (DMA non-blocking) */
bool Usart3_Vofa_SendScaled(const int32_t *data, uint8_t count, float scale);

/** FireWater: printf-style CSV string (DMA non-blocking) */
bool Usart3_Vofa_Printf(const char *format, ...);

/** Raw bytes (DMA non-blocking) */
bool Usart3_Vofa_SendRaw(const uint8_t *data, uint16_t len);

/** Check if DMA TX in progress */
bool Usart3_Vofa_IsTxBusy(void);

/*******************************************************************************
 * Public API — RX
 ******************************************************************************/

uint16_t Usart3_Vofa_RxAvailable(void);
uint16_t Usart3_Vofa_ReadRaw(uint8_t *buf, uint16_t max_len);

/** Read command frame: scans for tail {0xAF, 0xFA} */
uint16_t Usart3_Vofa_ReadCmd(uint8_t *buf, uint16_t max_len);

/** Read one line (up to '\\n') */
uint16_t Usart3_Vofa_ReadLine(uint8_t *buf, uint16_t max_len);

/*******************************************************************************
 * Official Vofa.c bridge
 ******************************************************************************/

/**
 * @brief  Move all available bytes from Usart3_IO ring_buf into Vofa FIFO.
 *         Call in main loop before Vofa_ReadCmd / Vofa_ReadLine.
 */
void Usart3_Vofa_FeedRx(Vofa_HandleTypedef *handle);

#ifdef __cplusplus
}
#endif

#endif /* __USART3_VOFA_H__ */
