/**
 *******************************************************************************
 * @file  Usart3_Vofa.h
 * @brief USART3 + VOFA+ protocol wrapper.
 *
 * Sits on top of Usart3_IO and adds:
 *   - FireWater (printf-style CSV) sending
 *   - Command-frame RX with configurable tail (e.g. {0xAF, 0xFA})
 *   - Line-based RX
 *   - JustFloat with int32_t scaled input (fixed-point, ×1000 convention)
 *   - RawData pass-through to Usart3_IO
 *
 * Does NOT touch USART/DMA registers directly — delegates to Usart3_IO.
 * Does NOT use VOFA driver's internal FIFO — uses Usart3_IO ring_buf.
 *
 * Scaled-integer convention:
 *   All physical quantities are stored as int32_t × 1000 on MCU.
 *   Usart3_Vofa_SendScaled(..., 0.001f) converts to float on-the-fly
 *   (single-cycle FPU instruction per channel on Cortex-M4F), then sends
 *   as standard JustFloat binary.  VOFA+ receives clean float values
 *   — no manual division needed.
 *
 * Typical usage:
 * @code
 *   Usart3_Vofa_Init(NULL);                            // 115200, PB12/PB13
 *
 *   // --- FireWater (text, debug) ---
 *   Usart3_Vofa_Printf("id:%d,iq:%d,vbus:%d\r\n", id_i32, iq_i32, vbus_i32);
 *
 *   // --- JustFloat (binary, high-frequency 9-ch motor data) ---
 *   int32_t buf[9] = { ia_mA, ib_mA, ic_mA, vbus_mV, angle_mdeg, ... };
 *   if (!Usart3_Vofa_IsTxBusy())
 *       Usart3_Vofa_SendScaled(buf, 9, 0.001f);        // ×0.001 → VOFA+ sees A, V, deg
 *
 *   // --- Command parsing ---
 *   uint8_t cmd[32];
 *   uint16_t n = Usart3_Vofa_ReadCmd(cmd, sizeof(cmd));
 *   if (n >= 4 && cmd[n-2] == 0xAF && cmd[n-1] == 0xFA) {
 *       // process command
 *   }
 * @endcode
 *******************************************************************************
 */

#ifndef __USART3_VOFA_H__
#define __USART3_VOFA_H__

#include <stdint.h>
#include <stdbool.h>
#include "Usart3_HW.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/** Max single TX frame (matches Usart3_IO TX buffer) */
#define USART3_VOFA_TX_MAX         (256U)

/** Max channels in one JustFloat frame (limited by TX buffer: 252/4 = 63) */
#define USART3_VOFA_MAX_CHANNELS   (16U)

/** Default scale: int32 × 0.001 → float (mV→V, mA→A, mdeg→deg, ...) */
#define USART3_VOFA_SCALE_MILLI    (0.001f)

/** VOFA+ command tail bytes */
#define USART3_VOFA_CMD_TAIL       {0xAF, 0xFA}

/*******************************************************************************
 * Public API — Init / DeInit
 ******************************************************************************/

/**
 * @brief  Initialize VOFA wrapper: Usart3_IO init + RX start.
 * @param  hw_cfg  NULL = defaults (115200, PB12 RX, PB13 TX)
 */
void Usart3_Vofa_Init(const Usart3_HW_Config_t *hw_cfg);

/**
 * @brief  De-initialize (stops USART, DMA, IRQ).
 */
void Usart3_Vofa_DeInit(void);

/*******************************************************************************
 * Public API — TX
 ******************************************************************************/

/**
 * @brief  Send int32_t array as JustFloat, with on-the-fly scaling.
 *
 *         Converts each element:  float_val = (float)data[i] * scale
 *         then sends in standard JustFloat binary format (little-endian
 *         float32 + 4-byte tail).  Single-cycle FPU conversion per channel.
 *
 * @param  data    Int32 array (values pre-multiplied by 1000, e.g. 1234 = 1.234A)
 * @param  count   Number of channels (1 ~ USART3_VOFA_MAX_CHANNELS)
 * @param  scale   Scaling factor (e.g. 0.001f to convert mA→A, mV→V)
 * @return true if DMA started, false if busy or invalid params
 *
 * @note   DMA non-blocking.  Caller should check Usart3_Vofa_IsTxBusy()
 *         before next send.
 *
 * Typical:
 * @code
 *   int32_t buf[9] = {ia_mA, ib_mA, ic_mA, vbus_mV, ...};
 *   Usart3_Vofa_SendScaled(buf, 9, 0.001f);
 * @endcode
 */
bool Usart3_Vofa_SendScaled(const int32_t *data, uint8_t count, float scale);

/**
 * @brief  FireWater: printf-style CSV string (DMA, non-blocking).
 * @param  format  printf format string (e.g. "temp:%.2f,adc:%d\r\n")
 * @return true if DMA started, false if busy or format error
 * @note   Max formatted length = USART3_VOFA_TX_MAX. Caller should
 *         check Usart3_Vofa_IsTxBusy() before calling again.
 */
bool Usart3_Vofa_Printf(const char *format, ...);

/**
 * @brief  Send raw bytes (DMA, non-blocking).
 * @param  data  Buffer (copied internally, caller may reuse immediately)
 * @param  len   Byte count (1 ~ 256)
 * @return true if DMA started
 */
bool Usart3_Vofa_SendRaw(const uint8_t *data, uint16_t len);

/**
 * @brief  Check if DMA TX is still in progress.
 */
bool Usart3_Vofa_IsTxBusy(void);

/*******************************************************************************
 * Public API — RX
 ******************************************************************************/

/**
 * @brief  Number of bytes available in RX ring buffer.
 */
uint16_t Usart3_Vofa_RxAvailable(void);

/**
 * @brief  Read raw bytes from RX ring buffer (non-blocking).
 * @return Number of bytes actually read.
 */
uint16_t Usart3_Vofa_ReadRaw(uint8_t *buf, uint16_t max_len);

/**
 * @brief  Read one command frame from RX ring buffer.
 *         A command is delimited by tail bytes {0xAF, 0xFA}.
 * @param  buf      Output buffer (receives data INCLUDING tail bytes)
 * @param  max_len  Max bytes to read
 * @return Number of bytes read (0 if no data or partial tail only).
 *         Check: if return > 0 and last 2 bytes == {0xAF, 0xFA}, complete.
 * @note   Non-blocking — reads whatever is available.
 */
uint16_t Usart3_Vofa_ReadCmd(uint8_t *buf, uint16_t max_len);

/**
 * @brief  Read one line (up to '\n') from RX ring buffer.
 * @param  buf      Output buffer (receives data INCLUDING '\n' if present)
 * @param  max_len  Max bytes to read
 * @return Number of bytes read (0 if no data).
 * @note   Non-blocking — if '\n' not yet received, returns partial.
 */
uint16_t Usart3_Vofa_ReadLine(uint8_t *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* __USART3_VOFA_H__ */
