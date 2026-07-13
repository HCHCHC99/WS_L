/**
 *******************************************************************************
 * @file  Vofa_Bridge.h
 * @brief Bridge between official Vofa.c and our Usart3_IO stack.
 *
 * Implements Vofa_SendDataCallBack / Vofa_GetDataCallBack using Usart3_IO.
 * Adds Vofa_Bridge_FeedRx() to move data from ring_buf → Vofa FIFO.
 *
 * Usage:
 * @code
 *   Vofa_HandleTypedef vofa1;
 *
 *   // Init
 *   Usart3_Vofa_Init(NULL);
 *   Vofa_Init(&vofa1, VOFA_MODE_SKIP);
 *
 *   // In main loop:
 *   Vofa_Bridge_FeedRx(&vofa1);           // ring_buf → Vofa FIFO
 *   uint16_t n = Vofa_ReadCmd(&vofa1, cmd, sizeof(cmd));
 *
 *   // Send JustFloat
 *   float data[3] = {1.0f, 2.0f, 3.0f};
 *   Vofa_JustFloat(&vofa1, data, 3);
 * @endcode
 *******************************************************************************
 */

#ifndef __VOFA_BRIDGE_H__
#define __VOFA_BRIDGE_H__

#include <stdint.h>
#include "Vofa.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Feed all available bytes from Usart3_IO ring_buf into Vofa FIFO.
 *         Call this regularly in main loop before using Vofa_ReadCmd/ReadLine.
 * @param  handle  Pointer to Vofa handle
 */
void Vofa_Bridge_FeedRx(Vofa_HandleTypedef *handle);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_BRIDGE_H__ */
