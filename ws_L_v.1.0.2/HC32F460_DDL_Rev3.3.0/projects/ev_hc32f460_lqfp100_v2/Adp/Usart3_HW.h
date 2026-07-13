/**
 *******************************************************************************
 * @file  Usart3_HW.h
 * @brief USART3 hardware abstraction layer (GPIO / clock / USART / DMA / IRQ)
 *
 * Responsibilities:
 *   - Pin mux: PB12=USART3_RX(FUNC_33)  PB13=USART3_TX(FUNC_32)
 *   - USART clock, baudrate, UART mode
 *   - DMA2 single-shot TX (triggered by USART3_TI via AOS)
 *   - RX interrupt → per-byte callback
 *   - DMA-TC interrupt → TX-done callback
 *
 * NOT responsible for:
 *   - Buffer management, ring buffers, frame assembly, VOFA+ protocol
 *     → those belong to Usart3_IO.h
 *
 * Modeled after Adp/rs485.h
 *******************************************************************************
 */

#ifndef __USART3_HW_H__
#define __USART3_HW_H__

#include <stdint.h>
#include <stdbool.h>
#include "hc32_ll.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration structure
 ******************************************************************************/

typedef struct {
    uint32_t baudrate;          /* e.g. 115200 for VOFA+ */
    uint8_t  rx_port;           /* GPIO_PORT_x   (default GPIO_PORT_B)  */
    uint16_t rx_pin;            /* GPIO_PIN_xx   (default GPIO_PIN_12)  */
    uint8_t  rx_func;           /* GPIO_FUNC_xx  (default GPIO_FUNC_33) */
    uint8_t  tx_port;           /* GPIO_PORT_x   (default GPIO_PORT_B)  */
    uint16_t tx_pin;            /* GPIO_PIN_xx   (default GPIO_PIN_13)  */
    uint8_t  tx_func;           /* GPIO_FUNC_xx  (default GPIO_FUNC_32) */
} Usart3_HW_Config_t;

/*******************************************************************************
 * Callback types
 ******************************************************************************/

typedef void (*Usart3_HW_RxCallback_t)(uint8_t byte);
typedef void (*Usart3_HW_TxDoneCallback_t)(void);
typedef void (*Usart3_HW_ErrorCallback_t)(void);

/*******************************************************************************
 * Public API
 ******************************************************************************/

/**
 * @brief  Initialize USART3 hardware, DMA, AOS, and interrupts.
 * @param  cfg  Pointer to config struct (NULL = use defaults)
 * @note   Must be called before any other Usart3_HW functions.
 *         Does NOT enable TX or RX yet — call StartRx/StartTxDma.
 */
void Usart3_HW_Init(const Usart3_HW_Config_t *cfg);

/**
 * @brief  De-initialize hardware (disable clocks, DMA, IRQ).
 */
void Usart3_HW_DeInit(void);

/* ---- RX ---- */

/**
 * @brief  Enable RX + RX interrupt (received bytes fire RxCallback).
 */
void Usart3_HW_StartRx(void);

/**
 * @brief  Disable RX + RX interrupt.
 */
void Usart3_HW_StopRx(void);

/* ---- TX (DMA, single-shot) ---- */

/**
 * @brief  Start a DMA transfer. Returns immediately; TxDoneCallback fires when done.
 * @param  data  Buffer pointer (caller must keep valid until TxDoneCallback!)
 * @param  len   Number of bytes (1 ~ 256)
 * @return true if started, false if DMA busy or invalid params
 */
bool Usart3_HW_StartTxDma(const uint8_t *data, uint16_t len);

/**
 * @brief  Check if DMA TX is still in progress.
 */
bool Usart3_HW_IsTxBusy(void);

/* ---- TX (blocking polling, debug only) ---- */

/**
 * @brief  Send bytes by polling TXE flag (BLOCKING, no DMA/IRQ).
 * @note   Bypasses DMA. Only for debug/testing.
 */
void Usart3_HW_SendBlocking(const uint8_t *data, uint16_t len);

/* ---- Callback registration ---- */

void Usart3_HW_RegisterRxCallback(Usart3_HW_RxCallback_t cb);
void Usart3_HW_RegisterTxDoneCallback(Usart3_HW_TxDoneCallback_t cb);
void Usart3_HW_RegisterErrorCallback(Usart3_HW_ErrorCallback_t cb);

#ifdef __cplusplus
}
#endif

#endif /* __USART3_HW_H__ */
