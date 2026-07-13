/**
 *******************************************************************************
 * @file  Usart3_HW.h
 * @brief Generic UART + DMA hardware abstraction layer.
 *
 * Originally hardcoded to USART3/DMA2.  Now fully configurable via struct —
 * same code works with any USARTx + DMAx combination by passing a different
 * config.  NULL = use default (USART3, PB12/PB13, DMA2 CH0, 115200).
 *
 * Responsibilities:
 *   - GPIO pin mux
 *   - USART clock, baudrate, UART mode
 *   - DMA single-shot TX (triggered by USART_TI via AOS)
 *   - RX interrupt -> per-byte callback
 *   - DMA-TC interrupt -> TX-done callback
 *
 * To switch USART:  fill in a Usart3_HW_Config_t and pass to Init().
 * To switch DMA:    same struct — change dma_base / dma_ch / aos fields.
 * To switch pins:   change rx_/tx_ GPIO fields.
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
 * Configuration structure — fill in to target any USART + DMA combination
 ******************************************************************************/

typedef struct {
    /* ---- GPIO ---- */
    uint8_t     rx_port;        /* GPIO_PORT_x                                  */
    uint16_t    rx_pin;         /* GPIO_PIN_xx                                  */
    uint8_t     rx_func;        /* GPIO_FUNC_xx (AF for this USART RX)          */
    uint8_t     tx_port;        /* GPIO_PORT_x                                  */
    uint16_t    tx_pin;         /* GPIO_PIN_xx                                  */
    uint8_t     tx_func;        /* GPIO_FUNC_xx (AF for this USART TX)          */

    /* ---- USART ---- */
    uint32_t    baudrate;       /* e.g. 115200, 921600                          */
    uint32_t    fcg_periph;     /* FCG1_PERIPH_USARTx (clock gate)              */
    CM_USART_TypeDef *usart_base;  /* CM_USARTx base address                    */

    /* ---- DMA ---- */
    CM_DMA_TypeDef *dma_base;   /* CM_DMAx base address                         */
    uint8_t     dma_ch;         /* DMA_CH0 ~ DMA_CH3                            */
    uint32_t    dma_fcg;        /* FCG0_PERIPH_DMAx (clock gate)                */
    uint32_t    aos_target;     /* AOS_DMAx_y (trigger target)                  */
    uint32_t    aos_event;      /* EVT_SRC_USARTx_TI (trigger source)           */
    uint32_t    dma_tc_flag;    /* DMA_FLAG_TC_CHx (for status clear)           */
    uint32_t    dma_tc_int;     /* DMA_INT_TC_CHx (interrupt enable)            */

    /* ---- IRQ ---- */
    IRQn_Type   rx_err_irqn;    /* e.g. INT004_IRQn                             */
    en_int_src_t rx_err_int_src;/* e.g. INT_SRC_USARTx_EI                       */
    IRQn_Type   rx_full_irqn;   /* e.g. INT005_IRQn                             */
    en_int_src_t rx_full_int_src;/* e.g. INT_SRC_USARTx_RI                      */
    IRQn_Type   dma_tc_irqn;    /* e.g. INT042_IRQn                             */
    en_int_src_t dma_tc_int_src; /* e.g. INT_SRC_DMAx_TCy                       */
} Usart3_HW_Config_t;

/*******************************************************************************
 * Default configs (pass to Init or copy as starting point)
 ******************************************************************************/

/** USART3, PB12(RX)/PB13(TX), DMA2 CH0, 115200 — current project default */
extern const Usart3_HW_Config_t USART3_HW_CONFIG_DEFAULT;

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
 * @brief  Initialize UART hardware, DMA, AOS, and interrupts from config.
 * @param  cfg  NULL = use USART3_HW_CONFIG_DEFAULT (USART3, 115200, DMA2 CH0)
 */
void Usart3_HW_Init(const Usart3_HW_Config_t *cfg);

void Usart3_HW_DeInit(void);

void Usart3_HW_StartRx(void);
void Usart3_HW_StopRx(void);

bool Usart3_HW_StartTxDma(const uint8_t *data, uint16_t len);
bool Usart3_HW_IsTxBusy(void);

void Usart3_HW_SendBlocking(const uint8_t *data, uint16_t len);

void Usart3_HW_RegisterRxCallback(Usart3_HW_RxCallback_t cb);
void Usart3_HW_RegisterTxDoneCallback(Usart3_HW_TxDoneCallback_t cb);
void Usart3_HW_RegisterErrorCallback(Usart3_HW_ErrorCallback_t cb);

#ifdef __cplusplus
}
#endif

#endif /* __USART3_HW_H__ */
