#ifndef __RS485_H__
#define __RS485_H__

#include "hc32_ll.h"
#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

/*=============================================================================
 * Debug macros (controlled by rtt_manager.h)
 *============================================================================*/
#ifdef ADP_RS485_DEBUG
    #define RS485_DEBUG(fmt, ...)    MAIN_D("[RS485] " fmt, ##__VA_ARGS__)
#else
    #define RS485_DEBUG(fmt, ...)    ((void)0)
#endif

#ifdef ADP_RS485_ERR_DEBUG
    #define RS485_ERR_DBG(fmt, ...)  MAIN_D("[RS485_ERR] " fmt, ##__VA_ARGS__)
#else
    #define RS485_ERR_DBG(fmt, ...)  ((void)0)
#endif

/*=============================================================================
 * Baudrate constants
 *============================================================================*/
#define RS485_BAUDRATE_1200     (1200UL)
#define RS485_BAUDRATE_2400     (2400UL)
#define RS485_BAUDRATE_4800     (4800UL)
#define RS485_BAUDRATE_9600     (9600UL)
#define RS485_BAUDRATE_14400    (14400UL)
#define RS485_BAUDRATE_19200    (19200UL)
#define RS485_BAUDRATE_38400    (38400UL)
#define RS485_BAUDRATE_115200   (115200UL)

/*=============================================================================
 * Hardware configuration
 *============================================================================*/
typedef struct {
    uint32_t baudrate;       /* baudrate, e.g. RS485_BAUDRATE_9600 */
    uint8_t  dir_polarity;   /* 0=high-send/low-recv, 1=low-send/high-recv */
} RS485_HW_Config_t;

/*=============================================================================
 * Callback types
 *============================================================================*/
typedef void (*RS485_RxCallback_t)(uint8_t byte);
typedef void (*RS485_TxReadyCallback_t)(void);
typedef void (*RS485_TxCompleteCallback_t)(void);
typedef void (*RS485_ErrorCallback_t)(void);

/*=============================================================================
 * Public API
 *============================================================================*/

void RS485_HW_Init(const RS485_HW_Config_t *cfg);
void RS485_HW_DeInit(void);
void RS485_HW_SetBaudrate(uint32_t baudrate);

void RS485_HW_SendByte(uint8_t byte);
void RS485_HW_StartTx(void);
void RS485_HW_StartRx(void);

void RS485_HW_RegisterRxCallback(RS485_RxCallback_t cb);
void RS485_HW_RegisterTxReadyCallback(RS485_TxReadyCallback_t cb);
void RS485_HW_RegisterTxCompleteCallback(RS485_TxCompleteCallback_t cb);
void RS485_HW_RegisterErrorCallback(RS485_ErrorCallback_t cb);

/* Interrupt control (for HAL TX flow) */
void RS485_HW_EnableTxEmptyInt(void);
void RS485_HW_EnableTxCompleteInt(void);

#endif /* __RS485_H__ */
