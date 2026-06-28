#ifndef __COMM_HAL_H__
#define __COMM_HAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "rs485.h"
#include "rtt_manager.h"

/*=============================================================================
 * Debug macros
 *============================================================================*/
#ifdef ADP_COMM_HAL_DEBUG
    #define HAL_DEBUG(fmt, ...)    MAIN_D("[COMM_HAL] " fmt, ##__VA_ARGS__)
#else
    #define HAL_DEBUG(fmt, ...)    ((void)0)
#endif

#ifdef ADP_COMM_HAL_WARN
    #define HAL_WARN(fmt, ...)     MAIN_D("[COMM_HAL_WARN] " fmt, ##__VA_ARGS__)
#else
    #define HAL_WARN(fmt, ...)     ((void)0)
#endif

#ifdef ADP_COMM_HAL_ERR
    #define HAL_ERR(fmt, ...)      MAIN_D("[COMM_HAL_ERR] " fmt, ##__VA_ARGS__)
#else
    #define HAL_ERR(fmt, ...)      ((void)0)
#endif

/*=============================================================================
 * HAL configuration
 *============================================================================*/
typedef struct {
    uint16_t rx_buf_size;           /* RX ring buffer size (default 500) */
    uint16_t tx_buf_size;           /* TX ring buffer size (default 500) */
    uint8_t  rx_frame_queue_depth;  /* RX frame queue depth (default 10) */
    uint8_t  tx_queue_depth;        /* TX queue depth (default 10) */
    uint8_t  frame_timeout_ms;      /* Inter-frame timeout in ms (0 = auto from baudrate) */
} Comm_HAL_Config_t;

/*=============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief Initialize HAL with hardware and buffer configuration
 */
void Comm_HAL_Init(const RS485_HW_Config_t *hw_cfg, const Comm_HAL_Config_t *hal_cfg);

/**
 * @brief Send data (non-blocking, queues if busy)
 * @return true if sent or queued, false if queue full
 */
bool Comm_HAL_Send(const uint8_t *data, uint16_t len);

/**
 * @brief Receive a complete frame (non-blocking)
 * @return true if a frame was dequeued
 */
bool Comm_HAL_RecvFrame(uint8_t *buf, uint16_t *len, uint16_t max_len);

/**
 * @brief Main loop poll: frame assembly + TX queue drain
 */
void Comm_HAL_Poll(void);

/**
 * @brief Get idle time since last received byte (ms)
 */
uint32_t Comm_HAL_GetIdleTime(void);

/**
 * @brief Status queries
 */
uint16_t Comm_HAL_GetTxQueueCount(void);
bool Comm_HAL_IsBusy(void);

/**
 * @brief Get the configured inter-frame timeout (ms)
 */
uint8_t Comm_HAL_GetFrameTimeout(void);

#endif /* __COMM_HAL_H__ */
