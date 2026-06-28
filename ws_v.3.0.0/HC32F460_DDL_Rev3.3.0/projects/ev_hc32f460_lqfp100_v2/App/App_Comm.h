#ifndef __APP_COMM_H__
#define __APP_COMM_H__

#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

/*=============================================================================
 * Debug macros
 *============================================================================*/
#ifdef APP_COMM_DBG
    #define COMM_DBG(fmt, ...)     MAIN_D("[APP_COMM] " fmt, ##__VA_ARGS__)
#else
    #define COMM_DBG(fmt, ...)     ((void)0)
#endif

/*=============================================================================
 * Physical layer config (passed through to RS485 driver)
 *============================================================================*/
typedef struct {
    uint32_t baudrate;
    uint8_t  dir_polarity;   /* 0=high-send/low-recv, 1=low-send/high-recv */
} App_Comm_PhyConfig_t;

/*=============================================================================
 * HAL layer config (buffer sizes, queue depths, frame timeout)
 *============================================================================*/
typedef struct {
    uint16_t rx_buf_size;
    uint16_t tx_buf_size;
    uint8_t  rx_frame_queue_depth;
    uint8_t  tx_queue_depth;
    uint8_t  frame_timeout_ms;    /* 0 = auto-calculate from baudrate */
} App_Comm_HALConfig_t;

/*=============================================================================
 * Protocol layer config
 *============================================================================*/
typedef struct {
    uint8_t node_id;
    bool    enable_write_multi;
} App_Comm_ProtoConfig_t;

/*=============================================================================
 * Aggregated config
 *============================================================================*/
typedef struct {
    App_Comm_PhyConfig_t    phy;
    App_Comm_HALConfig_t    hal;
    App_Comm_ProtoConfig_t  proto;
} App_Comm_Config_t;

/*=============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief Initialize the complete communication stack
 *        (RS485 HW -> HAL -> ModbusRTU -> App callbacks)
 */
void App_Comm_Init(const App_Comm_Config_t *cfg);

/**
 * @brief Main loop poll: HAL frame assembly + TX drain + frame processing
 */
void App_Comm_Poll(void);

#endif /* __APP_COMM_H__ */
