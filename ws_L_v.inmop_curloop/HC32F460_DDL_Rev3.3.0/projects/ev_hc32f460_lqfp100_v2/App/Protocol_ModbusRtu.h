#ifndef __PROTOCOL_MODBUSRTU_H__
#define __PROTOCOL_MODBUSRTU_H__

#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

/*=============================================================================
 * Debug macros
 *============================================================================*/
#ifdef APP_PROTO_MODBUS_CRC_DBG
    #define MODBUS_CRC_DBG(fmt, ...)    MAIN_D("[MODBUS_CRC] " fmt, ##__VA_ARGS__)
#else
    #define MODBUS_CRC_DBG(fmt, ...)    ((void)0)
#endif

#ifdef APP_PROTO_MODBUS_PARSE_DBG
    #define MODBUS_PARSE_DBG(fmt, ...)  MAIN_D("[MODBUS_PARSE] " fmt, ##__VA_ARGS__)
#else
    #define MODBUS_PARSE_DBG(fmt, ...)  ((void)0)
#endif

#ifdef APP_PROTO_MODBUS_RX_DBG
    #define MODBUS_RX_DBG(fmt, ...)     MAIN_D("[MODBUS_RX] " fmt, ##__VA_ARGS__)
#else
    #define MODBUS_RX_DBG(fmt, ...)     ((void)0)
#endif

/*=============================================================================
 * Modbus constants
 *============================================================================*/
#define MODBUS_FRAME_FIX_LEN        (8U)
#define MODBUS_RESP_BUF_SIZE        (256U)

/*=============================================================================
 * Modbus function codes
 *============================================================================*/
#define MODBUS_FUNC_READ_HOLDING    (0x03U)
#define MODBUS_FUNC_WRITE_SINGLE    (0x06U)
#define MODBUS_FUNC_WRITE_MULTI     (0x10U)

/*=============================================================================
 * Modbus exception codes
 *============================================================================*/
#define MODBUS_EXCEPTION_ILLEGAL_FUNC       (0x01U)
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR  (0x02U)
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VAL   (0x03U)
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAIL  (0x04U)
#define MODBUS_EXCEPTION_ACKNOWLEDGE        (0x05U)
#define MODBUS_EXCEPTION_SLAVE_BUSY         (0x06U)

/*=============================================================================
 * Big-endian conversion helper
 *============================================================================*/
#define MODBUS_BE16(val)    (uint16_t)((((val) >> 8U) & 0xFFU) | (((val) & 0xFFU) << 8U))

/*=============================================================================
 * Modbus frame union (packed for overlay on raw bytes)
 *============================================================================*/
#pragma pack(push, 1)
typedef union {
    uint8_t raw[256];

    struct {
        uint8_t addr;
        uint8_t func;
    } header;

    struct {
        uint8_t  addr;
        uint8_t  func;
        uint16_t start_reg;
        uint16_t reg_count;
    } read_holding;

    struct {
        uint8_t  addr;
        uint8_t  func;
        uint16_t reg_addr;
        uint16_t reg_value;
    } write_single;

    struct {
        uint8_t  addr;
        uint8_t  func;
        uint16_t start_reg;
        uint16_t reg_count;
        uint8_t  byte_count;
        uint8_t  data[252];
    } write_multi;

    struct {
        uint8_t  addr;
        uint8_t  func;
        uint8_t  exception;
    } exception;
} ModbusFrame_t;
#pragma pack(pop)

/*=============================================================================
 * Protocol configuration
 *============================================================================*/
typedef struct {
    uint8_t  node_id;
    bool     enable_write_multi;
} ModbusRTU_Config_t;

/*=============================================================================
 * Callback types (implemented by application layer)
 *============================================================================*/
typedef bool (*ModbusRTU_ReadRegCallback_t)(uint16_t reg_addr, uint16_t *value);
typedef bool (*ModbusRTU_WriteRegCallback_t)(uint16_t reg_addr, uint16_t value);
typedef bool (*ModbusRTU_WriteMultiCallback_t)(uint16_t startReg, uint16_t regCount, const uint8_t *pData);
typedef bool (*ModbusRTU_ValidateRegCallback_t)(uint16_t reg_addr);

typedef struct {
    ModbusRTU_ReadRegCallback_t      on_read;
    ModbusRTU_WriteRegCallback_t     on_write;
    ModbusRTU_WriteMultiCallback_t   on_write_multi; /* NULL = fallback to per-register on_write */
    ModbusRTU_ValidateRegCallback_t  on_validate;
} ModbusRTU_Callbacks_t;

/*=============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief Initialize Modbus RTU protocol engine
 */
void ModbusRTU_Init(const ModbusRTU_Config_t *cfg, const ModbusRTU_Callbacks_t *cbs);

/**
 * @brief Process a raw Modbus frame (CRC check, dispatch, respond via HAL)
 */
void ModbusRTU_ProcessFrame(const uint8_t *frame, uint16_t len);

/**
 * @brief Calculate Modbus CRC16 over given data
 */
uint16_t ModbusRTU_CalcCrc16(const uint8_t *pData, uint16_t len);

#endif /* __PROTOCOL_MODBUSRTU_H__ */
