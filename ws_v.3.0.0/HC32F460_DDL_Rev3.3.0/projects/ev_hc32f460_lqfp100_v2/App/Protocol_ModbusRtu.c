#include "Protocol_ModbusRtu.h"
#include "Comm_HAL.h"
#include <string.h>

/*=============================================================================
 * Static data
 *============================================================================*/
static ModbusRTU_Config_t    m_stcConfig;
static ModbusRTU_Callbacks_t m_stcCallbacks;
static ModbusFrame_t         m_stcTxFrame;

/*=============================================================================
 * Forward declarations
 *============================================================================*/
static void ModbusRTU_SendResponse(const uint8_t *pBuf, uint16_t len);
static void ModbusRTU_SendException(uint8_t addr, uint8_t func, uint8_t exceptionCode);
static void ModbusRTU_HandleReadHolding(uint8_t addr, uint16_t startReg, uint16_t regCount);
static void ModbusRTU_HandleWriteSingle(uint8_t addr, uint16_t regAddr, uint16_t regValue);
static void ModbusRTU_HandleWriteMulti(uint8_t addr, uint16_t startReg, uint16_t regCount, const uint8_t *pData);

/*=============================================================================
 * CRC16 calculation
 *============================================================================*/
uint16_t ModbusRTU_CalcCrc16(const uint8_t *pData, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i, j;

    MODBUS_CRC_DBG("CRC calc start, len=%d", (int)len);

    for (i = 0U; i < len; i++)
    {
        crc ^= pData[i];
        MODBUS_CRC_DBG("  byte[%d]=0x%02X, crc=0x%04X", (int)i, pData[i], crc);
        for (j = 0U; j < 8U; j++)
        {
            if (crc & 0x0001U)
            {
                crc >>= 1U;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
        MODBUS_CRC_DBG("  after bit loop[%d], crc=0x%04X", (int)i, crc);
    }

    MODBUS_CRC_DBG("CRC calc done, result=0x%04X", crc);
    return crc;
}

/*=============================================================================
 * Send response via HAL
 *============================================================================*/
static void ModbusRTU_SendResponse(const uint8_t *pBuf, uint16_t len)
{
    uint16_t crc;
    bool     bRet;

    crc = ModbusRTU_CalcCrc16(pBuf, len);
    m_stcTxFrame.raw[len]      = (uint8_t)(crc & 0xFFU);
    m_stcTxFrame.raw[len + 1U] = (uint8_t)((crc >> 8U) & 0xFFU);

    memcpy(m_stcTxFrame.raw, pBuf, len);

    MODBUS_PARSE_DBG("send resp, len=%d, crc=0x%04X", (int)(len + 2U), crc);

    bRet = Comm_HAL_Send(m_stcTxFrame.raw, len + 2U);
    if (!bRet)
    {
        MAIN_D("[MODBUS_ERR] Comm_HAL_Send FAILED! len=%d", (int)(len + 2U));
    }
}

/*=============================================================================
 * Send exception response
 *============================================================================*/
static void ModbusRTU_SendException(uint8_t addr, uint8_t func, uint8_t exceptionCode)
{
    uint8_t buf[3];

    buf[0] = addr;
    buf[1] = func | 0x80U;
    buf[2] = exceptionCode;

    MODBUS_PARSE_DBG("send exception: addr=0x%02X, func=0x%02X, code=0x%02X",
                     addr, func, exceptionCode);

    ModbusRTU_SendResponse(buf, 3U);
}

/*=============================================================================
 * Handle read holding registers (function 0x03)
 *============================================================================*/
static void ModbusRTU_HandleReadHolding(uint8_t addr, uint16_t startReg, uint16_t regCount)
{
    uint16_t i;
    uint16_t regAddr;
    uint16_t regValue;
    uint16_t byteCount;

    MODBUS_PARSE_DBG("read holding: start=0x%04X, count=%d", startReg, (int)regCount);

    if (regCount == 0U || regCount > 125U)
    {
        MODBUS_PARSE_DBG("  regCount invalid: %d", (int)regCount);
        ModbusRTU_SendException(addr, MODBUS_FUNC_READ_HOLDING, MODBUS_EXCEPTION_ILLEGAL_DATA_VAL);
        return;
    }

    /* Validate all registers before reading */
    for (i = 0U; i < regCount; i++)
    {
        regAddr = startReg + i;
        if (m_stcCallbacks.on_validate && !m_stcCallbacks.on_validate(regAddr))
        {
            MODBUS_PARSE_DBG("  invalid reg addr: 0x%04X", regAddr);
            ModbusRTU_SendException(addr, MODBUS_FUNC_READ_HOLDING, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR);
            return;
        }
    }

    byteCount = (uint16_t)(regCount * 2U);
    m_stcTxFrame.raw[0] = addr;
    m_stcTxFrame.raw[1] = MODBUS_FUNC_READ_HOLDING;
    m_stcTxFrame.raw[2] = (uint8_t)byteCount;

    for (i = 0U; i < regCount; i++)
    {
        regAddr = startReg + i;

        if (m_stcCallbacks.on_read && m_stcCallbacks.on_read(regAddr, &regValue))
        {
            m_stcTxFrame.raw[3U + i * 2U]      = (uint8_t)((regValue >> 8U) & 0xFFU);
            m_stcTxFrame.raw[3U + i * 2U + 1U] = (uint8_t)(regValue & 0xFFU);
            MODBUS_PARSE_DBG("  reg[0x%04X]=0x%04X", regAddr, regValue);
        }
        else
        {
            MODBUS_PARSE_DBG("  on_read failed: reg=0x%04X", regAddr);
            ModbusRTU_SendException(addr, MODBUS_FUNC_READ_HOLDING, MODBUS_EXCEPTION_SLAVE_DEVICE_FAIL);
            return;
        }
    }

    MODBUS_PARSE_DBG("read holding done, byteCount=%d", (int)byteCount);
    ModbusRTU_SendResponse(m_stcTxFrame.raw, 3U + byteCount);
}

/*=============================================================================
 * Handle write single register (function 0x06)
 *============================================================================*/
static void ModbusRTU_HandleWriteSingle(uint8_t addr, uint16_t regAddr, uint16_t regValue)
{
    MODBUS_PARSE_DBG("write single: reg=0x%04X, value=0x%04X", regAddr, regValue);

    if (m_stcCallbacks.on_validate && !m_stcCallbacks.on_validate(regAddr))
    {
        MODBUS_PARSE_DBG("  invalid reg addr: 0x%04X", regAddr);
        ModbusRTU_SendException(addr, MODBUS_FUNC_WRITE_SINGLE, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR);
        return;
    }

    if (m_stcCallbacks.on_write && !m_stcCallbacks.on_write(regAddr, regValue))
    {
        MODBUS_PARSE_DBG("  on_write failed: reg=0x%04X", regAddr);
        ModbusRTU_SendException(addr, MODBUS_FUNC_WRITE_SINGLE, MODBUS_EXCEPTION_SLAVE_DEVICE_FAIL);
        return;
    }

    /* Echo response */
    m_stcTxFrame.write_single.addr      = addr;
    m_stcTxFrame.write_single.func      = MODBUS_FUNC_WRITE_SINGLE;
    m_stcTxFrame.write_single.reg_addr  = MODBUS_BE16(regAddr);
    m_stcTxFrame.write_single.reg_value = MODBUS_BE16(regValue);

    ModbusRTU_SendResponse(m_stcTxFrame.raw, 6U);
    MODBUS_PARSE_DBG("write single OK");
}

/*=============================================================================
 * Handle write multiple registers (function 0x10)
 *============================================================================*/
static void ModbusRTU_HandleWriteMulti(uint8_t addr, uint16_t startReg, uint16_t regCount, const uint8_t *pData)
{
    uint16_t i;
    uint16_t regAddr;
    uint16_t regValue;

    MODBUS_PARSE_DBG("write multi: start=0x%04X, count=%d", startReg, (int)regCount);

    if (regCount == 0U || regCount > 123U)
    {
        MODBUS_PARSE_DBG("  invalid regCount: %d", (int)regCount);
        ModbusRTU_SendException(addr, MODBUS_FUNC_WRITE_MULTI, MODBUS_EXCEPTION_ILLEGAL_DATA_VAL);
        return;
    }

    /* Validate all registers first */
    for (i = 0U; i < regCount; i++)
    {
        regAddr = startReg + i;
        if (m_stcCallbacks.on_validate && !m_stcCallbacks.on_validate(regAddr))
        {
            MODBUS_PARSE_DBG("  invalid reg addr: 0x%04X", regAddr);
            ModbusRTU_SendException(addr, MODBUS_FUNC_WRITE_MULTI, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR);
            return;
        }
    }

    /* Write all registers — prefer batch callback for atomic Flash save */
    if (m_stcCallbacks.on_write_multi)
    {
        if (!m_stcCallbacks.on_write_multi(startReg, regCount, pData))
        {
            MODBUS_PARSE_DBG("  on_write_multi failed");
            ModbusRTU_SendException(addr, MODBUS_FUNC_WRITE_MULTI, MODBUS_EXCEPTION_SLAVE_DEVICE_FAIL);
            return;
        }
    }
    else
    {
        for (i = 0U; i < regCount; i++)
        {
            regAddr  = startReg + i;
            regValue = (uint16_t)((pData[i * 2U] << 8U) | pData[i * 2U + 1U]);

            if (m_stcCallbacks.on_write && !m_stcCallbacks.on_write(regAddr, regValue))
            {
                MODBUS_PARSE_DBG("  on_write failed: reg=0x%04X", regAddr);
                ModbusRTU_SendException(addr, MODBUS_FUNC_WRITE_MULTI, MODBUS_EXCEPTION_SLAVE_DEVICE_FAIL);
                return;
            }
        }
    }

    /* Response */
    m_stcTxFrame.write_multi.addr       = addr;
    m_stcTxFrame.write_multi.func       = MODBUS_FUNC_WRITE_MULTI;
    m_stcTxFrame.write_multi.start_reg  = MODBUS_BE16(startReg);
    m_stcTxFrame.write_multi.reg_count  = MODBUS_BE16(regCount);

    ModbusRTU_SendResponse(m_stcTxFrame.raw, 6U);
    MODBUS_PARSE_DBG("write multi OK");
}

/*=============================================================================
 * Process a received Modbus frame
 *============================================================================*/
void ModbusRTU_ProcessFrame(const uint8_t *frame, uint16_t len)
{
    uint16_t crcCalc, crcRecv;
    uint16_t dataLen;

    MODBUS_RX_DBG("frame recv, len=%d", (int)len);

    if (len < MODBUS_FRAME_FIX_LEN)
    {
        MODBUS_PARSE_DBG("frame too short, len=%d < min=%d", (int)len, (int)MODBUS_FRAME_FIX_LEN);
        return;
    }

    /* Copy frame for CRC calculation (frame may be in a temp buffer) */
    memcpy(m_stcTxFrame.raw, frame, len);

    uint8_t addr = m_stcTxFrame.header.addr;
    uint8_t func = m_stcTxFrame.header.func;
    dataLen = len - 4U;

    MODBUS_PARSE_DBG("addr=0x%02X, func=0x%02X, dataLen=%d", addr, func, (int)dataLen);

    /* Address check */
    if (addr != m_stcConfig.node_id)
    {
        MODBUS_PARSE_DBG("  addr mismatch: recv=0x%02X, local=0x%02X", addr, m_stcConfig.node_id);
        return;
    }

    /* CRC check */
    crcRecv = (uint16_t)(frame[len - 1U] << 8U) | frame[len - 2U];
    crcCalc = ModbusRTU_CalcCrc16(frame, len - 2U);

    if (crcCalc != crcRecv)
    {
        MODBUS_CRC_DBG("CRC mismatch! calc=0x%04X, recv=0x%04X", crcCalc, crcRecv);
        return;
    }

    MODBUS_CRC_DBG("CRC match OK, addr=0x%02X, func=0x%02X", addr, func);

    /* Dispatch by function code */
    switch (func)
    {
    case MODBUS_FUNC_READ_HOLDING:
    {
        if (dataLen < 4U)
        {
            ModbusRTU_SendException(addr, func, MODBUS_EXCEPTION_ILLEGAL_DATA_VAL);
            break;
        }
        uint16_t startReg = MODBUS_BE16(m_stcTxFrame.read_holding.start_reg);
        uint16_t regCount = MODBUS_BE16(m_stcTxFrame.read_holding.reg_count);
        ModbusRTU_HandleReadHolding(addr, startReg, regCount);
        break;
    }

    case MODBUS_FUNC_WRITE_SINGLE:
    {
        if (dataLen < 4U)
        {
            ModbusRTU_SendException(addr, func, MODBUS_EXCEPTION_ILLEGAL_DATA_VAL);
            break;
        }
        uint16_t regAddr  = MODBUS_BE16(m_stcTxFrame.write_single.reg_addr);
        uint16_t regValue = MODBUS_BE16(m_stcTxFrame.write_single.reg_value);
        ModbusRTU_HandleWriteSingle(addr, regAddr, regValue);
        break;
    }

    case MODBUS_FUNC_WRITE_MULTI:
    {
        if (!m_stcConfig.enable_write_multi)
        {
            ModbusRTU_SendException(addr, func, MODBUS_EXCEPTION_ILLEGAL_FUNC);
            break;
        }
        if (dataLen < 5U)
        {
            ModbusRTU_SendException(addr, func, MODBUS_EXCEPTION_ILLEGAL_DATA_VAL);
            break;
        }
        uint16_t startReg  = MODBUS_BE16(m_stcTxFrame.write_multi.start_reg);
        uint16_t regCount  = MODBUS_BE16(m_stcTxFrame.write_multi.reg_count);
        uint8_t  byteCount = m_stcTxFrame.write_multi.byte_count;

        if ((uint16_t)byteCount != (regCount * 2U))
        {
            MODBUS_PARSE_DBG("  byteCount mismatch: %d != %d", byteCount, regCount * 2U);
            ModbusRTU_SendException(addr, func, MODBUS_EXCEPTION_ILLEGAL_DATA_VAL);
            break;
        }

        if ((uint16_t)dataLen < (5U + (uint16_t)byteCount))
        {
            MODBUS_PARSE_DBG("  frame too short: dataLen=%d < 5+%d", (int)dataLen, (int)byteCount);
            ModbusRTU_SendException(addr, func, MODBUS_EXCEPTION_ILLEGAL_DATA_VAL);
            break;
        }

        ModbusRTU_HandleWriteMulti(addr, startReg, regCount, m_stcTxFrame.write_multi.data);
        break;
    }

    default:
        MODBUS_PARSE_DBG("  unsupported func: 0x%02X", func);
        ModbusRTU_SendException(addr, func, MODBUS_EXCEPTION_ILLEGAL_FUNC);
        break;
    }
}

/*=============================================================================
 * Initialize protocol engine
 *============================================================================*/
void ModbusRTU_Init(const ModbusRTU_Config_t *cfg, const ModbusRTU_Callbacks_t *cbs)
{
    if (cfg != NULL)
    {
        m_stcConfig = *cfg;
    }
    if (cbs != NULL)
    {
        m_stcCallbacks = *cbs;
    }

    MODBUS_PARSE_DBG("Init done, node_id=%d, write_multi=%d",
                     m_stcConfig.node_id, m_stcConfig.enable_write_multi);
}
