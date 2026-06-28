#include "Params.h"
#include "param_manager.h"
#include "TickTimer.h"
#include <string.h>
#include "rtt_log.h"

/*=============================================================================
 * 全局变量定义
 *============================================================================*/
AppParamRecord_t g_AppParam;            /* 应用程序参数（Flash 存储） */



// 定义非阻塞延时器（用于5秒定时）
static NonBlockingDelay_t s_paramPrintDelay = {0};

/**
 * @brief  内部函数：实际打印所有参数值
 */
static void Param_PrintAllValues_Internal(void)
{
    PARAM_PRINT("========== Parameter Dump ==========");
    PARAM_PRINT("[Config Parameters - Flash Stored]");
    PARAM_PRINT("  REG_NODE_ID           (0x2710): %u", g_AppParam.node_id);
    PARAM_PRINT("  REG_TARGET_SPEED      (0x2711): %d", g_AppParam.target_speed);
    PARAM_PRINT("  REG_TARGET_ANGLE      (0x2712): %d (%.1f deg)", 
           g_AppParam.target_angle, (float)g_AppParam.target_angle / 10.0f);
    PARAM_PRINT("  REG_VOLTAGE_UPPER_LIMIT(0x2714): %u (%.1f V)", 
           g_AppParam.voltage_upper_limit, (float)g_AppParam.voltage_upper_limit / 10.0f);
    PARAM_PRINT("  REG_VOLTAGE_LOWER_LIMIT(0x2715): %u (%.1f V)", 
           g_AppParam.voltage_lower_limit, (float)g_AppParam.voltage_lower_limit / 10.0f);
    PARAM_PRINT("  REG_CURRENT_UPPER_LIMIT(0x2716): %u (%.1f A)", 
           g_AppParam.current_upper_limit, (float)g_AppParam.current_upper_limit / 1000.0f);
    PARAM_PRINT("  REG_CLOSE_LIMIT_ANGLE (0x271C): %d (%.1f deg)", 
           g_AppParam.close_limit_angle, (float)g_AppParam.close_limit_angle / 10.0f);
    PARAM_PRINT("  REG_OPEN_LIMIT_ANGLE  (0x271D): %d (%.1f deg)", 
           g_AppParam.open_limit_angle, (float)g_AppParam.open_limit_angle / 10.0f);
    PARAM_PRINT("  REG_CURRENT_DETECT_MS (0x271E): %u ms", g_AppParam.current_detect_ms);
    
    PARAM_PRINT("[Internal Parameters - Not Exposed to Modbus]");
    PARAM_PRINT("  baud_rate:                 %lu", g_AppParam.baud_rate);
    PARAM_PRINT("  voltage_upper_hysteresis:  %u (%.1f V)", 
           g_AppParam.voltage_upper_hysteresis, (float)g_AppParam.voltage_upper_hysteresis / 10.0f);
    PARAM_PRINT("  voltage_lower_hysteresis:  %u (%.1f V)", 
           g_AppParam.voltage_lower_hysteresis, (float)g_AppParam.voltage_lower_hysteresis / 10.0f);
    PARAM_PRINT("  overvoltage_trigger_count: %u", g_AppParam.overvoltage_trigger_count);
    PARAM_PRINT("  undervoltage_trigger_count:%u", g_AppParam.undervoltage_trigger_count);
    PARAM_PRINT("  current_hysteresis_ma:     %u mA", g_AppParam.current_hysteresis_ma);
    PARAM_PRINT("  current_release_ms:        %u ms", g_AppParam.current_release_ms);
    PARAM_PRINT("  overcurrent_trigger_count: %u", g_AppParam.overcurrent_trigger_count);
    
    PARAM_PRINT("[Header/Footer Info]");
    PARAM_PRINT("  head_magic:  0x%08X", g_AppParam.head_magic);
    PARAM_PRINT("  tail_magic:  0x%08X", g_AppParam.tail_magic);
    PARAM_PRINT("  sequence_id: %u", g_AppParam.sequence_id);
    PARAM_PRINT("  erase_count: %u", g_AppParam.erase_count);
    PARAM_PRINT("  checksum:    0x%08X", g_AppParam.checksum);
    PARAM_PRINT("===================================");
}

/**
 * @brief  打印所有参数值（每隔5秒输出一次）
 * @note   使用非阻塞延时，需要在主循环中周期性调用
 */
void Param_PrintAllValues(void)
{
#ifdef PARAM_PRINT_DBG
    // 检查是否应该打印
    if (!s_paramPrintDelay.isRunning)
    {
        // 首次启动，立即打印
        nbDelay_Start(&s_paramPrintDelay);
        s_paramPrintDelay.delayMs = 5000;  // 5秒间隔
        Param_PrintAllValues_Internal();
    }
    else if (nbDelay_IsComplete(&s_paramPrintDelay))
    {
        // 时间到了，重新启动定时器并打印
        nbDelay_Start(&s_paramPrintDelay);
        s_paramPrintDelay.delayMs = 5000;
        Param_PrintAllValues_Internal();
    }
#else
    (void)s_paramPrintDelay;  // 避免未使用警告
#endif
}

/*=============================================================================
 * 根据寄存器地址读取参数值
 *============================================================================*/
int32_t Param_ReadByReg(uint16_t regAddr, uint16_t *pValue)
{
    if (pValue == NULL)
    {
        return PARAM_ERR;
    }

    switch (regAddr)
    {
    /* --- 配置参数（Flash 存储） --- */
    case REG_NODE_ID:
        *pValue = g_AppParam.node_id;
        break;

    case REG_TARGET_SPEED:
        *pValue = (uint16_t)g_AppParam.target_speed;
        break;

    case REG_TARGET_ANGLE:
        *pValue = (uint16_t)g_AppParam.target_angle;
        break;

    case REG_VOLTAGE_UPPER_LIMIT:
        *pValue = g_AppParam.voltage_upper_limit;
        break;

    case REG_VOLTAGE_LOWER_LIMIT:
        *pValue = g_AppParam.voltage_lower_limit;
        break;

    case REG_CURRENT_UPPER_LIMIT:
        *pValue = g_AppParam.current_upper_limit;
        break;

    case REG_CLOSE_LIMIT_ANGLE:
        *pValue = (uint16_t)g_AppParam.close_limit_angle;
        break;

    case REG_OPEN_LIMIT_ANGLE:
        *pValue = (uint16_t)g_AppParam.open_limit_angle;
        break;

    case REG_CURRENT_DETECT_MS:
        *pValue = g_AppParam.current_detect_ms;
        break;

    /* --- 控制命令（读回电机仲裁器当前方向） --- */
    case REG_CTRL_CMD:
        /* 读操作由 App_Modbus 处理，这里返回 0 */
        *pValue = 0;
        break;

    /* --- 故障状态 --- */
    case REG_FAULT_STATUS:
        /* 故障状态由 App_FaultHandler 管理，读操作在 App_Modbus 中会调用 FaultHandler_GetFaultStatus */
        *pValue = 0;
        break;

    default:
        /* 未定义的保留寄存器，返回 0 */
        if ((regAddr >= 0x2710U) && (regAddr <= 0x271EU))
        {
            *pValue = 0U;
            return PARAM_OK;
        }
        return PARAM_ERR_INVD_PARAM;
    }

    return PARAM_OK;
}

/*=============================================================================
 * 根据寄存器地址写入参数值（只写入内存，不执行业务逻辑）
 *============================================================================*/
/*=============================================================================
 * 根据寄存器地址写入参数值（只写入内存，不执行业务逻辑）
 *============================================================================*/
int32_t Param_WriteByReg(uint16_t regAddr, uint16_t value)
{
    switch (regAddr)
    {
    case REG_NODE_ID:
        g_AppParam.node_id = value;
        break;

    case REG_TARGET_SPEED:
        g_AppParam.target_speed = (int16_t)value;
        break;

    case REG_TARGET_ANGLE:
        g_AppParam.target_angle = (int16_t)value;
        break;

    case REG_VOLTAGE_UPPER_LIMIT:
        g_AppParam.voltage_upper_limit = value;
        break;

    case REG_VOLTAGE_LOWER_LIMIT:
        g_AppParam.voltage_lower_limit = value;
        break;

    case REG_CURRENT_UPPER_LIMIT:
        g_AppParam.current_upper_limit = value;
        break;

    case REG_CLOSE_LIMIT_ANGLE:
        g_AppParam.close_limit_angle = (int16_t)value;
        break;

    case REG_OPEN_LIMIT_ANGLE:
        g_AppParam.open_limit_angle = (int16_t)value;
        break;

    case REG_CURRENT_DETECT_MS:
        g_AppParam.current_detect_ms = value;
        break;

    /* 控制命令：不存储，直接返回成功（实际业务逻辑在 App_Modbus 中处理） */
    case REG_CTRL_CMD:
        /* 什么都不做，直接返回成功，不存储到 Flash */
        PARAMS_DBG("REG_CTRL_CMD write: 0x%04X (control only, not stored)", value);
        break;

    /* 故障状态：不存储，直接返回成功 */
    case REG_FAULT_STATUS:
        PARAMS_DBG("REG_FAULT_STATUS write: 0x%04X (fault clear, not stored)", value);
        break;

    default:
        return PARAM_ERR_INVD_PARAM;
    }

    return PARAM_OK;
}

/*=============================================================================
 * 检查寄存器地址是否在有效范围内
 *============================================================================*/
bool Param_IsValidRegister(uint16_t regAddr)
{
    /* 客户协议定义的寄存器地址列表 */
    const uint16_t validRegisters[] = {
        REG_NODE_ID,
        REG_TARGET_SPEED,
        REG_TARGET_ANGLE,
        REG_VOLTAGE_UPPER_LIMIT,
        REG_VOLTAGE_LOWER_LIMIT,
        REG_CURRENT_UPPER_LIMIT,
        REG_CLOSE_LIMIT_ANGLE,
        REG_OPEN_LIMIT_ANGLE,
        REG_CURRENT_DETECT_MS,
        REG_CTRL_CMD,
        REG_REAL_SPEED,
        REG_REAL_ANGLE,
        REG_REAL_VOLTAGE,
        REG_REAL_CURRENT,
        REG_REAL_DIRECTION,
        REG_FAULT_STATUS,
    };
    
    const uint16_t reservedRegisters[] = {
        0x2713U, 0x2717U, 0x2718U, 0x2719U, 0x271AU, 0x271BU, 0x271FU
    };
    
    for (int i = 0; i < sizeof(validRegisters) / sizeof(validRegisters[0]); i++) {
        if (regAddr == validRegisters[i]) {
            return true;
        }
    }
    
    for (int i = 0; i < sizeof(reservedRegisters) / sizeof(reservedRegisters[0]); i++) {
        if (regAddr == reservedRegisters[i]) {
            return true;
        }
    }
    
    return false;
}
