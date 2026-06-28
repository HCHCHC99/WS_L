#include "App_Comm.h"
#include "Comm_HAL.h"
#include "Protocol_ModbusRtu.h"
#include "param_manager.h"
#include "Params.h"
#include "TickTimer.h"
#include "App_Realtime.h"
#include "App_FaultHandler.h"
#include "App_Motor_Project.h"
#include <string.h>

/*=============================================================================
 * Auto-send config (echo mode, disabled by default)
 *============================================================================*/
#ifdef APP_COMM_AUTO_SEND_ENABLE
    #define AUTO_SEND_INTERVAL_MS   (3000UL)
    #define AUTO_SEND_MSG           "123\r\n"
    #define AUTO_SEND_MSG_LEN       (5U)
    static NonBlockingDelay_t m_stcAutoSendDelay;
#endif

/*=============================================================================
 * Reset delay
 *============================================================================*/
#define RESET_DELAY_MS              (200U)

/*=============================================================================
 * Static data
 *============================================================================*/
static App_Comm_Config_t m_stcConfig;
static bool m_bCommControlEnabled;
static bool m_bResetPending;
static NonBlockingDelay_t m_stcResetDelay;

static const Param_Config_t m_stcParamConfig = {
    .pParamBuf      = &g_AppParam,
    .paramSize      = sizeof(AppParamRecord_t),
    .magicHead      = PARAM_MAGIC_HEAD,
    .magicTail      = PARAM_MAGIC_TAIL,
    .checksumOffset = offsetof(AppParamRecord_t, checksum),
    .seqOffset      = offsetof(AppParamRecord_t, sequence_id),
    .eraseCntOffset = offsetof(AppParamRecord_t, erase_count),
};

/*=============================================================================
 * Forward declarations
 *============================================================================*/
static void App_Comm_SetParamDefaults(void);
static bool App_Comm_OnReadReg(uint16_t reg_addr, uint16_t *value);
static bool App_Comm_OnWriteReg(uint16_t reg_addr, uint16_t value);
static bool App_Comm_OnValidateReg(uint16_t reg_addr);
static void App_Comm_HandleCtrlCmd(uint16_t cmd);
#ifdef APP_COMM_AUTO_SEND_ENABLE
static void App_Comm_AutoSendProcess(void);
#endif

/*=============================================================================
 * Set default parameters (callback for Param_Init)
 *============================================================================*/
static void App_Comm_SetParamDefaults(void)
{
    memset(&g_AppParam, 0, sizeof(AppParamRecord_t));
    g_AppParam.head_magic = PARAM_MAGIC_HEAD;
    g_AppParam.tail_magic = PARAM_MAGIC_TAIL;

    g_AppParam.node_id               = (uint16_t)PARAM_DEFAULT_NODE_ID;
    g_AppParam.target_speed          = (int16_t)PARAM_DEFAULT_TARGET_SPEED;
    g_AppParam.target_angle          = (int16_t)PARAM_DEFAULT_TARGET_ANGLE;
    g_AppParam.voltage_upper_limit   = (uint16_t)PARAM_DEFAULT_VOLTAGE_UPPER_LIMIT;
    g_AppParam.voltage_lower_limit   = (uint16_t)PARAM_DEFAULT_VOLTAGE_LOWER_LIMIT;
    g_AppParam.current_upper_limit   = (uint16_t)PARAM_DEFAULT_CURRENT_UPPER_LIMIT;
    g_AppParam.current_detect_ms     = (uint16_t)PARAM_DEFAULT_CURRENT_DETECT_MS;
    g_AppParam.close_limit_angle     = (int16_t)PARAM_DEFAULT_CLOSE_LIMIT_ANGLE;
    g_AppParam.open_limit_angle      = (int16_t)PARAM_DEFAULT_OPEN_LIMIT_ANGLE;
    g_AppParam.baud_rate             = PARAM_DEFAULT_BAUD_RATE;

    g_AppParam.voltage_upper_hysteresis   = (uint16_t)PARAM_DEFAULT_VOLTAGE_UPPER_HYSTERESIS;
    g_AppParam.voltage_lower_hysteresis   = (uint16_t)PARAM_DEFAULT_VOLTAGE_LOWER_HYSTERESIS;
    g_AppParam.overvoltage_trigger_count  = (uint8_t)PARAM_DEFAULT_OVERVOLTAGE_TRIGGER_CNT;
    g_AppParam.undervoltage_trigger_count = (uint8_t)PARAM_DEFAULT_UNDERVOLTAGE_TRIGGER_CNT;
    g_AppParam.current_hysteresis_ma      = (uint16_t)PARAM_DEFAULT_CURRENT_HYSTERESIS_MA;
    g_AppParam.current_release_ms         = (uint16_t)PARAM_DEFAULT_CURRENT_RELEASE_MS;
    g_AppParam.overcurrent_trigger_count  = (uint8_t)PARAM_DEFAULT_OVERCURRENT_TRIGGER_CNT;

    g_AppParam.reserved[0] = 0;
    g_AppParam.reserved[1] = 0;
    g_AppParam.reserved[2] = 0;

    g_AppParam.tail_magic = PARAM_MAGIC_TAIL;
}

/*=============================================================================
 * Validate register callback — called by ModbusRTU before read/write
 *============================================================================*/
static bool App_Comm_OnValidateReg(uint16_t reg_addr)
{
    return Param_IsValidRegister(reg_addr);
}

/*=============================================================================
 * Read register callback — called by ModbusRTU for each register in 0x03
 *============================================================================*/
static bool App_Comm_OnReadReg(uint16_t reg_addr, uint16_t *value)
{
    if (value == NULL) return false;

    switch (reg_addr)
    {
    case REG_REAL_SPEED:
        *value = (uint16_t)Realtime_GetSpeed();
        break;
    case REG_REAL_ANGLE:
        *value = (uint16_t)Realtime_GetAngle();
        break;
    case REG_REAL_VOLTAGE:
        *value = Realtime_GetVoltage();
        break;
    case REG_REAL_CURRENT:
        *value = Realtime_GetCurrent();
        break;
    case REG_REAL_DIRECTION:
        *value = (uint16_t)Realtime_GetDirection();
        break;
    case REG_FAULT_STATUS:
        *value = FaultHandler_GetFaultStatus();
        break;
    case REG_CTRL_CMD:
    {
        uint16_t u16Value = 0U;
        int16_t direction = Realtime_GetDirection();
        if (direction == 1)
            u16Value |= CTRL_CMD_FWD;
        else if (direction == 2)
            u16Value |= CTRL_CMD_REV;
        if (m_bCommControlEnabled)
            u16Value |= 0x0040U;
        *value = u16Value;
        break;
    }
    default:
        return (Param_ReadByReg(reg_addr, value) == PARAM_OK);
    }
    return true;
}

/*=============================================================================
 * Write register callback — called by ModbusRTU for each register write
 *============================================================================*/
static bool App_Comm_OnWriteReg(uint16_t reg_addr, uint16_t value)
{
    int32_t ret;

    /* Control command register — handle immediately, don't persist to Flash */
    if (reg_addr == REG_CTRL_CMD)
    {
        App_Comm_HandleCtrlCmd(value);
        return true;
    }

    /* Fault status register — clear faults by mask */
    if (reg_addr == REG_FAULT_STATUS)
    {
        FaultHandler_ClearFaultByMask(value);
        return true;
    }

    /* Normal parameter — write to RAM then persist to Flash */
    ret = Param_WriteByReg(reg_addr, value);
    if (ret != PARAM_OK)
    {
        COMM_DBG("Param_WriteByReg failed: reg=0x%04X, ret=%ld", reg_addr, (long)ret);
        return false;
    }

    /* Handle baudrate change */
    if (reg_addr == REG_NODE_ID)
    {
        /* Node ID changed at runtime — ModbusRTU would need re-init.
         * For simplicity, the new ID takes effect after next reset. */
        COMM_DBG("node_id changed to %d (takes effect after reset)", value);
    }

    ret = Param_Save(&m_stcParamConfig);
    if (ret != PARAM_OK)
    {
        COMM_DBG("Param_Save failed, ret=%ld", (long)ret);
    }
    else
    {
        COMM_DBG("Param saved to Flash");
    }

    return true;
}

/*=============================================================================
 * Batch write callback — called by ModbusRTU for 0x10 multi-register writes.
 * Writes all registers to RAM first, then persists to Flash once.
 *============================================================================*/
static bool App_Comm_OnWriteMulti(uint16_t startReg, uint16_t regCount, const uint8_t *pData)
{
    uint16_t i;
    uint16_t regAddr;
    uint16_t regValue;

    COMM_DBG("write multi: start=0x%04X, count=%d", startReg, (int)regCount);

    /* Special registers must be written individually (0x06), not batched */
    for (i = 0U; i < regCount; i++)
    {
        regAddr = startReg + i;
        if (regAddr == REG_CTRL_CMD || regAddr == REG_FAULT_STATUS)
        {
            COMM_DBG("  reg 0x%04X requires single-write (0x06)", regAddr);
            return false;
        }
    }

    /* Phase 1: write all to RAM */
    for (i = 0U; i < regCount; i++)
    {
        regAddr  = startReg + i;
        regValue = (uint16_t)((pData[i * 2U] << 8U) | pData[i * 2U + 1U]);

        if (Param_WriteByReg(regAddr, regValue) != PARAM_OK)
        {
            COMM_DBG("  Param_WriteByReg failed: reg=0x%04X", regAddr);
            return false;
        }
    }

    /* Phase 2: persist to Flash once for the entire batch */
    if (Param_Save(&m_stcParamConfig) != PARAM_OK)
    {
        COMM_DBG("  Param_Save failed");
        return false;
    }

    COMM_DBG("write multi OK, %d regs saved to Flash", (int)regCount);
    return true;
}

/*=============================================================================
 * Handle control commands from REG_CTRL_CMD
 *============================================================================*/
static void App_Comm_HandleCtrlCmd(uint16_t cmd)
{
    COMM_DBG("CTRL_CMD=0x%04X", cmd);

    /* 1. RESET — highest priority */
    if (cmd & CTRL_CMD_RESET)
    {
        MAIN_D("[APP_COMM] RESET command: start reset delay %dms", (int)RESET_DELAY_MS);
        m_bResetPending = true;
        nbDelay_Start(&m_stcResetDelay);
        return;
    }

    /* 2. STOP — disable control, stop motor */
    if (cmd & CTRL_CMD_STOP)
    {
        MAIN_D("[APP_COMM] STOP: RS485 control DISABLED, motor STOP");
        m_bCommControlEnabled = false;

        MotorManualIOEvent_t ioEvent;
        memset(&ioEvent, 0, sizeof(MotorManualIOEvent_t));
        ioEvent.dir = DIR_NONE;
        ioEvent.type = CMD_TYPE_STOP;
        ioEvent.speed = 0.0f;
        EventBus_Publish(TOPIC_MANUAL_IO, &ioEvent);
        return;
    }

    /* 3. ESTOP — emergency stop, keep control enabled */
    if (cmd & CTRL_CMD_ESTOP)
    {
        MAIN_D("[APP_COMM] ESTOP: emergency stop, control still ENABLED");

        MotorManualIOEvent_t ioEvent;
        memset(&ioEvent, 0, sizeof(MotorManualIOEvent_t));
        ioEvent.dir = DIR_NONE;
        ioEvent.type = CMD_TYPE_STOP;
        ioEvent.speed = 0.0f;
        EventBus_Publish(TOPIC_MANUAL_IO, &ioEvent);

        COMM_DBG("Emergency stop executed, control enabled=%d", m_bCommControlEnabled);
        return;
    }

    /* 4. START — enable control */
    if (cmd & CTRL_CMD_START)
    {
        MAIN_D("[APP_COMM] START: RS485 control ENABLED");
        m_bCommControlEnabled = true;
        return;
    }

    /* 5. FWD/REV — require START first */
    if (!m_bCommControlEnabled)
    {
        MAIN_D("[APP_COMM] ERROR: FWD/REV rejected - control not enabled (send START first)");
        return;
    }

    float target_speed_percent = (float)g_AppParam.target_speed / 100.0f;
    if (target_speed_percent < 0.0f) target_speed_percent = 0.0f;
    if (target_speed_percent > 100.0f) target_speed_percent = 100.0f;

    if (cmd & CTRL_CMD_FWD)
    {
        MAIN_D("[APP_COMM] FWD: speed=%.1f%%", target_speed_percent);

        MotorManualIOEvent_t ioEvent;
        memset(&ioEvent, 0, sizeof(MotorManualIOEvent_t));
        ioEvent.dir = DIR_FWD;
        ioEvent.type = CMD_TYPE_RUN_FWD;
        ioEvent.speed = target_speed_percent;
        EventBus_Publish(TOPIC_MANUAL_IO, &ioEvent);
    }
    else if (cmd & CTRL_CMD_REV)
    {
        MAIN_D("[APP_COMM] REV: speed=%.1f%%", target_speed_percent);

        MotorManualIOEvent_t ioEvent;
        memset(&ioEvent, 0, sizeof(MotorManualIOEvent_t));
        ioEvent.dir = DIR_REV;
        ioEvent.type = CMD_TYPE_RUN_REV;
        ioEvent.speed = target_speed_percent;
        EventBus_Publish(TOPIC_MANUAL_IO, &ioEvent);
    }
}

#ifdef APP_COMM_AUTO_SEND_ENABLE
/*=============================================================================
 * Auto-send process (echo/debug mode)
 *============================================================================*/
static void App_Comm_AutoSendProcess(void)
{
    if (nbDelay_IsComplete(&m_stcAutoSendDelay))
    {
        COMM_DBG("Auto send: %s", AUTO_SEND_MSG);
        Comm_HAL_Send((uint8_t *)AUTO_SEND_MSG, AUTO_SEND_MSG_LEN);
        nbDelay_Start(&m_stcAutoSendDelay);
    }
}
#endif

/*=============================================================================
 * Initialize the complete communication stack
 *============================================================================*/
void App_Comm_Init(const App_Comm_Config_t *cfg)
{
    if (cfg == NULL) return;
    m_stcConfig = *cfg;

    /* 1. Init parameter storage (load from Flash or set defaults) */
    Param_Init(&m_stcParamConfig, App_Comm_SetParamDefaults);

    /* 2. Init realtime data module */
    Realtime_Init();

    /* 3. Override config with Flash-loaded values */
    RS485_HW_Config_t hwCfg;
    hwCfg.baudrate     = g_AppParam.baud_rate;
    hwCfg.dir_polarity = cfg->phy.dir_polarity;

    Comm_HAL_Config_t halCfg;
    halCfg.rx_buf_size          = cfg->hal.rx_buf_size;
    halCfg.tx_buf_size          = cfg->hal.tx_buf_size;
    halCfg.rx_frame_queue_depth = cfg->hal.rx_frame_queue_depth;
    halCfg.tx_queue_depth       = cfg->hal.tx_queue_depth;
    halCfg.frame_timeout_ms     = cfg->hal.frame_timeout_ms;

    Comm_HAL_Init(&hwCfg, &halCfg);

    /* 4. Init Modbus RTU protocol with callbacks */
    ModbusRTU_Config_t protoCfg;
    protoCfg.node_id           = (uint8_t)g_AppParam.node_id;
    protoCfg.enable_write_multi = cfg->proto.enable_write_multi;

    ModbusRTU_Callbacks_t cbs;
    cbs.on_read       = App_Comm_OnReadReg;
    cbs.on_write      = App_Comm_OnWriteReg;
    cbs.on_write_multi = App_Comm_OnWriteMulti;
    cbs.on_validate   = App_Comm_OnValidateReg;

    ModbusRTU_Init(&protoCfg, &cbs);

    /* 5. Init reset delay */
    nbDelay_Init(&m_stcResetDelay, RESET_DELAY_MS);
    m_bResetPending        = false;
    m_bCommControlEnabled  = false;

#ifdef APP_COMM_AUTO_SEND_ENABLE
    nbDelay_Init(&m_stcAutoSendDelay, AUTO_SEND_INTERVAL_MS);
    nbDelay_Start(&m_stcAutoSendDelay);
#endif

    MAIN_D("App_Comm initialized: node_id=%d, baud=%lu",
           g_AppParam.node_id, g_AppParam.baud_rate);
}

/*=============================================================================
 * Main loop poll
 *============================================================================*/
void App_Comm_Poll(void)
{
    uint8_t  frameBuf[256];
    uint16_t frameLen;

    /* 1. HAL poll: frame assembly + TX queue drain */
    Comm_HAL_Poll();

    /* 2. Process received frames */
    if (Comm_HAL_RecvFrame(frameBuf, &frameLen, sizeof(frameBuf)))
    {
        COMM_DBG("Dequeue frame, len=%d", frameLen);
        ModbusRTU_ProcessFrame(frameBuf, frameLen);
    }

#ifdef APP_COMM_AUTO_SEND_ENABLE
    App_Comm_AutoSendProcess();
#endif

    /* 3. Reset delay check */
    if (m_bResetPending && nbDelay_IsComplete(&m_stcResetDelay))
    {
        MAIN_D("[APP_COMM] Reset delay done, system reset now!");
        m_bResetPending = false;
        __NVIC_SystemReset();
    }
}
