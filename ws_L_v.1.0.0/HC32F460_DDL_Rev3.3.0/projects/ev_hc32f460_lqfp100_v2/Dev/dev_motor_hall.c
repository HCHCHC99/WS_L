#include "dev_motor_hall.h"
#include "TickTimer.h"
#include "timer6_timebase.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>

// 打印间隔控制（2000ms）
static uint32_t s_u32LastPrintTime = 0;
#define PRINT_INTERVAL_MS   2000

// ========== 内部辅助函数 ==========

// 将浮点数RPM转换为整数（RPM * 10，保留一位小数）
static uint32_t rpm_to_int(float rpm) {
    return (uint32_t)(rpm * 10);
}

// 更新缓存数据
static void MotorHall_UpdateCache(MotorHall_Device_t* dev) {
    if (!dev || !dev->handle) return;
    
    // 更新底层数据
    motor_hall_update(dev->handle);
    
    // 读取并缓存所有数据
    dev->cached_rpm = motor_hall_get_rpm(dev->handle);
    dev->cached_rpm_raw = motor_hall_get_rpm_raw(dev->handle);
    dev->cached_direction = motor_hall_get_direction(dev->handle);
    dev->direction_confidence = motor_hall_get_direction_confidence(dev->handle);
    dev->cached_pulse_interval = motor_hall_get_pulse_interval_us(dev->handle);
    dev->hall_a_count = motor_hall_get_hall_a_count(dev->handle);
    dev->hall_b_count = motor_hall_get_hall_b_count(dev->handle);
    dev->total_pulse_count = motor_hall_get_total_pulse_count(dev->handle);
    dev->is_running = motor_hall_is_running(dev->handle);
    dev->is_stalled = motor_hall_is_stalled(dev->handle);
    dev->hall_status = motor_hall_get_status(dev->handle);
    
    // 检查方向变化
    if (motor_hall_is_direction_changed(dev->handle)) {
        dev->direction_changed = 1;
    }
    
    // // 调试打印：当有有效速度时打印（使用 MOTOR_HALL_OUT）
    // if (dev->cached_rpm > 10) {
    //     uint32_t rpm_int = rpm_to_int(dev->cached_rpm);
    //     MOTOR_HALL_OUT("RPM=%lu.%lu, Dir=%d, Pulse=%lu us\r\n", 
    //                    rpm_int / 10, rpm_int % 10, dev->cached_direction, dev->cached_pulse_interval);
    // }
    
}

// 发布速度反馈事件
static void MotorHall_PublishSpeedEvent(MotorHall_Device_t* dev) {
    if (!dev || !dev->handle) return;
    
    // 发布当前转速（整数，单位 RPM）
    int32_t speed_rpm = (int32_t)(dev->cached_rpm);
    EventBus_Publish(TOPIC_MOTOR_SPEED_FEEDBACK, &speed_rpm);
}

// ========== 标准设备操作实现 ==========

DeviceResult_t MotorHall_Device_Init(void* handle) {
    MOTOR_HALL_DEBUG("MotorHall_Device_Init: handle=0x%p\r\n", handle);
    
    // 立即打印 handle 指向的内存的前 32 个字节
    MOTOR_HALL_DEBUG("Raw handle data: ");
    uint8_t* p = (uint8_t*)handle;
    for (int i = 0; i < 32; i++) {
        MOTOR_HALL_DEBUG("%02X ", p[i]);
    }
    MOTOR_HALL_DEBUG("\r\n");
    
    MotorHall_Device_t* dev = (MotorHall_Device_t*)handle;
    if (!dev) return RESULT_PARAM_ERR;
    
    MOTOR_HALL_DEBUG("MotorHall_Init: motor_id=%d, update_interval=%d ms\r\n", 
                     dev->dev_cfg.motor_id, dev->dev_cfg.update_interval_ms);
    
    MOTOR_HALL_DEBUG("MotorHall: Hall_A Port=%d, Pin=%d, Hall_B Port=%d, Pin=%d\r\n",
                     dev->config.hall_a_port, dev->config.hall_a_pin,
                     dev->config.hall_b_port, dev->config.hall_b_pin);
    
    // 创建底层实例
    dev->handle = motor_hall_create(&dev->config);
    if (!dev->handle) {
        MOTOR_HALL_DEBUG("Failed to create handle!\r\n");
        return RESULT_ERROR;
    }
    
    // 启动测量
    motor_hall_start(dev->handle);
    motor_hall_reset_counts(dev->handle);
    
    // 初始化缓存
    dev->initialized = 1;
    dev->cached_rpm = 0;
    dev->cached_rpm_raw = 0;
    dev->cached_direction = MOTOR_DIRECTION_NONE;
    dev->direction_confidence = 0;
    dev->cached_pulse_interval = 0;
    dev->hall_a_count = 0;
    dev->hall_b_count = 0;
    dev->total_pulse_count = 0;
    dev->is_running = 0;
    dev->is_stalled = 0;
    dev->hall_status = HALL_STATUS_NONE;
    dev->direction_changed = 0;
    dev->last_update_time = tickTimer_GetCount();
    
    MOTOR_HALL_DEBUG("Init success!\r\n");
    
    return RESULT_OK;
}

DeviceResult_t MotorHall_Device_Deinit(void* handle) {
    MotorHall_Device_t* dev = (MotorHall_Device_t*)handle;
    if (!dev) return RESULT_PARAM_ERR;
    
    MOTOR_HALL_DEBUG("MotorHall_Deinit: motor_id=%d\r\n", dev->dev_cfg.motor_id);
    
    if (dev->handle) {
        motor_hall_stop(dev->handle);
        motor_hall_destroy(dev->handle);
        dev->handle = NULL;
    }
    
    dev->initialized = 0;
    return RESULT_OK;
}

DeviceResult_t MotorHall_Device_Read(void* handle, void* data, uint32_t size) {
    MotorHall_Device_t* dev = (MotorHall_Device_t*)handle;
    if (!dev || !data) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;
    
    if (size == sizeof(float)) {
        *(float*)data = dev->cached_rpm;
    } else if (size == sizeof(uint32_t)) {
        *(uint32_t*)data = (uint32_t)(dev->cached_rpm);
    } else {
        return RESULT_PARAM_ERR;
    }
    
    return RESULT_OK;
}

DeviceResult_t MotorHall_Device_Write(void* handle, const void* data, uint32_t size) {
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t MotorHall_Device_Control(void* handle, DeviceCommandData_t* cmd) {
    MotorHall_Device_t* dev = (MotorHall_Device_t*)handle;
    if (!dev || !cmd) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;
    
    switch(cmd->cmd) {
        case CMD_MOTOR_HALL_GET_RPM:
            {
                uint32_t rpm_int = rpm_to_int(dev->cached_rpm);
                MOTOR_HALL_OUT("CMD: GET_RPM -> %lu.%lu\r\n", rpm_int / 10, rpm_int % 10);
            }
            if (cmd->response && cmd->response_size >= sizeof(float)) {
                *(float*)cmd->response = dev->cached_rpm;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_MOTOR_HALL_GET_DIRECTION:
            // MOTOR_HALL_OUT("CMD: GET_DIRECTION -> %d (conf=%d%%)\r\n", 
            //                dev->cached_direction, dev->direction_confidence);
            if (cmd->response && cmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)cmd->response = (uint8_t)dev->cached_direction;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_MOTOR_HALL_GET_PULSE_COUNT:
            MOTOR_HALL_OUT("CMD: GET_PULSE_COUNT A=%lu, B=%lu, Tot=%lu\r\n",
                           dev->hall_a_count, dev->hall_b_count, dev->total_pulse_count);
            if (cmd->response && cmd->response_size >= sizeof(uint32_t) * 3) {
                uint32_t* resp = (uint32_t*)cmd->response;
                resp[0] = dev->hall_a_count;
                resp[1] = dev->hall_b_count;
                resp[2] = dev->total_pulse_count;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_MOTOR_HALL_GET_HALL_STATUS:
            if (cmd->response && cmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)cmd->response = (uint8_t)dev->hall_status;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_MOTOR_HALL_RESET_COUNTS:
            MOTOR_HALL_DEBUG("CMD: RESET_COUNTS\r\n");
            if (dev->handle) {
                motor_hall_reset_counts(dev->handle);
                MotorHall_UpdateCache(dev);
            }
            return RESULT_OK;
            
        case CMD_MOTOR_HALL_GET_RUNNING_STATE:
            if (cmd->response && cmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)cmd->response = dev->is_running;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        default:
            MOTOR_HALL_DEBUG("Unknown cmd=0x%08X\r\n", cmd->cmd);
            return RESULT_ERROR;
    }
}

DeviceResult_t MotorHall_Device_Update(void* handle) {
    MotorHall_Device_t* dev = (MotorHall_Device_t*)handle;
    if (!dev || !dev->initialized) return RESULT_ERROR;
    
    uint32_t now = tickTimer_GetCount();
    
    // 高速更新：每次调用都更新缓存（保持1ms）
    if (now - dev->last_update_time < dev->dev_cfg.update_interval_ms) {
        return RESULT_OK;
    }
    dev->last_update_time = now;
    
    // 更新缓存数据（高速）
    MotorHall_UpdateCache(dev);
    
    // 发布速度反馈事件（高速）
    MotorHall_PublishSpeedEvent(dev);
    
    // // 方向变化检测（高速）
    // if (dev->direction_changed) {
    //     MOTOR_HALL_OUT("Direction changed to %d (confidence=%d%%)\r\n", 
    //                    dev->cached_direction, dev->direction_confidence);
    //     dev->direction_changed = 0;
    // }
    
    // ========== 打印输出：2000ms一次 ==========
    if (now - s_u32LastPrintTime >= PRINT_INTERVAL_MS) {
        s_u32LastPrintTime = now;
        
        uint32_t rpm_int = rpm_to_int(dev->cached_rpm);
        MOTOR_HALL_OUT("RPM=%lu.%lu, Dir=%d, Pulse=%lu us, Running=%d, Stalled=%d\r\n",
                       rpm_int / 10, rpm_int % 10,
                       dev->cached_direction, dev->cached_pulse_interval,
                       dev->is_running, dev->is_stalled);
    }
    
    return RESULT_OK;
}

// ========== 电机霍尔特定接口 ==========

float MotorHall_Device_GetRPM(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    return dev->cached_rpm;
}

float MotorHall_Device_GetRPMRaw(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    return dev->cached_rpm_raw;
}

motor_direction_t MotorHall_Device_GetDirection(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return MOTOR_DIRECTION_NONE;
    return dev->cached_direction;
}

uint8_t MotorHall_Device_GetDirectionConfidence(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    return dev->direction_confidence;
}

uint8_t MotorHall_Device_IsDirectionChanged(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    uint8_t changed = dev->direction_changed;
    dev->direction_changed = 0;
    return changed;
}

uint32_t MotorHall_Device_GetHallACount(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    return dev->hall_a_count;
}

uint32_t MotorHall_Device_GetHallBCount(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    return dev->hall_b_count;
}

uint32_t MotorHall_Device_GetTotalPulseCount(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    return dev->total_pulse_count;
}

void MotorHall_Device_ResetCounts(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return;
    if (dev->handle) {
        MOTOR_HALL_DEBUG("Reset counts\r\n");
        motor_hall_reset_counts(dev->handle);
        MotorHall_UpdateCache(dev);
    }
}

uint8_t MotorHall_Device_IsRunning(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    return dev->is_running;
}

uint8_t MotorHall_Device_IsStalled(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return 1;
    return dev->is_stalled;
}

hall_working_status_t MotorHall_Device_GetHallStatus(MotorHall_Device_t* dev) {
    if (!dev || !dev->initialized) return HALL_STATUS_NONE;
    return dev->hall_status;
}

MotorHall_Device_t* MotorHall_Device_Create(const motor_hall_config_t* config, const MotorHall_DeviceConfig_t* dev_cfg) {
    if (!config) return NULL;
    
    MOTOR_HALL_DEBUG("=== MotorHall_Create INPUT ===\r\n");
    MOTOR_HALL_DEBUG("config: hall_a_port=%d, hall_a_pin=0x%04X\r\n", config->hall_a_port, config->hall_a_pin);
    MOTOR_HALL_DEBUG("config: hall_b_port=%d, hall_b_pin=0x%04X\r\n", config->hall_b_port, config->hall_b_pin);
    MOTOR_HALL_DEBUG("config: eirq_ch_a=%lu, eirq_ch_b=%lu\r\n", config->eirq_ch_a, config->eirq_ch_b);
    MOTOR_HALL_DEBUG("config: irqn_a=%d, irqn_b=%d\r\n", config->irqn_a, config->irqn_b);
    MOTOR_HALL_DEBUG("config: irq_src_a=%lu, irq_src_b=%lu\r\n", config->irq_src_a, config->irq_src_b);
    MOTOR_HALL_DEBUG("config: irq_priority=%d\r\n", config->irq_priority);
    MOTOR_HALL_DEBUG("config: pole_pairs=%d, hall_count=%d, custom_pulses_per_rev=%d\r\n", 
                     config->pole_pairs, config->hall_count, config->custom_pulses_per_rev);
    
    if (dev_cfg) {
        MOTOR_HALL_DEBUG("dev_cfg: motor_id=%d, update_interval=%d ms\r\n", 
                         dev_cfg->motor_id, dev_cfg->update_interval_ms);
    }
    
    MotorHall_Device_t* dev = (MotorHall_Device_t*)malloc(sizeof(MotorHall_Device_t));
    if (!dev) return NULL;
    
    memset(dev, 0, sizeof(MotorHall_Device_t));
    
    // ========== 逐个成员赋值，不使用 memcpy ==========
    // 复制底层配置
    dev->config.hall_a_port = config->hall_a_port;
    dev->config.hall_a_pin = config->hall_a_pin;
    dev->config.hall_b_port = config->hall_b_port;
    dev->config.hall_b_pin = config->hall_b_pin;
    
    dev->config.eirq_ch_a = config->eirq_ch_a;
    dev->config.eirq_ch_b = config->eirq_ch_b;
    dev->config.irqn_a = config->irqn_a;
    dev->config.irqn_b = config->irqn_b;
    dev->config.irq_src_a = config->irq_src_a;
    dev->config.irq_src_b = config->irq_src_b;
    dev->config.irq_priority = config->irq_priority;
    
    dev->config.pole_pairs = config->pole_pairs;
    dev->config.hall_count = config->hall_count;
    dev->config.custom_pulses_per_rev = config->custom_pulses_per_rev;
    
    // 复制设备层特有配置
    if (dev_cfg) {
        dev->dev_cfg.motor_id = dev_cfg->motor_id;
        dev->dev_cfg.update_interval_ms = dev_cfg->update_interval_ms;
    } else {
        dev->dev_cfg.motor_id = 0;
        dev->dev_cfg.update_interval_ms = 10;
    }
    
    dev->initialized = 0;
    
    // 验证复制结果
    MOTOR_HALL_DEBUG("=== MotorHall_Create AFTER COPY ===\r\n");
    MOTOR_HALL_DEBUG("dev->config.hall_a_port=%d, hall_a_pin=0x%04X\r\n", 
                     dev->config.hall_a_port, dev->config.hall_a_pin);
    MOTOR_HALL_DEBUG("dev->config.hall_b_port=%d, hall_b_pin=0x%04X\r\n", 
                     dev->config.hall_b_port, dev->config.hall_b_pin);
    MOTOR_HALL_DEBUG("dev->config.eirq_ch_a=%lu, eirq_ch_b=%lu\r\n", 
                     dev->config.eirq_ch_a, dev->config.eirq_ch_b);
    MOTOR_HALL_DEBUG("dev->config.irqn_a=%d, irqn_b=%d\r\n", 
                     dev->config.irqn_a, dev->config.irqn_b);
    
    return dev;
}

// ========== 全局操作函数表 ==========
const DeviceOps_t g_motor_hall_ops = {
    .init = MotorHall_Device_Init,
    .deinit = MotorHall_Device_Deinit,
    .read = MotorHall_Device_Read,
    .write = MotorHall_Device_Write,
    .control = MotorHall_Device_Control,
    .update = MotorHall_Device_Update
};
