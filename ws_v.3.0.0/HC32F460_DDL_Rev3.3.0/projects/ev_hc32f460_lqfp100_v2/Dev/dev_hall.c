#include "dev_hall.h"
#include "TickTimer.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// ========== 内部辅助函数 ==========

// 统一 GPIO 读取函数：将16位端口值转换为单个引脚状态
static uint8_t Hall_ReadPinState(Hall_Device_t* dev) {
    // GPIO_READ宏调用GPIO_ReadInputPins，它返回en_pin_state_t枚举：
    // PIN_RESET (0) 或 PIN_SET (1)
    // 我们需要将其转换为0或1
    en_pin_state_t pin_state = GPIO_READ(dev->port, dev->pin);
    uint8_t state = (pin_state == PIN_SET) ? 1 : 0;
    
    // 为了调试，我们也读取整个端口的值
    uint16_t port_value = GPIO_ReadInputPort(dev->port);
    
    // 调试输出：显示GPIO读取结果（使用 DEBUG_HALL）
    HALL_DEBUG("GPIO Read: ID=%d, Port=%d, Pin=0x%04X, PinState=%d, PortValue=0x%04X, CalculatedState=%d\r\n", 
               dev->hall_id, dev->port, dev->pin, pin_state, port_value, state);
    
    // 检查引脚状态是否与端口值一致
    if (state != ((port_value & dev->pin) != 0)) {
        HALL_DEBUG("WARNING: Pin state mismatch! Direct=%d, FromPort=%d\r\n", 
                   state, ((port_value & dev->pin) != 0));
    }
    
    return state;
}

static uint8_t Hall_SoftwareFilter(Hall_Device_t* dev, uint8_t new_sample) {
    uint8_t window_size = dev->window_size;
    if (window_size > sizeof(dev->sample_buffer)) {
        window_size = sizeof(dev->sample_buffer);
    }
    if (window_size < 1) {
        return new_sample;
    }
    
    dev->sample_buffer[dev->sample_index] = new_sample;
    dev->sample_index = (dev->sample_index + 1) % window_size;
    if (dev->sample_count < window_size) {
        dev->sample_count++;
    }
    
    uint8_t high_count = 0;
    for (uint8_t i = 0; i < dev->sample_count; i++) {
        if (dev->sample_buffer[i]) {
            high_count++;
        }
    }
    
    uint8_t filtered = (high_count > (dev->sample_count / 2)) ? 1 : 0;
    HALL_DEBUG("Filter: ID=%d, NewSample=%d, Filtered=%d, HighCount=%d, Total=%d\r\n",
               dev->hall_id, new_sample, filtered, high_count, dev->sample_count);
    
    return filtered;
}

static Topic_t Hall_GetTopic(Hall_Device_t* dev) {
    return dev->is_soft_limit ? TOPIC_LIMIT_SOFT : TOPIC_LIMIT_HARD;
}

static void Hall_PublishEvent(Hall_Device_t* dev, uint8_t new_state, uint8_t old_state) {
    uint8_t active_state = (new_state == dev->active_level) ? 1 : 0;
    uint8_t old_active = (old_state == dev->active_level) ? 1 : 0;
    
    if (active_state == old_active) return;
    
    if (active_state) {
        HALL_OUT("Triggered: ID=%d, Dir=%s\r\n", 
                 dev->hall_id, 
                 (dev->bind_dir == DIR_FWD) ? "Forward" : "Reverse");
    } else {
        HALL_OUT("Released: ID=%d, Dir=%s\r\n", 
                 dev->hall_id, 
                 (dev->bind_dir == DIR_FWD) ? "Forward" : "Reverse");
    }
    
    MotorLimitEvent_t limitEvent = {
        .dir = (MotorDir_t)dev->bind_dir,
        .is_active = (active_state == 1)
    };
    
    EventBus_Publish(Hall_GetTopic(dev), &limitEvent);
}

// ========== 标准设备操作实现 ==========

DeviceResult_t Hall_Device_Init(void* handle) {
    Hall_Device_t* dev = (Hall_Device_t*)handle;
    if (!dev) return RESULT_PARAM_ERR;
    
    HALL_DEBUG("Init start: ID=%d, Port=%d, Pin=%d, ActiveLevel=%d, BindDir=%d\r\n", 
               dev->hall_id, dev->port, dev->pin, dev->active_level, dev->bind_dir);
    
    Input_GPIO_Init(dev->port, dev->pin, ENABLE);
    
    dev->initialized = 1;
    dev->current_state = 0;
    dev->last_stable_state = 0;
    dev->last_change_time = tickTimer_GetCount();
    dev->last_sample_time = tickTimer_GetCount();
    dev->sample_index = 0;
    dev->sample_count = 0;
    memset(dev->sample_buffer, 0, sizeof(dev->sample_buffer));
    
    // 使用统一的读取函数
    uint8_t raw_state = Hall_ReadPinState(dev);
    uint8_t init_state = (raw_state == dev->active_level) ? 1 : 0;
    dev->current_state = init_state;
    dev->last_stable_state = init_state;
    
    HALL_DEBUG("Init complete: ID=%d, raw=%d, active_level=%d, init_state=%d\r\n", 
               dev->hall_id, raw_state, dev->active_level, init_state);
    
    Hall_PublishEvent(dev, init_state, !init_state);
    
    return RESULT_OK;
}

DeviceResult_t Hall_Device_Deinit(void* handle) {
    Hall_Device_t* dev = (Hall_Device_t*)handle;
    if (!dev) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_OK;
    
    HALL_DEBUG("Deinit: ID=%d\r\n", dev->hall_id);
    
    dev->initialized = 0;
    return RESULT_OK;
}

DeviceResult_t Hall_Device_Read(void* handle, void* data, uint32_t size) {
    Hall_Device_t* dev = (Hall_Device_t*)handle;
    if (!dev || !data) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;
    
    if (size == sizeof(uint8_t)) {
        *(uint8_t*)data = dev->current_state;
        HALL_DEBUG("Read: ID=%d, State=%d\r\n", dev->hall_id, dev->current_state);
    } else {
        return RESULT_PARAM_ERR;
    }
    return RESULT_OK;
}

DeviceResult_t Hall_Device_Write(void* handle, const void* data, uint32_t size) {
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t Hall_Device_Control(void* handle, DeviceCommandData_t* cmd) {
    Hall_Device_t* dev = (Hall_Device_t*)handle;
    if (!dev || !cmd) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;
    
    switch(cmd->cmd) {
        case CMD_HALL_GET_STATE:
            HALL_DEBUG("CMD: GET_STATE -> %d\r\n", dev->current_state);
            if (cmd->response && cmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)cmd->response = dev->current_state;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_HALL_SET_DEBOUNCE:
            if (cmd->params && cmd->param_size == sizeof(uint16_t)) {
                dev->debounce_ms = *(uint16_t*)cmd->params;
                HALL_DEBUG("CMD: SET_DEBOUNCE -> %d ms (ID=%d)\r\n", 
                           dev->debounce_ms, dev->hall_id);
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_HALL_SET_ACTIVE_LEVEL:
            if (cmd->params && cmd->param_size == sizeof(uint8_t)) {
                dev->active_level = *(uint8_t*)cmd->params;
                HALL_DEBUG("CMD: SET_ACTIVE_LEVEL -> %d (ID=%d)\r\n", 
                           dev->active_level, dev->hall_id);
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        default:
            HALL_DEBUG("Unknown cmd=0x%08X (ID=%d)\r\n", cmd->cmd, dev->hall_id);
            return RESULT_ERROR;
    }
}

DeviceResult_t Hall_Device_Update(void* handle) {
    Hall_Device_t* dev = (Hall_Device_t*)handle;
    if (!dev || !dev->initialized) return RESULT_ERROR;
    
    uint32_t now = tickTimer_GetCount();
    
    // 调试输出：显示更新时间信息（使用 DEBUG_HALL）
    HALL_DEBUG("Update: ID=%d, Now=%u, LastSample=%u, Interval=%d, Diff=%u\r\n",
               dev->hall_id, (unsigned int)now, (unsigned int)dev->last_sample_time, 
               dev->sample_interval, (unsigned int)(now - dev->last_sample_time));
    
    if (now - dev->last_sample_time < dev->sample_interval) {
        HALL_DEBUG("Skip: ID=%d, Sample interval not reached\r\n", dev->hall_id);
        return RESULT_OK;
    }
    dev->last_sample_time = now;
    
    // 使用统一的读取函数
    uint8_t raw_state = Hall_ReadPinState(dev);
    uint8_t active_state = (raw_state == dev->active_level) ? 1 : 0;
    
    HALL_DEBUG("State calc: ID=%d, Raw=%d, ActiveLevel=%d, ActiveState=%d\r\n",
               dev->hall_id, raw_state, dev->active_level, active_state);
    
    uint8_t filtered_state = Hall_SoftwareFilter(dev, active_state);
    
    HALL_DEBUG("State check: ID=%d, Filtered=%d, LastStable=%d, LastChange=%u, Debounce=%d, Diff=%u\r\n",
               dev->hall_id, filtered_state, dev->last_stable_state, 
               (unsigned int)dev->last_change_time, dev->debounce_ms, 
               (unsigned int)(now - dev->last_change_time));
    
    if (filtered_state != dev->last_stable_state) {
        if (now - dev->last_change_time >= dev->debounce_ms) {
            uint8_t old_state = dev->current_state;
            dev->last_stable_state = filtered_state;
            dev->current_state = filtered_state;
            dev->last_change_time = now;
            HALL_DEBUG("State changed: ID=%d, Old=%d, New=%d\r\n", 
                       dev->hall_id, old_state, filtered_state);
            Hall_PublishEvent(dev, filtered_state, old_state);
        } else {
            HALL_DEBUG("Debounce waiting: ID=%d, Need %u more ms\r\n", 
                       dev->hall_id, (unsigned int)(dev->debounce_ms - (now - dev->last_change_time)));
        }
    } else {
        HALL_DEBUG("State stable: ID=%d, No change\r\n", dev->hall_id);
    }
    
    return RESULT_OK;
}

// ========== 霍尔特定接口 ==========

uint8_t Hall_Device_GetState(Hall_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    return dev->current_state;
}

uint8_t Hall_Device_IsTriggered(Hall_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    return (dev->current_state == dev->active_level) ? 1 : 0;
}

Hall_Device_t* Hall_Device_Create(const Hall_Config_t* config) {
    if (!config) return NULL;
    
    HALL_DEBUG("Create: ID=%d, Port=%d, Pin=%d, ActiveLevel=%d, BindDir=%d\r\n", 
               config->hall_id, config->port, config->pin, 
               config->active_level, config->bind_dir);
    
    Hall_Device_t* dev = (Hall_Device_t*)malloc(sizeof(Hall_Device_t));
    if (!dev) {
        HALL_DEBUG("Create failed: Out of memory (ID=%d)\r\n", config->hall_id);
        return NULL;
    }
    
    memset(dev, 0, sizeof(Hall_Device_t));
    dev->port = config->port;
    dev->pin = config->pin;
    dev->active_level = config->active_level;
    dev->bind_dir = config->bind_dir;
    dev->is_soft_limit = config->is_soft_limit;
    dev->debounce_ms = config->debounce_ms;
    dev->window_size = config->window_size;
    dev->sample_interval = config->sample_interval;
    dev->hall_id = config->hall_id;
    
    HALL_DEBUG("Create success: ID=%d, Dir=%s, %s limit\r\n", 
               dev->hall_id,
               (dev->bind_dir == DIR_FWD) ? "Forward" : "Reverse",
               dev->is_soft_limit ? "Soft" : "Hard");
    
    return dev;
}

const DeviceOps_t g_hall_ops = {
    .init = Hall_Device_Init,
    .deinit = Hall_Device_Deinit,
    .read = Hall_Device_Read,
    .write = Hall_Device_Write,
    .control = Hall_Device_Control,
    .update = Hall_Device_Update
};
