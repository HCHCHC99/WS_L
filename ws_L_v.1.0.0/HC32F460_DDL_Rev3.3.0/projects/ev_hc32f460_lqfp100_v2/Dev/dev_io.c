#include "dev_io.h"
#include "TickTimer.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// 统一 GPIO 读取函数
static uint8_t IO_ReadPinState(IO_Device_t* dev) {
    // GPIO_READ宏调用GPIO_ReadInputPins，它返回en_pin_state_t枚举：
    // PIN_RESET (0) 或 PIN_SET (1)
    // 我们需要将其转换为0或1
    en_pin_state_t pin_state = GPIO_READ(dev->port, dev->pin);
    uint8_t state = (pin_state == PIN_SET) ? 1 : 0;
    
    // 为了调试，我们也读取整个端口的值
    uint16_t port_value = GPIO_ReadInputPort(dev->port);
    
    // 调试输出：显示GPIO读取结果
    MAIN_D("IO GPIO Read: ID=%d, Port=%d, Pin=0x%04X, PinState=%d, PortValue=0x%04X, CalculatedState=%d\r\n", 
           dev->io_id, dev->port, dev->pin, pin_state, port_value, state);
    
    // 检查引脚状态是否与端口值一致
    uint8_t calculated_from_port = ((port_value & dev->pin) != 0) ? 1 : 0;
    if (state != calculated_from_port) {
        MAIN_D("WARNING: Pin state mismatch! Direct=%d, FromPort=%d\r\n", state, calculated_from_port);
    }
    
    return state;
}

// 软件滤波函数
static uint8_t IO_SoftwareFilter(IO_Device_t* dev, uint8_t new_sample) {
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
    
    return (high_count > (dev->sample_count / 2)) ? 1 : 0;
}

// 发布事件
static void IO_PublishEvent(IO_Device_t* dev, uint8_t new_state, uint8_t old_state) {
    uint8_t active_state = (new_state == dev->active_level) ? 1 : 0;
    uint8_t old_active = (old_state == dev->active_level) ? 1 : 0;
    
    if (active_state == old_active) return;
    
    MotorManualIOEvent_t ioEvent;
    
    if (active_state) {
        // 按钮按下
        if (dev->io_id == 0) {
            ioEvent.dir = DIR_FWD;
            ioEvent.type = CMD_TYPE_RUN_FWD;
        } else {
            ioEvent.dir = DIR_REV;
            ioEvent.type = CMD_TYPE_RUN_REV;
        }
        ioEvent.speed = 85.0f;
    } else {
        // 按钮释放
        if (dev->io_id == 0) {
            ioEvent.dir = DIR_FWD;
        } else {
            ioEvent.dir = DIR_REV;
        }
        ioEvent.type = CMD_TYPE_STOP;
        ioEvent.speed = 0.0f;
    }
    
    EventBus_Publish(TOPIC_MANUAL_IO, &ioEvent);
}

// ========== 标准设备操作实现 ==========

DeviceResult_t IO_Device_Init(void* handle) {
    IO_Device_t* dev = (IO_Device_t*)handle;
    if (!dev) return RESULT_PARAM_ERR;
    
    // 配置GPIO为输入模式（使用上拉）
    Input_GPIO_Init(dev->port, dev->pin, ENABLE);
    
    dev->initialized = 1;
    dev->current_state = 0;
    dev->last_stable_state = 0;
    dev->last_change_time = tickTimer_GetCount();
    dev->last_sample_time = tickTimer_GetCount();
    dev->sample_index = 0;
    dev->sample_count = 0;
    memset(dev->sample_buffer, 0, sizeof(dev->sample_buffer));
    
    // 读取初始状态 - 使用统一的读取函数
    uint8_t raw_state = IO_ReadPinState(dev);
    uint8_t init_state = (raw_state == dev->active_level) ? 1 : 0;
    dev->current_state = init_state;
    dev->last_stable_state = init_state;
    
    MAIN_D("IO init: ID=%d, Port=%d, Pin=%d, raw=%d, active_level=%d, init=%d\r\n", 
           dev->io_id, dev->port, dev->pin, raw_state, dev->active_level, init_state);
    
    return RESULT_OK;
}

DeviceResult_t IO_Device_Deinit(void* handle) {
    IO_Device_t* dev = (IO_Device_t*)handle;
    if (!dev) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_OK;
    
    dev->initialized = 0;
    return RESULT_OK;
}

DeviceResult_t IO_Device_Read(void* handle, void* data, uint32_t size) {
    IO_Device_t* dev = (IO_Device_t*)handle;
    if (!dev || !data) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;
    
    if (size == sizeof(uint8_t)) {
        *(uint8_t*)data = dev->current_state;
    } else {
        return RESULT_PARAM_ERR;
    }
    return RESULT_OK;
}

DeviceResult_t IO_Device_Write(void* handle, const void* data, uint32_t size) {
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t IO_Device_Control(void* handle, DeviceCommandData_t* cmd) {
    IO_Device_t* dev = (IO_Device_t*)handle;
    if (!dev || !cmd) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;
    
    switch(cmd->cmd) {
        case CMD_IO_GET_INPUT:
            if (cmd->response && cmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)cmd->response = dev->current_state;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_IO_SET_DEBOUNCE:
            if (cmd->params && cmd->param_size == sizeof(uint16_t)) {
                dev->debounce_ms = *(uint16_t*)cmd->params;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        case CMD_IO_SET_ACTIVE_LEVEL:
            if (cmd->params && cmd->param_size == sizeof(uint8_t)) {
                dev->active_level = *(uint8_t*)cmd->params;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
            
        default:
            return RESULT_ERROR;
    }
}

DeviceResult_t IO_Device_Update(void* handle) {
    IO_Device_t* dev = (IO_Device_t*)handle;
    if (!dev || !dev->initialized) return RESULT_ERROR;
    
    uint32_t now = tickTimer_GetCount();
    
    // 调试输出：显示更新时间信息
    MAIN_D("IO Update Start: ID=%d, Now=%u, LastSample=%u, Interval=%d, Diff=%u\r\n",
           dev->io_id, (unsigned int)now, (unsigned int)dev->last_sample_time, dev->sample_interval, 
           (unsigned int)(now - dev->last_sample_time));
    
    if (now - dev->last_sample_time < dev->sample_interval) {
        MAIN_D("IO Update Skip: ID=%d, Sample interval not reached\r\n", dev->io_id);
        return RESULT_OK;
    }
    dev->last_sample_time = now;
    
    // 使用统一的读取函数
    uint8_t raw_state = IO_ReadPinState(dev);
    uint8_t active_state = (raw_state == dev->active_level) ? 1 : 0;
    
    MAIN_D("IO State Calc: ID=%d, Raw=%d, ActiveLevel=%d, ActiveState=%d\r\n",
           dev->io_id, raw_state, dev->active_level, active_state);
    
    uint8_t filtered_state = IO_SoftwareFilter(dev, active_state);
    
    MAIN_D("IO State Check: ID=%d, Filtered=%d, LastStable=%d, LastChange=%u, Debounce=%d, Diff=%u\r\n",
           dev->io_id, filtered_state, dev->last_stable_state, 
           (unsigned int)dev->last_change_time, dev->debounce_ms, (unsigned int)(now - dev->last_change_time));
    
    if (filtered_state != dev->last_stable_state) {
        if (now - dev->last_change_time >= dev->debounce_ms) {
            uint8_t old_state = dev->current_state;
            dev->last_stable_state = filtered_state;
            dev->current_state = filtered_state;
            dev->last_change_time = now;
            MAIN_D("IO State Changed: ID=%d, Old=%d, New=%d\r\n", 
                   dev->io_id, old_state, filtered_state);
            IO_PublishEvent(dev, filtered_state, old_state);
        } else {
            MAIN_D("IO Debounce Waiting: ID=%d, Need %u more ms\r\n", 
                   dev->io_id, (unsigned int)(dev->debounce_ms - (now - dev->last_change_time)));
        }
    } else {
        MAIN_D("IO State Stable: ID=%d, No change\r\n", dev->io_id);
    }
    
    return RESULT_OK;
}

// ========== IO特定接口 ==========

uint8_t IO_Device_GetInput(IO_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    return dev->current_state;
}

IO_Device_t* IO_Device_Create(const IO_Config_t* config) {
    if (!config) return NULL;
    
    IO_Device_t* dev = (IO_Device_t*)malloc(sizeof(IO_Device_t));
    if (!dev) return NULL;
    
    memset(dev, 0, sizeof(IO_Device_t));
    dev->port = config->port;
    dev->pin = config->pin;
    dev->active_level = config->active_level;
    dev->debounce_ms = config->debounce_ms;
    dev->window_size = config->window_size;
    dev->sample_interval = config->sample_interval;
    dev->io_id = config->io_id;
    
    return dev;
}

const DeviceOps_t g_io_ops = {
    .init = IO_Device_Init,
    .deinit = IO_Device_Deinit,
    .read = IO_Device_Read,
    .write = IO_Device_Write,
    .control = IO_Device_Control,
    .update = IO_Device_Update
};
