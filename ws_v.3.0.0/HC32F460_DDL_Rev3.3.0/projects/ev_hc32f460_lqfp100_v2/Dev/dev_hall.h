#ifndef DEV_HALL_H_
#define DEV_HALL_H_

#include "device_manager.h"
#include "Gpio_io.h"
#include "EventBus.h"
#include "dev_motor.h"
#include "rtt_manager.h"

// ========== 调试宏定义 ==========
// 开关在 rtt_manager.h 中统一管理：DEV_HALL

#ifdef DEV_HALL
    #define HALL_DEBUG(fmt, ...)    MAIN_D("[HALL_DEBUG] " fmt, ##__VA_ARGS__)
    #define HALL_OUT(fmt, ...)      MAIN_D("[HALL_OUT] " fmt, ##__VA_ARGS__)
#else
    #define HALL_DEBUG(fmt, ...)    ((void)0)
    #define HALL_OUT(fmt, ...)      ((void)0)
#endif

// ========== 霍尔设备命令码 ==========
#define CMD_HALL_GET_STATE      (CMD_BASE_HALL + 0x01)
#define CMD_HALL_SET_DEBOUNCE   (CMD_BASE_HALL + 0x02)
#define CMD_HALL_SET_ACTIVE_LEVEL (CMD_BASE_HALL + 0x03)

// ========== 霍尔设备配置 ==========
typedef struct {
    uint8_t port;               // GPIO端口
    uint16_t pin;               // GPIO引脚
    uint8_t active_level;       // 有效电平（0=低有效触发，1=高有效触发）
    uint8_t bind_dir;           // 绑定的方向（DIR_FWD=正转限位，DIR_REV=反转限位）
    uint8_t is_soft_limit;      // 是否为软限位（0=硬限位，1=软限位）
    uint16_t debounce_ms;       // 防抖时间(ms)
    uint8_t window_size;        // 采样窗口大小
    uint16_t sample_interval;   // 采样间隔(ms)
    uint8_t hall_id;            // 霍尔ID
} Hall_Config_t;

// ========== 霍尔设备结构体 ==========
typedef struct {
    uint8_t port;               // GPIO端口
    uint16_t pin;               // GPIO引脚
    uint8_t active_level;       // 有效电平
    uint8_t bind_dir;           // 绑定的方向
    uint8_t is_soft_limit;      // 是否软限位
    uint16_t debounce_ms;       // 防抖时间
    uint8_t window_size;        // 采样窗口
    uint16_t sample_interval;   // 采样间隔
    uint8_t hall_id;            // 霍尔ID
    
    uint8_t initialized;        // 初始化标志
    uint8_t current_state;      // 当前状态
    uint8_t last_stable_state;  // 上次稳定状态
    uint32_t last_change_time;  // 上次状态变化时间
    uint32_t last_sample_time;  // 上次采样时间
    uint8_t sample_buffer[8];   // 采样缓冲区
    uint8_t sample_index;       // 采样索引
    uint8_t sample_count;       // 有效采样数
} Hall_Device_t;

// ========== 标准设备操作 ==========
DeviceResult_t Hall_Device_Init(void* handle);
DeviceResult_t Hall_Device_Deinit(void* handle);
DeviceResult_t Hall_Device_Read(void* handle, void* data, uint32_t size);
DeviceResult_t Hall_Device_Write(void* handle, const void* data, uint32_t size);
DeviceResult_t Hall_Device_Control(void* handle, DeviceCommandData_t* cmd);
DeviceResult_t Hall_Device_Update(void* handle);

// ========== 霍尔特定接口 ==========
uint8_t Hall_Device_GetState(Hall_Device_t* dev);
uint8_t Hall_Device_IsTriggered(Hall_Device_t* dev);
Hall_Device_t* Hall_Device_Create(const Hall_Config_t* config);

// ========== 全局操作函数表 ==========
extern const DeviceOps_t g_hall_ops;

#endif /* DEV_HALL_H_ */
