#ifndef DEV_ADC_H_
#define DEV_ADC_H_

#include "device_manager.h"
#include "EventBus.h"
#include "Adc.h"
#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

// ========== 调试宏定义 ==========
// 开关在 rtt_manager.h 中统一管理：DEV_ADC

#ifdef DEV_ADC
    #define ADC_DEV_DEBUG(fmt, ...)    MAIN_D("[ADC_DEV_DEBUG] " fmt, ##__VA_ARGS__)
    #define ADC_DEBUG(fmt, ...)        MAIN_D("[ADC_DEBUG] " fmt, ##__VA_ARGS__)
    #define ADC_OUT(fmt, ...)          MAIN_D("[ADC_OUT] " fmt, ##__VA_ARGS__)
#else
    #define ADC_DEV_DEBUG(fmt, ...)    ((void)0)
    #define ADC_DEBUG(fmt, ...)        ((void)0)
    #define ADC_OUT(fmt, ...)          ((void)0)
#endif

// ========== ADC设备命令码 ==========
#define CMD_ADC_GET_RAW_VALUE       (CMD_BASE_ADC + 0x01)   // 获取原始ADC值
#define CMD_ADC_GET_VOLTAGE_MV      (CMD_BASE_ADC + 0x02)   // 获取电压值(mV)
#define CMD_ADC_GET_AVERAGE_VALUE   (CMD_BASE_ADC + 0x03)   // 获取平均值

// ========== ADC采集模式 ==========
typedef enum {
    ADC_ACQ_MODE_INTERRUPT = 0,     // 中断模式：ADC转换完成触发中断，在中断中读取数据
    ADC_ACQ_MODE_AOS_DMA   = 1,     // AOS+DMA模式：ADC通过AOS事件路由触发DMA搬运数据
} en_adc_acq_mode_t;

// ========== ADC设备配置 ==========
typedef struct {
    uint8_t             u8AdcId;            // ADC实例ID (由Adc_Create返回)
    uint8_t             u8Channel;          // ADC通道号
    uint8_t             u8Port;             // GPIO端口
    uint16_t            u16Pin;             // GPIO引脚
    en_adc_acq_mode_t   enAcqMode;          // 采集模式：中断模式 / AOS+DMA模式
    uint16_t            u16DmaBufferSize;   // DMA缓冲区大小（仅AOS+DMA模式有效）
    uint8_t             u8DmaChannel;       // DMA通道号（仅AOS+DMA模式有效）
} ADC_Config_t;

// ========== ADC设备结构体 ==========
typedef struct {
    uint8_t             u8AdcId;            // ADC实例ID
    ADC_Config_t        stcConfig;          // 设备配置
    uint8_t             u8Initialized;      // 初始化标志
    
    // 缓存数据
    uint16_t            u16RawValue;        // 最新原始ADC值
    uint16_t            u16VoltageMv;       // 最新电压值(mV)
    uint16_t            u16AverageValue;    // 平均值（DMA模式下为缓冲区均值，中断模式下为最新值）
    
    // DMA缓冲区（仅AOS+DMA模式使用）
    uint16_t*           pu16DmaBuffer;      // DMA缓冲区指针
    uint16_t            u16DmaBufferSize;   // DMA缓冲区大小
    
    // 时间管理
    uint32_t            u32LastUpdateTime;
} ADC_Device_t;

// ========== 读取响应结构体 ==========
typedef struct {
    uint16_t u16RawValue;       // 原始值
    uint16_t u16VoltageMv;      // 电压(mV)
    uint16_t u16AverageValue;   // 平均值
} ADC_ReadResponse_t;

// ========== 标准设备操作（Device Manager接口） ==========
DeviceResult_t ADC_Device_Init(void* handle);
DeviceResult_t ADC_Device_Deinit(void* handle);
DeviceResult_t ADC_Device_Read(void* handle, void* data, uint32_t size);
DeviceResult_t ADC_Device_Write(void* handle, const void* data, uint32_t size);
DeviceResult_t ADC_Device_Control(void* handle, DeviceCommandData_t* cmd);
DeviceResult_t ADC_Device_Update(void* handle);

// ========== ADC特定接口 ==========
uint16_t ADC_Device_GetRawValue(ADC_Device_t* pstcDev);
uint16_t ADC_Device_GetVoltageMV(ADC_Device_t* pstcDev);
uint16_t ADC_Device_GetAverageValue(ADC_Device_t* pstcDev);
ADC_Device_t* ADC_Device_Create(const ADC_Config_t* pstcConfig);

// ========== 全局操作函数表 ==========
extern const DeviceOps_t g_adc_ops;

#endif /* DEV_ADC_H_ */
