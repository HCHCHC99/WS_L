#ifndef __RTT_MANAGER_H__
#define __RTT_MANAGER_H__

#include "rtt_log.h"
#include "TickTimer.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

/*=============================================================================
 * RTT 调试宏统一管理
 *
 * 使用方式：
 *   1. 在 rtt_manager.h 中取消对应模块的 #define 注释即可开启调试
 *   2. 各模块的 .h 文件保留自己的 #ifdef XXX_DEBUG ... #endif 实现
 *   3. 各模块的 .h 文件 #include "rtt_manager.h" 以获取开关定义
 *
 * 模块分类：
 *   ADP_*    - 硬件适配层 (Adapter)
 *   APP_*    - 应用层 (Application)
 *   DEV_*    - 设备层 (Device)
 *   UTILS_*  - 工具层 (Utils)
 *============================================================================*/

// #define PARAM_PRINT_DBG             /* 参数定时打印 (每5秒打印一次) */   // 添加这行

 /*=============================================================================
 * App_FaultHandler 调试宏
 *============================================================================*/
// #define APP_FAULT_HANDLER_DBG

/*=============================================================================
 * [ADP] 硬件适配层调试开关
 *============================================================================*/
// #define ADP_CLOCK_DEBUG             /* Sysclk 时钟调试 */
// #define ADP_RS485_DEBUG             /* RS485 通用调试 */
// #define ADP_RS485_FRAME_DEBUG       /* RS485 帧调试 */
// #define ADP_RS485_WARN_DEBUG        /* RS485 警告调试 */
// #define ADP_RS485_ERR_DEBUG         /* RS485 错误调试 */
// #define ADP_RS485_DEBUG
// #define ADP_DMA_DEBUG               /* DMA 调试 */
// #define ADC_Adp_DEBUG               /* ADC 调试 */
// #define ADP_FLASH_DEBUG             /* Flash 调试 */



/*=============================================================================
 * [APP] 应用层调试开关
 *============================================================================*/
// #define APP_MODBUS_INIT_DBG         /* Modbus 初始化 */
// #define APP_MODBUS_POLL_DBG         /* Modbus 轮询 */
// #define APP_MODBUS_RX_DBG           /* Modbus 原始帧接收 */
// #define APP_MODBUS_PARSE_DBG        /* Modbus 帧解析 */
// #define APP_MODBUS_CRC_DBG          /* Modbus CRC 校验（日志量大，调试时按需开启） */
// #define APP_MODBUS_AUTO_DBG         /* Modbus 自动发送（从机模式默认关闭） */

/*=============================================================================
 * [DEV] 设备层调试开关
 *============================================================================*/
// #define DEV_EVENT_BUS               /* EventBus 事件总线 */
// #define DEV_EVENT_BUS_VERBOSE       /* EventBus 详细调试 */
// #define DEV_RTURN                   /* 旋转编码器 */
// #define DEV_POWER                   /* 电源 */
// #define DEV_MOTOR                   /* 电机 */
// #define DEV_HALL                    /* 霍尔传感器 */
// #define DEV_VOLTAGE                 /* 电压 */
// #define DEV_ADC                     /* ADC */
// #define DEV_SENSOR                  /* 传感器(高频) */
// #define DEV_SENSOR_REAL             /* 传感器真实模式调试(ADC采样+过流检测) */
// #define DEV_SENSOR_SLOW             /* 传感器(慢速) */
// #define DEBUG_SENSOR_SLOW
// #define DEV_MOTOR_HALL              /* 电机霍尔 */
// #define DEV_MOTOR_HALL_OUTPUT       /* 电机霍尔输出 */

/*=============================================================================
 * [UTILS] 工具层调试开关
 *============================================================================*/
// #define UTILS_RING_BUF              /* 环形缓冲区 */
// #define PARAM_DEBUG                 /* 参数管理 */
// #define APP_PARAMS_DBG              /* 参数读写接口 (App_Params) */
// #define QUEUE_INIT_PRINT            /* 队列初始化 */
// #define QUEUE_SEND_PRINT            /* 队列发送 */
// #define QUEUE_RECV_PRINT            /* 队列接收 */
// #define QUEUE_STATS_PRINT           /* 队列统计 */
// #define LOCK_INIT_PRINT             /* 锁初始化 */
// #define LOCK_TRY_PRINT              /* 锁尝试获取 */
// #define LOCK_LOCK_PRINT             /* 锁阻塞获取 */
// #define LOCK_UNLOCK_PRINT           /* 锁释放 */

// 带间隔控制的打印函数结构体
typedef struct {
    NonBlockingDelay_t delay;
    uint32_t intervalMs;
    uint32_t skippedCount;
} RTT_IntervalPrinter_t;

// 声明带间隔的打印函数
// 用法: INTERVAL_DECLARE(SENSOR_D, 1000); 然后就可以用 SENSOR_D("value=%d", val);
#define INTERVAL_DECLARE(name, interval_ms) \
    static RTT_IntervalPrinter_t name##_printer = { \
        .delay.startTick = 0, \
        .delay.delayMs = ((interval_ms) == -1 ? 0 : (interval_ms)), \
        .delay.isRunning = false, \
        .intervalMs = (interval_ms), \
        .skippedCount = 0, \
    }; \
    void name(const char* format, ...) { \
        va_list _args; \
        va_start(_args, format); \
        if ((interval_ms) != -1) { \
            _rtt_interval_print(&name##_printer, format, _args); \
        } \
        va_end(_args); \
    }
// 内部函数，由宏调用
void _rtt_interval_print(RTT_IntervalPrinter_t* printer, const char* format, va_list args);

// 获取丢弃次数
uint32_t RTT_GetSkippedCount(RTT_IntervalPrinter_t* printer);

// 清除丢弃计数
void RTT_ClearSkippedCount(RTT_IntervalPrinter_t* printer);


#endif /* __RTT_MANAGER_H__ */

