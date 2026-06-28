#ifndef EVENT_BUS_H_
#define EVENT_BUS_H_

// EventBus - 事件总线系统，用于发布订阅模式的事件处理
#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

// ========== 调试宏定义 ==========
// 开关在 rtt_manager.h 中统一管理：DEV_EVENT_BUS / DEV_EVENT_BUS_VERBOSE

#ifdef DEV_EVENT_BUS
    #define EVENT_BUS_DEBUG_PRINT(fmt, ...)    MAIN_D("[EVENT_BUS] " fmt, ##__VA_ARGS__)
#else
    #define EVENT_BUS_DEBUG_PRINT(fmt, ...)    ((void)0)
#endif

#ifdef DEV_EVENT_BUS_VERBOSE
    #define EVENT_BUS_VERBOSE_PRINT(fmt, ...)  MAIN_D("[EVENT_BUS] " fmt, ##__VA_ARGS__)
#else
    #define EVENT_BUS_VERBOSE_PRINT(fmt, ...)  ((void)0)
#endif

typedef enum {
    TOPIC_POWER = 0,
    TOPIC_LIMIT_HARD,
    TOPIC_LIMIT_SOFT,
    TOPIC_CAN_EVENT,
    TOPIC_MOTOR_CMD,
    TOPIC_MOTOR_SPEED_FEEDBACK, 
    TOPIC_MOTOR_DRIVE_EXEC,
    TOPIC_MANUAL_IO,
    TOPIC_ALARM,
    TOPIC_VOLTAGE_ALARM,
    TOPIC_CURRENT_ALARM,
    TOPIC_RTURN_LIMIT,
    TOPIC_FAULT_CLEAR,
    TOPIC_MANUAL_RS485,
    TOPIC_MAX
} Topic_t;

// 主题名称映射（用于调试打印）
static const char* const g_topic_names[] = {
    [TOPIC_POWER]               = "POWER",
    [TOPIC_LIMIT_HARD]          = "LIMIT_HARD",
    [TOPIC_LIMIT_SOFT]          = "LIMIT_SOFT",
    [TOPIC_CAN_EVENT]           = "CAN_EVENT",
    [TOPIC_MOTOR_CMD]           = "MOTOR_CMD",
    [TOPIC_MOTOR_SPEED_FEEDBACK] = "MOTOR_SPEED_FEEDBACK",
    [TOPIC_MOTOR_DRIVE_EXEC]    = "MOTOR_DRIVE_EXEC",
    [TOPIC_MANUAL_IO]           = "MANUAL_IO",
    [TOPIC_ALARM]               = "ALARM",
    [TOPIC_VOLTAGE_ALARM]       = "VOLTAGE_ALARM",
    [TOPIC_CURRENT_ALARM]       = "CURRENT_ALARM",
    [TOPIC_RTURN_LIMIT]         = "RTURN_LIMIT",
    [TOPIC_FAULT_CLEAR]         = "FAULT_CLEAR",
    [TOPIC_MANUAL_RS485]        = "MANUAL_RS485"
};

typedef void (*EventCallback)(void* payload);

/**
 * @brief 订阅话题（带优先级）
 * @param topic 关注的话题
 * @param callback 触发时的回调函数
 * @param priority 优先级（0=最高，数值越大优先级越低）
 * @return 是否订阅成功
 * 
 * @note 优先级排序规则：
 *       - 数值越小优先级越高（0 > 1 > 2 > ...）
 *       - 同等优先级下，先订阅的先执行
 */
bool EventBus_Subscribe(Topic_t topic, EventCallback callback, uint8_t priority);

void EventBus_Init(void);
void EventBus_Publish(Topic_t topic, void* payload);

/**
 * @brief 启用事件驱动（门控机制）
 *        调用此函数后，所有缓存的 Publish 才会真正执行回调。
 *        在此之前调用的 Publish 仅记录事件标志，不执行回调。
 */
void EventBus_Enable(void);

/**
 * @brief 查询事件总线是否已启用
 * @return true 已启用，false 未启用（初始化阶段）
 */
bool EventBus_IsEnabled(void);

#endif /* EVENT_BUS_H_ */
