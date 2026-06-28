#ifndef __MSG_QUEUE_H__
#define __MSG_QUEUE_H__

#include "stdint.h"
#include "stdbool.h"
#include "lock.h"
#include "rtt_manager.h"

/*=============================================================================
 * 调试宏定义（旧名称兼容）
 * 开关在 rtt_manager.h 中统一管理：QUEUE_xxx_PRINT
 *============================================================================*/
#ifdef QUEUE_INIT_PRINT
    #define QUEUE_INIT_PRINT(fmt, ...)   MAIN_D("[QUEUE_INIT] " fmt, ##__VA_ARGS__)
#else
    #define QUEUE_INIT_PRINT(fmt, ...)   ((void)0)
#endif

#ifdef QUEUE_SEND_PRINT
    #define QUEUE_SEND_PRINT(fmt, ...)   MAIN_D("[QUEUE_SEND] " fmt, ##__VA_ARGS__)
#else
    #define QUEUE_SEND_PRINT(fmt, ...)   ((void)0)
#endif

#ifdef QUEUE_RECV_PRINT
    #define QUEUE_RECV_PRINT(fmt, ...)   MAIN_D("[QUEUE_RECV] " fmt, ##__VA_ARGS__)
#else
    #define QUEUE_RECV_PRINT(fmt, ...)   ((void)0)
#endif

#ifdef QUEUE_STATS_PRINT
    #define QUEUE_STATS_PRINT(fmt, ...)  MAIN_D("[QUEUE_STATS] " fmt, ##__VA_ARGS__)
#else
    #define QUEUE_STATS_PRINT(fmt, ...)  ((void)0)
#endif

/* 消息优先级 */
typedef enum {
    MSG_PRIO_URGENT = 0,
    MSG_PRIO_HIGH = 1,
    MSG_PRIO_NORMAL = 2,
    MSG_PRIO_LOW = 3
} msg_prio_t;

/* 消息结构体 */
typedef struct {
    msg_prio_t priority;
    uint16_t type;
    uint16_t len;
    uint32_t timestamp;
    uint8_t *data;
    uint8_t buffer[256];
    void (*callback)(void*);
    void *callback_arg;
} msg_t;

/* 队列配置 */
typedef struct {
    uint16_t max_size;
    bool overwrite;
    bool priority_enabled;
    uint16_t timeout_ms;
} queue_config_t;

/* 队列统计信息 */
typedef struct {
    uint16_t total_enqueued;
    uint16_t total_dequeued;
    uint16_t total_dropped;
    uint16_t max_usage;
    uint16_t overflow_count;
} queue_stats_t;

/* 队列结构体 */
typedef struct {
    msg_t *buffer;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
    queue_config_t config;
    queue_stats_t stats;
    mutex_t lock;
    char name[32];           /* 队列名称，用于调试 */
} msg_queue_t;

/* 初始化消息队列 */
void MsgQueue_Init(msg_queue_t *queue, msg_t *buffer, uint16_t size, queue_config_t *config, const char *name);

/* 发送消息 */
bool MsgQueue_Send(msg_queue_t *queue, msg_t *msg, bool blocking, const char *caller);

/* 发送紧急消息 */
bool MsgQueue_SendUrgent(msg_queue_t *queue, msg_t *msg, const char *caller);

/* 接收消息 */
bool MsgQueue_Receive(msg_queue_t *queue, msg_t *msg, uint32_t timeoutMs, const char *caller);

/* 查看队首消息 */
bool MsgQueue_Peek(msg_queue_t *queue, msg_t *msg);

/* 获取队列中的消息数量 */
uint16_t MsgQueue_GetCount(msg_queue_t *queue);

/* 获取队列统计信息 */
void MsgQueue_GetStats(msg_queue_t *queue, queue_stats_t *stats);

/* 清空队列 */
void MsgQueue_Clear(msg_queue_t *queue);

/* 根据类型删除消息 */
uint16_t MsgQueue_DeleteByType(msg_queue_t *queue, uint16_t type);

/* 等待队列非空 */
bool MsgQueue_WaitNotEmpty(msg_queue_t *queue, uint32_t timeoutMs);

#endif
