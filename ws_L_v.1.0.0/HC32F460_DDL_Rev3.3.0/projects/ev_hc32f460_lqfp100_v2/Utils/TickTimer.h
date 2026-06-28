#ifndef TICKTIMER_H
#define TICKTIMER_H

#include "stdint.h"
#include "stdbool.h"
// #include "stdlib.h"

// 非阻塞延时器结构体
typedef struct {
    uint64_t startTick;   // 延时启动时的滴答计数（调用nbDelay_Start时记录）
    uint64_t delayMs;     // 目标延时时间（单位：ms，通过nbDelay_Init/nbDelay_SetTime设置）
    bool isRunning;       // 延时器运行状态：true=正在延时，false=未启动/已完成
} NonBlockingDelay_t;

// 初始化滴答定时器
void tickTimer_Init(void);

// 获取当前滴答数（毫秒）
uint64_t tickTimer_GetCount(void);

// 阻塞式延时（单位：毫秒）
void tickTimer_DelayMs(uint64_t ms);

// 中断服务函数中调用的滴答更新函数
// 1ms 调用一次
void tickTimer_Update(void);

// 非阻塞延时函数组

// 初始化非阻塞延时器
void nbDelay_Init(NonBlockingDelay_t* delayObj, uint64_t delayMs);

// 启动非阻塞延时
void nbDelay_Start(NonBlockingDelay_t* delayObj);


// 检查非阻塞延时是否完成（完成后会结束延时）
bool nbDelay_IsComplete(NonBlockingDelay_t* delayObj);

// 检查非阻塞延时是否完成（完成后不会结束延时）
bool nbDelay_IsComplete_noclose(NonBlockingDelay_t* delayObj);

// 停止非阻塞延时
void nbDelay_Stop(NonBlockingDelay_t* delayObj);

// 设置新的延时时间（不启动）
void nbDelay_SetTime(NonBlockingDelay_t* delayObj, uint64_t delayMs);

// 获取距离上次调用经过的滴答数
uint64_t tickTimer_GetElapsedSinceLastCall(void);

uint64_t tickTimer_GetRawTick(void);

#endif // TICKTIMER_H
