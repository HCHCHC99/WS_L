#include "TickTimer.h"
#include <stddef.h>

static volatile uint64_t s_tickCount = 0;
static uint64_t s_lastCallTick = 0;

// 初始化滴答定时器
void tickTimer_Init(void)
{
    s_tickCount = 0;
    s_lastCallTick = tickTimer_GetCount(); 
}

// Systick.c 中添加函数实现
uint64_t tickTimer_GetRawTick(void) {
    uint64_t tick;
    //__disable_irq();  // 关中断，避免读取时被中断修改（保证数据完整性）
    tick = s_tickCount;
    //__enable_irq();
    return tick;
}

// 获取当前滴答数
uint64_t tickTimer_GetCount(void)
{
    uint64_t tick;
    //__disable_irq();
    tick = s_tickCount;
    //__enable_irq();
    return tick;
}

// 阻塞式延时
void tickTimer_DelayMs(uint64_t ms)
{
    uint64_t startTick = tickTimer_GetCount();
    while ((tickTimer_GetCount() - startTick) < ms);
}

// 定时器中断中调用
__attribute__((section(".ramfunc"))) void tickTimer_Update(void)
{
    s_tickCount++;
}

// 非阻塞延时函数实现

void nbDelay_Init(NonBlockingDelay_t* delayObj, uint64_t delayMs) {
    // 新增：参数合法性检查（避免空指针和无效延时）
    if (delayObj == NULL) {
        return;  // 或断言：assert(delayObj != NULL);（需包含assert.h）
    }

    
    delayObj->startTick = 0;
    delayObj->delayMs = delayMs;
    delayObj->isRunning = false;
}

void nbDelay_Start(NonBlockingDelay_t* delayObj)
{
    delayObj->startTick = tickTimer_GetCount();
    delayObj->isRunning = true;
}

bool nbDelay_IsComplete(NonBlockingDelay_t* delayObj)
{
    if (!delayObj->isRunning) {
        return false;
    }
    
    uint64_t currentTick = tickTimer_GetCount();
    uint64_t elapsed = currentTick - delayObj->startTick;
    
    if (elapsed >= delayObj->delayMs) {
        delayObj->isRunning = false;
        return true;
    }
    
    return false;
}

// 新增函数：检查延时是否完成但不结束延时
bool nbDelay_IsComplete_noclose(NonBlockingDelay_t* delayObj)
{
    if (!delayObj->isRunning) {
        return false;
    }
    
    uint64_t currentTick = tickTimer_GetCount();
    uint64_t elapsed = currentTick - delayObj->startTick;
    
    // 只检查是否完成，不修改isRunning状态
    return (elapsed >= delayObj->delayMs);
}

void nbDelay_Stop(NonBlockingDelay_t* delayObj)
{
    delayObj->isRunning = false;
}

void nbDelay_SetTime(NonBlockingDelay_t* delayObj, uint64_t delayMs) {
    if (delayObj == NULL) {
        return;
    }

    delayObj->delayMs = delayMs;
}

// 获取距离上次调用经过的滴答数
uint64_t tickTimer_GetElapsedSinceLastCall(void)
{
    uint64_t currentTick;
    uint64_t elapsed;
    
    //__disable_irq();
    currentTick = s_tickCount;  // 直接访问，避免函数调用
    elapsed = currentTick - s_lastCallTick;
    s_lastCallTick = currentTick;
    //__enable_irq();
    
    return elapsed;
}
