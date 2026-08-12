#include "lock.h"
#include "TickTimer.h"
#include <stddef.h>

void Lock_Init(mutex_t *lock, lock_type_t type, const char *name)
{
    if (lock != NULL) {
        lock->state = LOCK_FREE;
        lock->owner_task = 0;
        lock->lock_count = 0;
        lock->wait_count = 0;
        lock->type = type;
        lock->lock_time = 0;
        LOCK_INIT_PRINT("Init lock '%s' (type=%d)", name, type);
    }
}

bool Lock_TryLock(mutex_t *lock, const char *caller)
{
    bool result = false;
    
    if (lock == NULL) return false;
    
    __disable_irq();
    
    if (lock->state == LOCK_FREE) {
        lock->state = LOCK_BUSY;
        lock->owner_task = 1;
        lock->lock_count = 1;
        lock->lock_time = tickTimer_GetCount();
        result = true;
        LOCK_TRY_PRINT("SUCCESS - caller='%s', lock acquired, owner=1", caller);
    } 
    else if (lock->type == LOCK_TYPE_RECURSIVE && lock->owner_task == 1) {
        lock->lock_count++;
        result = true;
        LOCK_TRY_PRINT("RECURSIVE - caller='%s', lock_count=%d", caller, lock->lock_count);
    }
    else {
        LOCK_TRY_PRINT("FAILED - caller='%s', lock is BUSY, owner=%d, wait_count=%d", 
                   caller, lock->owner_task, lock->wait_count);
    }
    
    __enable_irq();
    return result;
}

bool Lock_Lock(mutex_t *lock, uint32_t timeoutMs, const char *caller)
{
    uint64_t startTick;
    
    if (lock == NULL) return false;
    
    LOCK_LOCK_PRINT("request - caller='%s', timeout=%dms", caller, timeoutMs);
    
    startTick = tickTimer_GetCount();
    
    while (1) {
        if (Lock_TryLock(lock, caller)) {
            LOCK_LOCK_PRINT("SUCCESS - caller='%s', elapsed=%dms", 
                       caller, (uint32_t)(tickTimer_GetCount() - startTick));
            return true;
        }
        
        if (timeoutMs > 0) {
            if ((tickTimer_GetCount() - startTick) >= timeoutMs) {
                LOCK_LOCK_PRINT("TIMEOUT - caller='%s', timeout=%dms", caller, timeoutMs);
                return false;
            }
        }
        
        /* simple delay to yield CPU */
        for (volatile int i = 0; i < 100; i++);
    }
}

void Lock_Unlock(mutex_t *lock, const char *caller)
{
    if (lock == NULL) return;
    
    __disable_irq();
    
    uint32_t elapsed = tickTimer_GetCount() - lock->lock_time;
    
    if (lock->type == LOCK_TYPE_RECURSIVE && lock->lock_count > 1) {
        lock->lock_count--;
        LOCK_UNLOCK_PRINT("RECURSIVE - caller='%s', lock_count=%d (still locked)", 
                   caller, lock->lock_count);
    } else {
        if (lock->state == LOCK_BUSY) {
            LOCK_UNLOCK_PRINT("SUCCESS - caller='%s', held_time=%dms, wait_count=%d", 
                       caller, (uint32_t)elapsed, lock->wait_count);
        } else {
            LOCK_UNLOCK_PRINT("WARNING - caller='%s', lock already FREE!", caller);
        }
        lock->state = LOCK_FREE;
        lock->owner_task = 0;
        lock->lock_count = 0;
        lock->lock_time = 0;
    }
    
    __enable_irq();
}

bool Lock_IsLocked(mutex_t *lock)
{
    if (lock == NULL) return false;
    return (lock->state == LOCK_BUSY);
}

uint32_t Lock_GetWaitCount(mutex_t *lock)
{
    if (lock == NULL) return 0;
    return lock->wait_count;
}
