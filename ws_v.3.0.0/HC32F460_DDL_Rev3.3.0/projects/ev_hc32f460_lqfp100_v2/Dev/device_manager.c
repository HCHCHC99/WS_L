#include "device_manager.h"
#include <stddef.h>
#include "rtt_log.h"
static DeviceNode_t s_device_registry[MAX_DEVICES];
static DeviceManagerConfig_t s_config;
static uint8_t s_initialized = 0;

// 内部辅助函数
static uint8_t DeviceManager_GetCurrentTaskId(void);
static bool DeviceManager_IsTimeout(uint64_t start_time, uint32_t timeout_ms);
static bool DeviceManager_ShouldUpdate(DeviceNode_t* node);

// 获取当前任务ID
static uint8_t DeviceManager_GetCurrentTaskId(void) {
    // 简单实现：可根据实际情况扩展
    return 1;  // 主循环上下文
}

// 检查是否超时
static bool DeviceManager_IsTimeout(uint64_t start_time, uint32_t timeout_ms) {
    if (timeout_ms == 0) return false;
    uint64_t elapsed = tickTimer_GetCount() - start_time;
    return (elapsed >= timeout_ms);
}

// 判断设备是否应该被更新
static bool DeviceManager_ShouldUpdate(DeviceNode_t* node) {
    if (node == NULL) return false;
    
    // 1. 检查设备状态：只有就绪状态的设备才能更新
    if (node->state != DEVICE_STATE_READY) return false;
    
    // 2. 检查是否允许更新
    if (node->update_allow == 0) return false;
    
    // 3. 检查更新间隔（时间片）
    if (node->update_interval == 0) {
        return true;
    } else {
        uint64_t current_time = tickTimer_GetCount();
        uint64_t elapsed = current_time - node->last_update_time;
        if (elapsed >= node->update_interval) {
            return true;
        }
    }
    
    return false;
}

// 初始化管理器
void DeviceManager_Init(const DeviceManagerConfig_t* config) {
    memset(s_device_registry, 0, sizeof(s_device_registry));
    
    if (config != NULL) {
        s_config = *config;
    } else {
        s_config.operation_timeout_ms = 1000;
        s_config.enable_mutex = 1;
        s_config.auto_subscribe = 1;
    }
    
    s_initialized = 1;
}

// 注册设备
DeviceResult_t DeviceManager_Register(uint8_t id, const char* name, DeviceType_t type, void* priv, DeviceOps_t ops) {
    if (!s_initialized) return RESULT_ERROR;
    if (id >= MAX_DEVICES) return RESULT_PARAM_ERR;
    
    __disable_irq();
    
    if (s_device_registry[id].used) {
        __enable_irq();
        return RESULT_PARAM_ERR;
    }
    
    s_device_registry[id].id = id;
    s_device_registry[id].type = type;
    s_device_registry[id].private_data = priv;
    s_device_registry[id].ops = ops;
    strncpy(s_device_registry[id].name, name, sizeof(s_device_registry[id].name) - 1);
    s_device_registry[id].state = DEVICE_STATE_UNINIT;
    s_device_registry[id].used = 1;
    s_device_registry[id].mutex_owner = 0;
    s_device_registry[id].mutex_lock_time = 0;
    
    // 初始化调度相关字段
    s_device_registry[id].update_interval = 0;
    s_device_registry[id].update_allow = 1;
    s_device_registry[id].last_update_time = tickTimer_GetCount();
    
    // 初始化EventBus订阅记录
    memset(s_device_registry[id].subscribed_topics, 0, sizeof(s_device_registry[id].subscribed_topics));
    
    __enable_irq();
    
    return RESULT_OK;
}

// 去注册设备
DeviceResult_t DeviceManager_Unregister(uint8_t id) {
    if (!s_initialized) return RESULT_ERROR;
    if (id >= MAX_DEVICES) return RESULT_PARAM_ERR;
    
    __disable_irq();
    
    if (!s_device_registry[id].used) {
        __enable_irq();
        return RESULT_NOT_FOUND;
    }
    
    DeviceNode_t* node = &s_device_registry[id];
    
    // 取消所有订阅
    for (int i = 0; i < TOPIC_MAX; i++) {
        if (node->subscribed_topics[i]) {
            // 注意：EventBus当前版本不支持取消订阅，这里只是清空记录
            // 如需完整功能，需要在EventBus中添加取消订阅接口
            node->subscribed_topics[i] = 0;
        }
    }
    
    if (node->state != DEVICE_STATE_UNINIT && node->ops.deinit != NULL) {
        __enable_irq();
        node->ops.deinit(node->private_data);
        __disable_irq();
    }
    
    memset(node, 0, sizeof(DeviceNode_t));
    
    __enable_irq();
    
    return RESULT_OK;
}

DeviceNode_t* DeviceManager_Get(uint8_t id) {
    if (!s_initialized) return NULL;
    if (id >= MAX_DEVICES) return NULL;
    if (!s_device_registry[id].used) return NULL;
    
    // MAIN_D("DeviceManager_Get: id=%d, private_data=0x%p\r\n", 
    //        id, s_device_registry[id].private_data);
    
    return &s_device_registry[id];
}

// 锁定设备
static DeviceResult_t DeviceManager_Lock(uint8_t id, uint32_t timeout_ms) {
    if (!s_initialized) return RESULT_ERROR;
    if (!s_config.enable_mutex) return RESULT_OK;
    
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return RESULT_NOT_FOUND;
    
    uint8_t current_task = DeviceManager_GetCurrentTaskId();
    uint64_t start_time = tickTimer_GetCount();
    
    while (1) {
        __disable_irq();
        
        if (node->mutex_owner == 0) {
            node->mutex_owner = current_task;
            node->mutex_lock_time = tickTimer_GetCount();
            __enable_irq();
            return RESULT_OK;
        } else if (node->mutex_owner == current_task) {
            __enable_irq();
            return RESULT_OK;
        }
        
        __enable_irq();
        
        if (timeout_ms != 0 && DeviceManager_IsTimeout(start_time, timeout_ms)) {
            return RESULT_TIMEOUT;
        }
        
        // 短暂延时
        volatile uint32_t delay_cnt = 0;
        for (uint32_t i = 0; i < 100; i++) {
            delay_cnt++;
        }
    }
}

// 解锁设备
static void DeviceManager_Unlock(uint8_t id) {
    if (!s_initialized) return;
    if (!s_config.enable_mutex) return;
    
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return;
    
    uint8_t current_task = DeviceManager_GetCurrentTaskId();
    
    __disable_irq();
    
    if (node->mutex_owner == current_task) {
        node->mutex_owner = 0;
        node->mutex_lock_time = 0;
    }
    
    __enable_irq();
}

// ========== 设备更新调度接口实现 ==========

// 更新单个设备
DeviceResult_t DeviceManager_Update(uint8_t id) {
    if (!s_initialized) return RESULT_ERROR;
    
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return RESULT_NOT_FOUND;
    if (!node->ops.update) return RESULT_PARAM_ERR;
    
    if (!DeviceManager_ShouldUpdate(node)) {
        return RESULT_OK;
    }
    
    if (DeviceManager_Lock(id, 0) != RESULT_OK) {
        return RESULT_BUSY;
    }
    
    DeviceResult_t res = node->ops.update(node->private_data);
    node->last_update_time = tickTimer_GetCount();
    
    DeviceManager_Unlock(id);
    
    return res;
}

// 更新所有设备
void DeviceManager_UpdateAll(void) {
    if (!s_initialized) return;
    
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (!s_device_registry[i].used) continue;
        if (!s_device_registry[i].ops.update) continue;
        
        DeviceManager_Update(i);
    }
}

// 开启设备的update_allow
void DeviceManager_EnableUpdate(uint8_t id) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return;
    
    __disable_irq();
    node->update_allow = 1;
    __enable_irq();
}

// 关闭设备的update_allow
void DeviceManager_DisableUpdate(uint8_t id) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return;
    
    __disable_irq();
    node->update_allow = 0;
    __enable_irq();
}

// 获取设备的update_allow状态
uint8_t DeviceManager_IsUpdateEnabled(uint8_t id) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return 0;
    
    return node->update_allow;
}

// 设置设备更新间隔
void DeviceManager_SetUpdateInterval(uint8_t id, uint16_t interval_ms) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return;
    
    __disable_irq();
    node->update_interval = interval_ms;
    node->last_update_time = tickTimer_GetCount();
    __enable_irq();
}

// 获取设备更新间隔
uint16_t DeviceManager_GetUpdateInterval(uint8_t id) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return 0;
    
    return node->update_interval;
}

// ========== EventBus集成接口实现 ==========

// 设备订阅主题
bool DeviceManager_Subscribe(uint8_t id, Topic_t topic, EventCallback callback) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return false;
    if (topic >= TOPIC_MAX || callback == NULL) return false;
    
    // 检查是否已经订阅
    if (node->subscribed_topics[topic]) {
        return true;  // 已经订阅过
    }
    
    // 调用EventBus订阅（使用优先级0）
    if (EventBus_Subscribe(topic, callback, 0)) {
        node->subscribed_topics[topic] = 1;
        return true;
    }
    
    return false;
}

// 设备取消订阅主题
bool DeviceManager_Unsubscribe(uint8_t id, Topic_t topic) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return false;
    if (topic >= TOPIC_MAX) return false;
    
    if (node->subscribed_topics[topic]) {
        // 注意：当前EventBus实现不支持取消订阅
        // 这里只清空记录，实际EventBus需要扩展此功能
        node->subscribed_topics[topic] = 0;
        return true;
    }
    
    return false;
}

// 设备发布事件（主动设备使用）
void DeviceManager_Publish(uint8_t id, Topic_t topic, void* payload) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return;
    if (topic >= TOPIC_MAX) return;
    
    // 发布到EventBus
    EventBus_Publish(topic, payload);
}

// 为设备设置默认的事件回调
void DeviceManager_SetEventCallback(uint8_t id, EventCallback callback) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node) return;
    
    // 可以为特定设备类型自动订阅相关主题
    // 例如：电机设备订阅速度反馈主题
    // 这里只是示例，具体订阅由用户决定
}

// 启用所有设备的更新
void DeviceManager_EnableAllUpdate(void) {
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (s_device_registry[i].used) {
            DeviceManager_EnableUpdate(i);
        }
    }
}

// 禁用所有设备的更新
void DeviceManager_DisableAllUpdate(void) {
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (s_device_registry[i].used) {
            DeviceManager_DisableUpdate(i);
        }
    }
}

// 重置所有设备的更新计时器
void DeviceManager_ResetAllUpdateTimers(void) {
    uint64_t current_time = tickTimer_GetCount();
    
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (s_device_registry[i].used) {
            s_device_registry[i].last_update_time = current_time;
        }
    }
}

// 按类型查找设备
DeviceNode_t* DeviceManager_FindByType(DeviceType_t type) {
    if (!s_initialized) return NULL;
    
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (s_device_registry[i].used && s_device_registry[i].type == type) {
            return &s_device_registry[i];
        }
    }
    return NULL;
}

// 按名称查找设备
DeviceNode_t* DeviceManager_FindByName(const char* name) {
    if (!s_initialized || name == NULL) return NULL;
    
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        if (s_device_registry[i].used && 
            strcmp(s_device_registry[i].name, name) == 0) {
            return &s_device_registry[i];
        }
    }
    return NULL;
}

// ========== 应用层统一接口 ==========

DeviceResult_t Device_Init(uint8_t id) {
    DeviceResult_t res;
    
    if (DeviceManager_Lock(id, s_config.operation_timeout_ms) != RESULT_OK) {
        return RESULT_BUSY;
    }
    
    res = Device_Init_NoLock(id);
    
    DeviceManager_Unlock(id);
    return res;
}

DeviceResult_t Device_Deinit(uint8_t id) {
    DeviceResult_t res;
    
    if (DeviceManager_Lock(id, s_config.operation_timeout_ms) != RESULT_OK) {
        return RESULT_BUSY;
    }
    
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node || !node->ops.deinit) {
        DeviceManager_Unlock(id);
        return RESULT_PARAM_ERR;
    }
    
    res = node->ops.deinit(node->private_data);
    if (res == RESULT_OK) {
        node->state = DEVICE_STATE_DEINIT;
    }
    
    DeviceManager_Unlock(id);
    return res;
}

DeviceResult_t Device_Read(uint8_t id, void* data, uint32_t size) {
    DeviceResult_t res;
    
    if (data == NULL && size > 0) return RESULT_PARAM_ERR;
    if (DeviceManager_Lock(id, s_config.operation_timeout_ms) != RESULT_OK) {
        return RESULT_BUSY;
    }
    
    res = Device_Read_NoLock(id, data, size);
    
    DeviceManager_Unlock(id);
    return res;
}

DeviceResult_t Device_Write(uint8_t id, const void* data, uint32_t size) {
    DeviceResult_t res;
    
    if (data == NULL && size > 0) return RESULT_PARAM_ERR;
    if (DeviceManager_Lock(id, s_config.operation_timeout_ms) != RESULT_OK) {
        return RESULT_BUSY;
    }
    
    res = Device_Write_NoLock(id, data, size);
    
    DeviceManager_Unlock(id);
    return res;
}

DeviceResult_t Device_Control(uint8_t id, DeviceCommandData_t* cmd) {
    DeviceResult_t res;
    
    if (cmd == NULL) return RESULT_PARAM_ERR;
    if (DeviceManager_Lock(id, s_config.operation_timeout_ms) != RESULT_OK) {
        return RESULT_BUSY;
    }
    
    res = Device_Control_NoLock(id, cmd);
    
    DeviceManager_Unlock(id);
    return res;
}

// ========== 应用层统一接口（无锁版本） ==========

DeviceResult_t Device_Init_NoLock(uint8_t id) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node || !node->ops.init) return RESULT_PARAM_ERR;
    
    DeviceResult_t res = node->ops.init(node->private_data);
    node->state = (res == RESULT_OK) ? DEVICE_STATE_READY : DEVICE_STATE_ERROR;
    return res;
}

DeviceResult_t Device_Read_NoLock(uint8_t id, void* data, uint32_t size) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node || !node->ops.read) return RESULT_PARAM_ERR;
    
    if (node->state == DEVICE_STATE_UNINIT) return RESULT_ERROR;
    if (node->state == DEVICE_STATE_ERROR) return RESULT_ERROR;
    
    return node->ops.read(node->private_data, data, size);
}

DeviceResult_t Device_Write_NoLock(uint8_t id, const void* data, uint32_t size) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node || !node->ops.write) return RESULT_PARAM_ERR;
    
    if (node->state == DEVICE_STATE_UNINIT) return RESULT_ERROR;
    if (node->state == DEVICE_STATE_ERROR) return RESULT_ERROR;
    
    return node->ops.write(node->private_data, data, size);
}

DeviceResult_t Device_Control_NoLock(uint8_t id, DeviceCommandData_t* cmd) {
    DeviceNode_t* node = DeviceManager_Get(id);
    if (!node || !node->ops.control) return RESULT_PARAM_ERR;
    
    if (node->state == DEVICE_STATE_UNINIT) return RESULT_ERROR;
    if (node->state == DEVICE_STATE_ERROR) return RESULT_ERROR;
    
    return node->ops.control(node->private_data, cmd);
}
