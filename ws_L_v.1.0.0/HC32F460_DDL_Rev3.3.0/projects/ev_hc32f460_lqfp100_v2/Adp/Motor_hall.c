#include "Motor_hall.h"
#include "timer6_timebase.h"
#include "TickTimer.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_interrupts.h"
#include <string.h>
#include <stdlib.h>
#include "rtt_log.h"

/* ========== 系统级全局变量 ========== */
static uint8_t g_system_initialized = 0;
static uint8_t g_next_handle_id = 0;

/* ========== 测量参数配置 ========== */
#define MIN_PULSE_INTERVAL_US       (50u)
#define MAX_PULSE_INTERVAL_US       (200000u)
#define STALL_DETECTION_MS          (500u)
#define STOP_DETECTION_MS           (50u)
#define RPM_WINDOW_SIZE             (6)
#define DIRECTION_CONFIRM_COUNT     (3)
#define HALL_CHECK_INTERVAL_MS      (1000)
#define HALL_WARNING_THRESHOLD      (10)

/* ========== 电机实例内部数据结构 ========== */
typedef struct {
    uint8_t id;
    uint8_t valid;
    
    motor_hall_config_t config;
    
    struct {
        uint8_t hall_a_port;
        uint16_t hall_a_pin;
        uint8_t hall_b_port;
        uint16_t hall_b_pin;
        uint8_t irq_priority;
    } irq_config;
    
    volatile uint32_t last_pulse_interval;
    volatile uint64_t last_pulse_time_us;
    volatile uint32_t pulse_counter;
    volatile uint8_t  speed_data_ready;
    volatile float    current_rpm;
    volatile float    filtered_rpm;
    
    volatile uint32_t hall_pulse_counts[2];
    volatile uint8_t  active_hall_count;
    
    volatile motor_direction_t current_direction;
    volatile motor_direction_t last_valid_direction;
    volatile uint8_t  direction_confidence;
    volatile uint8_t  direction_confirm_count;
    volatile uint8_t  direction_data_ready;
    volatile uint8_t  direction_changed;
    
    volatile uint32_t hall_a_last_second_count;
    volatile uint32_t hall_b_last_second_count;
    volatile hall_working_status_t hall_status;
    
    volatile uint8_t  stalled;
    volatile uint8_t  measurement_valid;
    volatile uint32_t total_measurements;
    volatile uint32_t error_count;
    
    float rpm_window[RPM_WINDOW_SIZE];
    uint8_t write_index;
    uint8_t valid_count;
    
    uint32_t interval_history[6];
    uint8_t interval_idx;
    uint8_t interval_valid_count;
    
    NonBlockingDelay_t rpm_timer;
    NonBlockingDelay_t hall_check_timer;
    
    uint8_t is_running;
    uint8_t need_update;
    
} motor_hall_instance_t;

#define MAX_MOTOR_INSTANCES     (4)

static motor_hall_instance_t g_instances[MAX_MOTOR_INSTANCES] = {0};

static motor_hall_instance_t* g_hall_a_map[16] = {NULL};
static motor_hall_instance_t* g_hall_b_map[16] = {NULL};

static motor_hall_instance_t* g_current_hall_a_instance = NULL;
static motor_hall_instance_t* g_current_hall_b_instance = NULL;

/* ========== 内部函数声明 ========== */
static void update_rpm_filter(motor_hall_instance_t* inst, float new_rpm);
static float calculate_average_interval(motor_hall_instance_t* inst);
static float interval_to_rpm(motor_hall_instance_t* inst, uint32_t interval_us);
static void perform_stall_detection(motor_hall_instance_t* inst);
static void check_hall_status(motor_hall_instance_t* inst);
static uint8_t calculate_active_hall_count(motor_hall_instance_t* inst);

/* ========== 系统初始化 ========== */
void motor_hall_system_init(void)
{
    if (g_system_initialized) return;
    
    Timer6_Timebase_Init();
    Timer6_Timebase_Start();
    
    memset(g_instances, 0, sizeof(g_instances));
    memset(g_hall_a_map, 0, sizeof(g_hall_a_map));
    memset(g_hall_b_map, 0, sizeof(g_hall_b_map));
    
    g_system_initialized = 1;
}

/* ========== 辅助函数 ========== */

static uint8_t calculate_active_hall_count(motor_hall_instance_t* inst)
{
    uint8_t count = 0;
    if (inst->config.eirq_ch_a != 0) count++;
    if (inst->config.eirq_ch_b != 0) count++;
    return count;
}

/* ========== 中断回调函数（全局） ========== */

static void hall_a_irq_callback(void)
{
    motor_hall_instance_t* inst = g_current_hall_a_instance;
    if (!inst || !inst->valid) return;
    
    EXTINT_ClearExtIntStatus(inst->config.eirq_ch_a);
    
    inst->hall_pulse_counts[0]++;
    
    uint32_t interval_counter = Timer6_Timebase_GetDelta();
    uint32_t interval_us = Timer6_Timebase_DeltaToUs(interval_counter);
    
    if (interval_us >= MIN_PULSE_INTERVAL_US && interval_us <= MAX_PULSE_INTERVAL_US) 
    {
        inst->last_pulse_interval = interval_us;
        inst->last_pulse_time_us = Timer6_Timebase_GetTimestamp();
        inst->pulse_counter++;
        inst->stalled = 0;
        inst->speed_data_ready = 1;
    }
}

static void hall_b_irq_callback(void)
{
    motor_hall_instance_t* inst = g_current_hall_b_instance;
    if (!inst || !inst->valid) return;
    
    EXTINT_ClearExtIntStatus(inst->config.eirq_ch_b);
    
    inst->hall_pulse_counts[1]++;
    
    uint32_t interval_counter = Timer6_Timebase_GetDelta();
    uint32_t interval_us = Timer6_Timebase_DeltaToUs(interval_counter);
    
    if (interval_us >= MIN_PULSE_INTERVAL_US && interval_us <= MAX_PULSE_INTERVAL_US) 
    {
        inst->last_pulse_interval = interval_us;
        inst->last_pulse_time_us = Timer6_Timebase_GetTimestamp();
        inst->pulse_counter++;
        inst->stalled = 0;
        inst->speed_data_ready = 1;
    }
    
    static uint8_t hall_b_last[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t hall_b_current = (GPIO_ReadInputPins(inst->irq_config.hall_b_port, inst->irq_config.hall_b_pin) == PIN_SET);
    
    inst->direction_data_ready = 1;
    
    if (hall_b_last[inst->id] == 0xFF) {
        hall_b_last[inst->id] = hall_b_current;
        return;
    }
    
    if (hall_b_last[inst->id] == 1 && hall_b_current == 0) {
        if (inst->current_rpm == 0) {
            hall_b_last[inst->id] = hall_b_current;
            return;
        }
        
        uint8_t hall_a_state = (GPIO_ReadInputPins(inst->irq_config.hall_a_port, inst->irq_config.hall_a_pin) == PIN_SET);
        motor_direction_t detected_direction = hall_a_state ? MOTOR_DIRECTION_FORWARD : MOTOR_DIRECTION_REVERSE;
        
        if (inst->current_direction != detected_direction) {
            inst->direction_confirm_count++;
            
            if (inst->direction_confirm_count >= DIRECTION_CONFIRM_COUNT) {
                motor_direction_t old_direction = inst->current_direction;
                inst->current_direction = detected_direction;
                inst->direction_confidence = 100;
                inst->last_valid_direction = detected_direction;
                inst->direction_confirm_count = 0;
                
                if (old_direction != detected_direction) {
                    inst->direction_changed = 1;
                }
            }
        } else {
            inst->direction_confirm_count = 0;
        }
    }
    
    hall_b_last[inst->id] = hall_b_current;
}

static void register_hall_irq(motor_hall_instance_t* inst, uint8_t is_hall_a)
{
    stc_extint_init_t stcExtiConfig;
    stc_irq_signin_config_t stcIrqRegiConf;
    stc_gpio_init_t stcPortInit;
    
    uint32_t eirq_ch;
    IRQn_Type irqn;
    uint32_t irq_src;
    uint8_t port;
    uint16_t pin;
    
    if (is_hall_a) {
        eirq_ch = inst->config.eirq_ch_a;
        irqn = (IRQn_Type)inst->config.irqn_a;
        irq_src = inst->config.irq_src_a;
        port = inst->config.hall_a_port;
        pin = inst->config.hall_a_pin;
        g_current_hall_a_instance = inst;
    } else {
        eirq_ch = inst->config.eirq_ch_b;
        irqn = (IRQn_Type)inst->config.irqn_b;
        irq_src = inst->config.irq_src_b;
        port = inst->config.hall_b_port;
        pin = inst->config.hall_b_pin;
        g_current_hall_b_instance = inst;
    }
    
    memset(&stcExtiConfig, 0, sizeof(stcExtiConfig));
    memset(&stcIrqRegiConf, 0, sizeof(stcIrqRegiConf));
    memset(&stcPortInit, 0, sizeof(stcPortInit));
    
    stcExtiConfig.u32Filter = EXTINT_FILTER_OFF;
    stcExtiConfig.u32FilterClock = EXTINT_FCLK_DIV1;
    stcExtiConfig.u32Edge = EXTINT_TRIG_BOTH;
    EXTINT_Init(eirq_ch, &stcExtiConfig);
    
    GPIO_StructInit(&stcPortInit);
    stcPortInit.u16PinDir = PIN_DIR_IN;
    stcPortInit.u16PinAttr = PIN_ATTR_DIGITAL;
    stcPortInit.u16PullUp = PIN_PU_ON;
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    GPIO_Init(port, pin, &stcPortInit);

    GPIO_ExtIntCmd(port, pin, ENABLE);

    LL_PERIPH_WP(LL_PERIPH_GPIO);
	
    stcIrqRegiConf.enIntSrc = (en_int_src_t)irq_src;
    stcIrqRegiConf.enIRQn = irqn;
    if (is_hall_a) {
        stcIrqRegiConf.pfnCallback = (func_ptr_t)hall_a_irq_callback;
    } else {
        stcIrqRegiConf.pfnCallback = (func_ptr_t)hall_b_irq_callback;
    }
    INTC_IrqSignIn(&stcIrqRegiConf);
    
    EXTINT_ClearExtIntStatus(eirq_ch);
    NVIC_ClearPendingIRQ(irqn);
    NVIC_SetPriority(irqn, inst->config.irq_priority);
    NVIC_EnableIRQ(irqn);
}

static void update_rpm_filter(motor_hall_instance_t* inst, float new_rpm)
{
    inst->rpm_window[inst->write_index] = new_rpm;
    inst->write_index = (inst->write_index + 1) % RPM_WINDOW_SIZE;
    
    if (inst->valid_count < RPM_WINDOW_SIZE) {
        inst->valid_count++;
    }
    
    float sum = 0;
    uint8_t i;
    for (i = 0; i < inst->valid_count; i++) {
        sum += inst->rpm_window[i];
    }
    inst->filtered_rpm = sum / inst->valid_count;
}

static float calculate_average_interval(motor_hall_instance_t* inst)
{
    if (inst->last_pulse_interval >= MIN_PULSE_INTERVAL_US && 
        inst->last_pulse_interval <= MAX_PULSE_INTERVAL_US) {
        
        inst->interval_history[inst->interval_idx] = inst->last_pulse_interval;
        inst->interval_idx = (inst->interval_idx + 1) % 6;
        
        if (inst->interval_valid_count < 6) {
            inst->interval_valid_count++;
        }
        
        inst->measurement_valid = 1;
    }
    
    if (inst->interval_valid_count < 2) {
        return 0;
    }
    
    uint32_t sum = 0;
    uint8_t i;
    for (i = 0; i < inst->interval_valid_count; i++) {
        sum += inst->interval_history[i];
    }
    
    return (float)sum / inst->interval_valid_count;
}

static float interval_to_rpm(motor_hall_instance_t* inst, uint32_t interval_us)
{
    if (interval_us == 0 || interval_us > MAX_PULSE_INTERVAL_US) {
        return 0;
    }
    
    uint16_t pulses_per_rev;
    
    if (inst->config.custom_pulses_per_rev > 0) {
        pulses_per_rev = inst->config.custom_pulses_per_rev;
    } else {
        uint8_t active_halls = calculate_active_hall_count(inst);
        if (active_halls == 0) return 0;
        pulses_per_rev = inst->config.pole_pairs * 2 * active_halls;
    }
    
    float rpm = 60000000.0f / ((float)interval_us * pulses_per_rev);
    
    if (rpm > 100000.0f) rpm = 100000.0f;
    
    return rpm;
}

static void perform_stall_detection(motor_hall_instance_t* inst)
{
    uint64_t current_time_us = Timer6_Timebase_GetTimestamp();
    uint64_t time_since_last_pulse = current_time_us - inst->last_pulse_time_us;
    
    if (inst->current_rpm > 0 && time_since_last_pulse > (STOP_DETECTION_MS * 1000ul)) {
        if (inst->current_direction != MOTOR_DIRECTION_STOP) {
            inst->current_direction = MOTOR_DIRECTION_STOP;
            inst->direction_confidence = 0;
            inst->direction_confirm_count = 0;
            inst->direction_changed = 1;
        }
        inst->is_running = 0;
    } else if (inst->current_rpm > 0) {
        inst->is_running = 1;
    }
    
    if (time_since_last_pulse > (STALL_DETECTION_MS * 1000ul)) {
        inst->stalled = 1;
        inst->current_rpm = 0;
        inst->filtered_rpm = 0;
        inst->is_running = 0;
    }
}

static void check_hall_status(motor_hall_instance_t* inst)
{
    static uint32_t last_check_time[4] = {0};
    uint32_t current_time = tickTimer_GetCount();
    
    if (current_time - last_check_time[inst->id] >= HALL_CHECK_INTERVAL_MS) {
        last_check_time[inst->id] = current_time;
        
        inst->hall_a_last_second_count = inst->hall_pulse_counts[0];
        inst->hall_b_last_second_count = inst->hall_pulse_counts[1];
        
        if (inst->hall_pulse_counts[0] == 0 && inst->hall_pulse_counts[1] == 0) {
            inst->hall_status = HALL_STATUS_NONE;
        } 
        else if (inst->hall_pulse_counts[0] > 0 && inst->hall_pulse_counts[1] == 0) {
            inst->hall_status = HALL_STATUS_A_ONLY;
            inst->active_hall_count = 1;
        }
        else if (inst->hall_pulse_counts[0] == 0 && inst->hall_pulse_counts[1] > 0) {
            inst->hall_status = HALL_STATUS_B_ONLY;
            inst->active_hall_count = 1;
        }
        else {
            uint32_t diff;
            uint32_t total = inst->hall_pulse_counts[0] + inst->hall_pulse_counts[1];
            
            if (inst->hall_pulse_counts[0] > inst->hall_pulse_counts[1]) {
                diff = inst->hall_pulse_counts[0] - inst->hall_pulse_counts[1];
            } else {
                diff = inst->hall_pulse_counts[1] - inst->hall_pulse_counts[0];
            }
            
            if (total > 0 && (diff * 100 / total) > HALL_WARNING_THRESHOLD) {
                inst->hall_status = HALL_STATUS_ERROR;
            } else {
                inst->hall_status = HALL_STATUS_BOTH;
                inst->active_hall_count = 2;
            }
        }
    }
}

static void calculate_rpm(motor_hall_instance_t* inst)
{
    float avg_interval = calculate_average_interval(inst);
    
    if (avg_interval == 0) {
        inst->current_rpm = 0;
        inst->filtered_rpm = 0;
        return;
    }
    
    float raw_rpm = interval_to_rpm(inst, (uint32_t)avg_interval);
    
    if (raw_rpm > 0) {
        inst->current_rpm = raw_rpm;
        update_rpm_filter(inst, raw_rpm);
        inst->total_measurements++;
    } else {
        inst->current_rpm = 0;
        inst->filtered_rpm = 0;
    }
}

/* ========== 公开接口实现 ========== */

motor_hall_handle_t motor_hall_create(const motor_hall_config_t* config)
{
    if (!g_system_initialized) {
        motor_hall_system_init();
    }
    
    uint8_t i;
    for (i = 0; i < MAX_MOTOR_INSTANCES; i++) {
        if (!g_instances[i].valid) {
            break;
        }
    }
    
    if (i >= MAX_MOTOR_INSTANCES) {
        MAIN_D("motor_hall_create: No available instance!\r\n");
        return NULL;
    }
    
    motor_hall_instance_t* inst = &g_instances[i];
    memset(inst, 0, sizeof(motor_hall_instance_t));
    
    // ========== 逐个成员赋值，不使用 memcpy ==========
    // GPIO配置
    inst->config.hall_a_port = config->hall_a_port;
    inst->config.hall_a_pin = config->hall_a_pin;
    inst->config.hall_b_port = config->hall_b_port;
    inst->config.hall_b_pin = config->hall_b_pin;
    
    // 中断配置
    inst->config.eirq_ch_a = config->eirq_ch_a;
    inst->config.eirq_ch_b = config->eirq_ch_b;
    inst->config.irqn_a = config->irqn_a;
    inst->config.irqn_b = config->irqn_b;
    inst->config.irq_src_a = config->irq_src_a;
    inst->config.irq_src_b = config->irq_src_b;
    inst->config.irq_priority = config->irq_priority;
    
    // 电机参数
    inst->config.pole_pairs = config->pole_pairs;
    inst->config.hall_count = config->hall_count;
    inst->config.custom_pulses_per_rev = config->custom_pulses_per_rev;
    
    // 调试打印，确认赋值正确
    MAIN_D("motor_hall_create: instance %d\r\n", i);
    MAIN_D("  hall_a_port=%d, hall_a_pin=0x%04X\r\n", 
           inst->config.hall_a_port, inst->config.hall_a_pin);
    MAIN_D("  hall_b_port=%d, hall_b_pin=0x%04X\r\n", 
           inst->config.hall_b_port, inst->config.hall_b_pin);
    MAIN_D("  eirq_ch_a=%lu, eirq_ch_b=%lu\r\n", 
           inst->config.eirq_ch_a, inst->config.eirq_ch_b);
    MAIN_D("  irqn_a=%d, irqn_b=%d\r\n", 
           inst->config.irqn_a, inst->config.irqn_b);
    MAIN_D("  irq_src_a=%lu, irq_src_b=%lu\r\n", 
           inst->config.irq_src_a, inst->config.irq_src_b);
    
    // 保存 irq_config 副本（用于中断回调）
    inst->irq_config.hall_a_port = config->hall_a_port;
    inst->irq_config.hall_a_pin = config->hall_a_pin;
    inst->irq_config.hall_b_port = config->hall_b_port;
    inst->irq_config.hall_b_pin = config->hall_b_pin;
    inst->irq_config.irq_priority = config->irq_priority;
    
    inst->id = g_next_handle_id++;
    inst->valid = 1;
    inst->current_direction = MOTOR_DIRECTION_NONE;
    inst->hall_status = HALL_STATUS_NONE;
    
    inst->active_hall_count = calculate_active_hall_count(inst);
    
    uint8_t j;
    for (j = 0; j < RPM_WINDOW_SIZE; j++) {
        inst->rpm_window[j] = 0;
    }
    inst->write_index = 0;
    inst->valid_count = 0;
    
    for (j = 0; j < 6; j++) {
        inst->interval_history[j] = 0;
    }
    inst->interval_idx = 0;
    inst->interval_valid_count = 0;
    
    nbDelay_Init(&inst->rpm_timer, 10);
    nbDelay_Start(&inst->rpm_timer);
    
    nbDelay_Init(&inst->hall_check_timer, HALL_CHECK_INTERVAL_MS);
    nbDelay_Start(&inst->hall_check_timer);
    
    if (config->eirq_ch_a != 0) {
        register_hall_irq(inst, 1);
    }
    if (config->eirq_ch_b != 0) {
        register_hall_irq(inst, 0);
    }
    
    return (motor_hall_handle_t)inst;
}

void motor_hall_destroy(motor_hall_handle_t handle)
{
    if (!handle) return;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    inst->valid = 0;
}

void motor_hall_start(motor_hall_handle_t handle)
{
    (void)handle;
}

void motor_hall_stop(motor_hall_handle_t handle)
{
    (void)handle;
}

void motor_hall_update(motor_hall_handle_t handle)
{
    if (!handle) return;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    if (!inst->valid) return;
    
    Timer6_Timebase_UpdateTimestamp();
    
    if (nbDelay_IsComplete(&inst->rpm_timer)) {
        calculate_rpm(inst);
        perform_stall_detection(inst);
        nbDelay_Start(&inst->rpm_timer);
    }
    
    if (nbDelay_IsComplete_noclose(&inst->hall_check_timer)) {
        check_hall_status(inst);
        nbDelay_Start(&inst->hall_check_timer);
    }
}

/* ========== 转速相关接口 ========== */

float motor_hall_get_rpm(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    
    if (!inst->is_running) return 0;
    return inst->filtered_rpm;
}

float motor_hall_get_rpm_raw(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    if (!inst->is_running) return 0;
    return inst->current_rpm;
}

uint32_t motor_hall_get_pulse_interval_us(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    return inst->last_pulse_interval;
}

uint8_t motor_hall_is_running(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    
    uint64_t current_time_us = Timer6_Timebase_GetTimestamp();
    uint64_t time_since_last_pulse = current_time_us - inst->last_pulse_time_us;
    
    return (time_since_last_pulse <= (STOP_DETECTION_MS * 1000ul));
}

uint8_t motor_hall_is_stalled(motor_hall_handle_t handle)
{
    if (!handle) return 1;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    return inst->stalled;
}

/* ========== 方向相关接口 ========== */

motor_direction_t motor_hall_get_direction(motor_hall_handle_t handle)
{
    if (!handle) return MOTOR_DIRECTION_NONE;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    
    if (inst->current_rpm == 0) return MOTOR_DIRECTION_STOP;
    if (inst->direction_confidence > 50) return inst->current_direction;
    return MOTOR_DIRECTION_NONE;
}

uint8_t motor_hall_get_direction_confidence(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    return inst->direction_confidence;
}

uint8_t motor_hall_is_direction_changed(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    uint8_t changed = inst->direction_changed;
    inst->direction_changed = 0;
    return changed;
}

/* ========== 霍尔计数接口 ========== */

uint32_t motor_hall_get_hall_a_count(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    return inst->hall_pulse_counts[0];
}

uint32_t motor_hall_get_hall_b_count(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    return inst->hall_pulse_counts[1];
}

uint32_t motor_hall_get_total_pulse_count(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    return inst->hall_pulse_counts[0] + inst->hall_pulse_counts[1];
}

void motor_hall_reset_counts(motor_hall_handle_t handle)
{
    if (!handle) return;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    
    __disable_irq();
    inst->hall_pulse_counts[0] = 0;
    inst->hall_pulse_counts[1] = 0;
    inst->pulse_counter = 0;
    inst->hall_a_last_second_count = 0;
    inst->hall_b_last_second_count = 0;
    inst->hall_status = HALL_STATUS_NONE;
    __enable_irq();
}

hall_working_status_t motor_hall_get_status(motor_hall_handle_t handle)
{
    if (!handle) return HALL_STATUS_NONE;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    return inst->hall_status;
}

uint8_t motor_hall_get_active_hall_count(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    return inst->active_hall_count;
}

uint16_t motor_hall_get_pulses_per_rev(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    
    if (inst->config.custom_pulses_per_rev > 0) {
        return inst->config.custom_pulses_per_rev;
    }
    return inst->config.pole_pairs * 2 * inst->active_hall_count;
}

uint8_t motor_hall_get_pole_pairs(motor_hall_handle_t handle)
{
    if (!handle) return 0;
    motor_hall_instance_t* inst = (motor_hall_instance_t*)handle;
    return inst->config.pole_pairs;
}
