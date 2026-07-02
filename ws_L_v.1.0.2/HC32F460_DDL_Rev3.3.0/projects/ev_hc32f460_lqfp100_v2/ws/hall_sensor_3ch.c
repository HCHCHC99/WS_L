#include "hall_sensor_3ch.h"
#include "dev_commutation.h"
#include "timer6_timebase.h"
#include "TickTimer.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_interrupts.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>

/* ========== Constants ========== */
#define MAX_INSTANCES             2
#define MIN_PULSE_INTERVAL_US     50u
#define RPM_WINDOW_SIZE           6
#define RPM_UPDATE_MIN_US      20000u
#define RPM_UPDATE_MIN_PULSES  6u
#define RPM_TIMEOUT_US         500000u

/* Hall state (0x00-0x07) -> rotor electrical angle (degrees). -1 = invalid. */
static const int16_t s_hall_to_angle[8] = {
    -1,    /* 0x00: 000 invalid */
     0,    /* 0x01: 001 */
   240,    /* 0x02: 010 */
   300,    /* 0x03: 011 */
   120,    /* 0x04: 100 */
    60,    /* 0x05: 101 */
   180,    /* 0x06: 110 */
    -1,    /* 0x07: 111 invalid */
};

/* ========== States ========== */
enum {
    STATE_IDLE = 0,
    STATE_ALIGNING,
    STATE_RUNNING,
    STATE_FAULT,
};

/* ========== Instance structure ========== */
typedef struct hall_3ch_instance_t {
    uint8_t id;
    uint8_t valid;
    uint8_t state;

    hall_3ch_config_t config;

    uint8_t  gpio_port[3];
    uint16_t gpio_pin[3];

    volatile uint8_t  last_hall_state;
    volatile uint8_t  last_step;
    volatile uint32_t last_pulse_interval;
    volatile uint64_t last_pulse_time_us;
    volatile uint32_t pulse_counter;
    volatile uint32_t last_counter;     /* per-instance, not shared across ISR channels */

    /* RPM: M-method (pulse count over time window) */
    volatile float    current_rpm;
    volatile float    filtered_rpm;
    float             rpm_window[RPM_WINDOW_SIZE];
    uint8_t           rpm_write_idx;
    uint8_t           rpm_valid_count;
    uint32_t          last_pulse_count;
    uint64_t          last_rpm_update_us;

    /* Alignment */
    hall3_direction_t  target_dir;
    uint64_t           align_start_time_us;

    /* Display step: always ±1 per transition for clean J-Scope visualization */
    uint8_t            display_step;

    /* Stall */
    volatile uint8_t  stalled;

} hall_3ch_instance_t;

/* ========== Global instances ========== */
static uint8_t g_system_initialized = 0;
static hall_3ch_instance_t g_instances[MAX_INSTANCES] = {{{0}}};
static hall_3ch_instance_t *g_irq_map[3] = {NULL, NULL, NULL};

/* Keil Watch debug globals */
volatile float    g_hall_rpm        = 0.0f;
volatile uint8_t  g_hall_state      = 0;
volatile uint8_t  g_hall_dir        = 0;
volatile uint8_t  g_hall_running    = 0;
volatile uint8_t  g_hall_stalled    = 0;
volatile uint8_t  g_hall_last_step  = 0;

/* J-Scope HSS ���μ�� */
volatile uint8_t  g_scope_ha     = 0;
volatile uint8_t  g_scope_hb     = 0;
volatile uint8_t  g_scope_hc     = 0;
volatile uint8_t  g_scope_step   = 0;
volatile int16_t  g_scope_rpm    = 0;

/* ISR debug counters */
static volatile uint32_t g_dbg_isr_fire      = 0;
static volatile uint32_t g_dbg_isr_unchanged = 0;
static volatile uint32_t g_dbg_isr_noise     = 0;
static volatile uint32_t g_dbg_isr_fault     = 0;
static volatile uint32_t g_dbg_isr_baddiff   = 0;
static volatile uint32_t g_dbg_isr_valid     = 0;
static volatile uint32_t g_dbg_isr_time_us   = 0;
static volatile uint32_t g_dbg_isr_time_max  = 0;
static volatile uint32_t g_dbg_fire_ch0      = 0;
static volatile uint32_t g_dbg_fire_ch1      = 0;
static volatile uint32_t g_dbg_fire_ch2      = 0;

/* ========== Forward declarations ========== */
static void hall_common_handler(uint8_t ch);

/* ========== System init ========== */
void hall_3ch_system_init(void)
{
    if (g_system_initialized) return;
    Timer6_Timebase_Init();
    Timer6_Timebase_Start();
    memset(g_instances, 0, sizeof(g_instances));
    g_system_initialized = 1;
}

/* ========== Register one Hall channel ========== */
static void register_hall_irq(hall_3ch_instance_t *inst, uint8_t ch, func_ptr_t cb)
{
    stc_extint_init_t       stcExti;
    stc_irq_signin_config_t stcIrq;
    stc_gpio_init_t         stcGpio;

    uint32_t  eirq_ch = inst->config.eirq_ch[ch];
    IRQn_Type irqn    = inst->config.irqn[ch];
    uint32_t  irq_src = inst->config.irq_src[ch];
    uint8_t   port    = inst->config.port[ch];
    uint16_t  pin     = inst->config.pin[ch];

    g_irq_map[ch] = inst;

    memset(&stcExti, 0, sizeof(stcExti));
    stcExti.u32Edge        = EXTINT_TRIG_BOTH;
    stcExti.u32Filter      = EXTINT_FILTER_ON;
    stcExti.u32FilterClock = EXTINT_FCLK_DIV64;
    EXTINT_Init(eirq_ch, &stcExti);

    GPIO_StructInit(&stcGpio);
    stcGpio.u16PinDir  = PIN_DIR_IN;
    stcGpio.u16PinAttr = PIN_ATTR_DIGITAL;
    stcGpio.u16PullUp  = PIN_PU_ON;
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    GPIO_Init(port, pin, &stcGpio);
    GPIO_ExtIntCmd(port, pin, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    memset(&stcIrq, 0, sizeof(stcIrq));
    stcIrq.enIntSrc    = (en_int_src_t)irq_src;
    stcIrq.enIRQn      = irqn;
    stcIrq.pfnCallback = cb;
    INTC_IrqSignIn(&stcIrq);

    EXTINT_ClearExtIntStatus(eirq_ch);
    NVIC_ClearPendingIRQ(irqn);
    NVIC_SetPriority(irqn, inst->config.irq_priority);
    NVIC_EnableIRQ(irqn);
}

/* ========== ISR entry points ========== */
static void hall_u_isr(void) { hall_common_handler(0); }
static void hall_v_isr(void) { hall_common_handler(1); }
static void hall_w_isr(void) { hall_common_handler(2); }

/* ========== Read all 3 Hall GPIOs as 3-bit state ========== */
static uint8_t read_hall_state_raw(const hall_3ch_instance_t *inst)
{
    uint8_t s = 0;
    s |= (GPIO_ReadInputPins(inst->gpio_port[0], inst->gpio_pin[0]) == PIN_SET) ? 0x04u : 0x00u;
    s |= (GPIO_ReadInputPins(inst->gpio_port[1], inst->gpio_pin[1]) == PIN_SET) ? 0x02u : 0x00u;
    s |= (GPIO_ReadInputPins(inst->gpio_port[2], inst->gpio_pin[2]) == PIN_SET) ? 0x01u : 0x00u;
    return s;
}

/* ========== Core ISR: minimal, like reference implementation ========== */
/* ========== Core ISR: �Ƴ����д�ӡ��ȷ�����µ�ִ��Ч�� ========== */
static void hall_common_handler(uint8_t ch)
{
    hall_3ch_instance_t *inst = g_irq_map[ch];
    if (!inst || !inst->valid) return;

    EXTINT_ClearExtIntStatus(inst->config.eirq_ch[ch]);

    g_dbg_isr_fire++;
    if (ch == 0) g_dbg_fire_ch0++;
    else if (ch == 1) g_dbg_fire_ch1++;
    else g_dbg_fire_ch2++;

    /* Read Hall state and update J-Scope immediately */
    uint8_t state = read_hall_state_raw(inst);
    g_scope_ha = (state >> 2) & 1;
    g_scope_hb = (state >> 1) & 1;
    g_scope_hc = state & 1;

    /* Look up commutation step — as early as possible */
    uint8_t step = inst->config.hall_to_step[state];
    if (step == 0xFFu) {
        g_dbg_isr_fault++;
        if (inst->state == STATE_RUNNING) {
            inst->state = STATE_FAULT;
            if (inst->config.on_fault) {
                inst->config.on_fault(state);
            }
        }
        return;
    }

    /* J-Scope step updated immediately after table lookup */
    g_scope_step = step;

    /* Raw delta for pulse tracking (inline, no division, per-instance) */
    uint32_t counter = Timer6_Timebase_GetCounter();
    uint32_t delta  = 0;
    if (counter >= inst->last_counter) {
        delta = counter - inst->last_counter;
    } else {
        delta = (65536u - inst->last_counter) + counter;
    }
    inst->last_counter = counter;

    /* During alignment/idle: record state */
    if (inst->state != STATE_RUNNING) {
        inst->last_hall_state  = state;
        inst->last_step        = step;
        inst->last_pulse_time_us = Timer6_Timebase_GetTimestamp();
        inst->pulse_counter++;
        return;
    }

    /* Step adjacency check — diagnostic only.
     * Hall sensor is the source of truth — always accept its reading. */
    {
        uint8_t old_step = inst->last_step;
        int8_t diff = (int8_t)step - (int8_t)old_step;
        if (diff < 0) diff += 6;
        if (diff != 1 && diff != 5) {
            g_dbg_isr_baddiff++;
        }
    }

    /* Update tracking state */
    g_dbg_isr_valid++;
    inst->last_hall_state        = state;
    inst->last_step              = step;
    inst->last_pulse_interval = delta;
    inst->last_pulse_time_us     = Timer6_Timebase_GetTimestamp();
    inst->pulse_counter++;

    /* Normalized display step: always ±1 per transition (internal use) */
    if (inst->target_dir == HALL3_DIR_FORWARD) {
        inst->display_step = (inst->display_step + 1u) % 6u;
    } else {
        inst->display_step = (inst->display_step + 5u) % 6u;
    }

    /* Commutation callback (Keep this lean!) */
    if (inst->config.on_step) {
        uint64_t t0 = Timer6_Timebase_GetTimestamp();
        inst->config.on_step(step, inst->target_dir);
        uint64_t t1 = Timer6_Timebase_GetTimestamp();

        uint32_t dt = (t1 > t0) ? (uint32_t)(t1 - t0) : 0;
        g_dbg_isr_time_us = dt;
        if (dt > g_dbg_isr_time_max) {
            g_dbg_isr_time_max = dt;
        }
    }
}

/* ========== RPM sliding window filter ========== */
static void update_rpm_filter(hall_3ch_instance_t *inst, float raw)
{
    inst->rpm_window[inst->rpm_write_idx] = raw;
    inst->rpm_write_idx = (inst->rpm_write_idx + 1) % RPM_WINDOW_SIZE;
    if (inst->rpm_valid_count < RPM_WINDOW_SIZE) {
        inst->rpm_valid_count++;
    }
    float sum = 0.0f;
    for (uint8_t i = 0; i < inst->rpm_valid_count; i++) {
        sum += inst->rpm_window[i];
    }
    inst->filtered_rpm = sum / (float)inst->rpm_valid_count;
}

/* ========== Create instance ========== */
hall_3ch_handle_t hall_3ch_create(const hall_3ch_config_t *cfg)
{
    if (!g_system_initialized) hall_3ch_system_init();
    if (!cfg) return NULL;

    uint8_t i;
    for (i = 0; i < MAX_INSTANCES; i++) {
        if (!g_instances[i].valid) break;
    }
    if (i >= MAX_INSTANCES) return NULL;

    hall_3ch_instance_t *inst = &g_instances[i];
    memset(inst, 0, sizeof(*inst));

    inst->id    = i;
    inst->valid = 1;
    inst->state = STATE_IDLE;

    memcpy(&inst->config, cfg, sizeof(hall_3ch_config_t));

    for (uint8_t ch = 0; ch < 3; ch++) {
        inst->gpio_port[ch] = cfg->port[ch];
        inst->gpio_pin[ch]  = cfg->pin[ch];
    }

    inst->last_hall_state = 0xFFu;

    register_hall_irq(inst, 0, (func_ptr_t)hall_u_isr);
    register_hall_irq(inst, 1, (func_ptr_t)hall_v_isr);
    register_hall_irq(inst, 2, (func_ptr_t)hall_w_isr);

    MAIN_D("[HALL3] Created instance %d", i);
    return (hall_3ch_handle_t)inst;
}

void hall_3ch_destroy(hall_3ch_handle_t h)
{
    if (!h) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    for (uint8_t ch = 0; ch < 3; ch++) {
        NVIC_DisableIRQ(inst->config.irqn[ch]);
    }
    inst->valid = 0;
}

void hall_3ch_start(hall_3ch_handle_t h, hall3_direction_t dir)
{
    if (!h) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    if (!inst->valid) return;

    inst->target_dir  = dir;
    inst->stalled     = 0;
    inst->last_pulse_time_us = Timer6_Timebase_GetTimestamp();

    inst->last_pulse_count   = inst->pulse_counter;
    inst->last_rpm_update_us = inst->last_pulse_time_us;

    inst->state = STATE_ALIGNING;
    inst->align_start_time_us = inst->last_pulse_time_us;
    inst->display_step = inst->config.align_step;

    if (inst->config.on_step) {
        inst->config.on_step(inst->config.align_step, dir);
    }
}

void hall_3ch_start_flying(hall_3ch_handle_t h, hall3_direction_t dir)
{
    if (!h) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    if (!inst->valid) return;

    inst->target_dir  = dir;
    inst->stalled     = 0;
    inst->last_pulse_time_us = Timer6_Timebase_GetTimestamp();

    inst->last_pulse_count   = inst->pulse_counter;
    inst->last_rpm_update_us = inst->last_pulse_time_us;

    /* Read current Hall position.
     * Sample twice; if they differ the rotor is moving — use the latest reading.
     * If still invalid, fall back to step 0 (safe default, first ISR will correct it). */
    uint8_t hall_state = read_hall_state_raw(inst);
    for (volatile int32_t _d = 0; _d < 100; _d++) { __NOP(); }
    uint8_t hall_state2 = read_hall_state_raw(inst);

    if (hall_state != hall_state2) {
        /* Rotor is moving between samples — use the latest reading */
        hall_state = hall_state2;
    }

    uint8_t step = inst->config.hall_to_step[hall_state];
    if (step == 0xFFu) {
        step = 0;  /* Invalid Hall code → safe default, first ISR corrects it */
    }

    inst->state           = STATE_RUNNING;
    inst->last_step       = step;
    inst->last_hall_state = hall_state;
    inst->display_step    = step;

    (void)Timer6_Timebase_GetDelta();

    if (inst->config.on_step) {
        inst->config.on_step(step, dir);
    }

    MAIN_D("[HALL3] Flying start: hall=0x%02X step=%d dir=%d",
           hall_state, step, (int)dir);
}

void hall_3ch_stop(hall_3ch_handle_t h)
{
    if (!h) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    inst->state    = STATE_IDLE;
    inst->stalled  = 0;
    inst->last_hall_state = 0xFFu;

    inst->current_rpm        = 0.0f;
    inst->filtered_rpm       = 0.0f;
    inst->rpm_valid_count    = 0;
    inst->last_pulse_count   = 0;
    inst->last_rpm_update_us = 0;
}

void hall_3ch_set_table(hall_3ch_handle_t h, const uint8_t table[8])
{
    if (!h || !table) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    uint8_t i;
    for (i = 0; i < 8; i++) {
        inst->config.hall_to_step[i] = table[i];
    }
    MAIN_D("[HALL3] set_table: [%d,%d,%d,%d,%d,%d,%d,%d]",
           (int)table[0], (int)table[1], (int)table[2], (int)table[3],
           (int)table[4], (int)table[5], (int)table[6], (int)table[7]);
}

/* ========== Read raw Hall state direct from GPIO ========== */
uint8_t hall_3ch_read_raw(hall_3ch_handle_t h)
{
    if (!h) return 0xFFu;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    return read_hall_state_raw(inst);
}

/* ========== Periodic update (called from main loop) ========== */
void hall_3ch_update(hall_3ch_handle_t h)
{
    if (!h) return;
    hall_3ch_instance_t *inst = (hall_3ch_instance_t *)h;
    if (!inst->valid) return;

    Timer6_Timebase_UpdateTimestamp();
    uint64_t now = Timer6_Timebase_GetTimestamp();

    /* M-method RPM: pulse count over time window (all states) */
    if (inst->state == STATE_RUNNING || inst->state == STATE_IDLE) {
        uint32_t pulse_delta = inst->pulse_counter - inst->last_pulse_count;
        uint64_t elapsed     = now - inst->last_rpm_update_us;

        if (elapsed >= RPM_UPDATE_MIN_US && pulse_delta >= RPM_UPDATE_MIN_PULSES) {
            float rpm = (float)pulse_delta * 60000000.0f
                      / ((float)elapsed * (float)(inst->config.pole_pairs * 6u));
            if (rpm > 100000.0f) rpm = 100000.0f;
            inst->current_rpm = rpm;
            update_rpm_filter(inst, rpm);
            inst->last_pulse_count   = inst->pulse_counter;
            inst->last_rpm_update_us = now;
        } else if (elapsed > RPM_TIMEOUT_US) {
            if (pulse_delta == 0) {
                inst->current_rpm = 0.0f;
                update_rpm_filter(inst, 0.0f);
            }
            inst->last_pulse_count   = inst->pulse_counter;
            inst->last_rpm_update_us = now;
        }
    }

    switch (inst->state) {

    case STATE_ALIGNING: {
        uint64_t elapsed = now - inst->align_start_time_us;
        if (elapsed >= (uint64_t)inst->config.align_duration_ms * 1000UL) {
            inst->state = STATE_RUNNING;
            inst->last_step = inst->config.align_step;

            inst->last_pulse_count   = inst->pulse_counter;
            inst->last_rpm_update_us = now;

            (void)Timer6_Timebase_GetDelta();

            uint8_t kick_step;
            if (inst->target_dir == HALL3_DIR_FORWARD) {
                kick_step = (inst->config.align_step + 1) % 6;
            } else {
                kick_step = (inst->config.align_step + 5) % 6;
            }

            inst->display_step = kick_step;

            if (inst->config.on_step) {
                inst->config.on_step(kick_step, inst->target_dir);
            }
        }
        break;
    }

    case STATE_RUNNING:
        if (inst->config.stall_timeout_ms > 0) {
            uint64_t since_pulse = now - inst->last_pulse_time_us;
            if (since_pulse > (uint64_t)inst->config.stall_timeout_ms * 1000UL) {
                inst->stalled = 1;
                inst->state   = STATE_IDLE;
            }
        }
        break;

    case STATE_FAULT:
    case STATE_IDLE:
    default:
        break;
    }

#if 0
    /* RPM debug (all states) */
    {
        static uint32_t rpm_dbg_cnt = 0;
        rpm_dbg_cnt++;
        if ((rpm_dbg_cnt & 0x1FFu) == 0) {
            const char *dir_str = (inst->target_dir == HALL3_DIR_FORWARD) ? "CW" :
                                   (inst->target_dir == HALL3_DIR_REVERSE) ? "CCW" : "-";
            uint8_t hs = inst->last_hall_state;
            MAIN_D("[HALL] RPM raw=%d filt=%d A%d B%d C%d - %ddeg %s %s-%s(%ddeg) | fire=%lu/%lu/%lu/%lu unch=%lu noise=%lu fault=%lu baddiff=%lu valid=%lu dtMax=%lu",
                   (int)inst->current_rpm, (int)inst->filtered_rpm,
                   (hs >> 2) & 1, (hs >> 1) & 1, hs & 1,
                   (int)s_hall_to_angle[(hs <= 7) ? hs : 7],
                   dir_str,
                   Commutation_GetHighPhase(inst->last_step),
                   Commutation_GetLowPhase(inst->last_step),
                   (int)Commutation_GetFieldAngle(inst->last_step),
                   (unsigned long)g_dbg_isr_fire,
                   (unsigned long)g_dbg_fire_ch0,
                   (unsigned long)g_dbg_fire_ch1,
                   (unsigned long)g_dbg_fire_ch2,
                   g_dbg_isr_unchanged, g_dbg_isr_noise,
                   g_dbg_isr_fault, g_dbg_isr_baddiff, g_dbg_isr_valid,
                   (unsigned long)g_dbg_isr_time_max);
        }
    }
#endif

    g_hall_rpm       = inst->filtered_rpm;
    g_scope_rpm      = (int16_t)inst->filtered_rpm;
    g_hall_state     = inst->state;
    g_hall_dir       = (uint8_t)inst->target_dir;
    g_hall_running   = (inst->state == STATE_RUNNING) ? 1 : 0;
    g_hall_stalled   = inst->stalled;
    g_hall_last_step = inst->last_step;
}

/* ========== Query API ========== */

float hall_3ch_get_rpm(hall_3ch_handle_t h)
{
    if (!h) return 0.0f;
    return ((hall_3ch_instance_t *)h)->filtered_rpm;
}

hall3_direction_t hall_3ch_get_direction(hall_3ch_handle_t h)
{
    if (!h) return HALL3_DIR_NONE;
    return ((hall_3ch_instance_t *)h)->target_dir;
}

uint8_t hall_3ch_is_running(hall_3ch_handle_t h)
{
    if (!h) return 0;
    return (((hall_3ch_instance_t *)h)->state == STATE_RUNNING) ? 1 : 0;
}

uint8_t hall_3ch_is_stalled(hall_3ch_handle_t h)
{
    if (!h) return 1;
    return ((hall_3ch_instance_t *)h)->stalled;
}
