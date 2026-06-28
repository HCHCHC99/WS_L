#include "dev_comm_runner.h"
#include "dev_commutation.h"
#include "tmr4_pwm.h"
#include "timer6_timebase.h"
#include "rtt_log.h"
#include "TickTimer.h"
#include <string.h>

/*=============================================================================
 * Hall-to-step lookup tables.
 *
 * Physical Hall sequence (measured on this motor):
 *   CW:  0x02 -> 0x06 -> 0x04 -> 0x05 -> 0x01 -> 0x03
 *   CCW: 0x01 -> 0x05 -> 0x04 -> 0x06 -> 0x02 -> 0x03
 *
 * CW rotation:  field lags rotor by 90° (sector -90° = reverse_map).
 * CCW rotation: field leads rotor by 90° (sector +90° = forward_map).
 *
 * CLOSED_FW uses s_hall2step_cw (FW = CW rotation �? sector -90°).
 * CLOSED_RV uses s_hall2step_ccw (RV = CCW rotation �? sector +90°).
 *=============================================================================*/
static const uint8_t s_hall2step_cw[8]  = {0xFF, 5, 3, 4, 1, 0, 2, 0xFF};  /* CW: sector -90° */
static const uint8_t s_hall2step_ccw[8] = {0xFF, 2, 0, 1, 4, 3, 5, 0xFF};  /* CCW: sector +90° */

/* Calibration-derived tables (computed from g_calib_table on mode 6/7 entry) */
uint8_t g_calib_cw_table[8];   /* g_calib_table + 5 = sector -90°, mode 6 CW (Keil Watch visible) */
uint8_t g_calib_ccw_table[8];  /* g_calib_table + 2 = sector +90°, mode 7 CCW (Keil Watch visible) */

/*=============================================================================
 * State
 *=============================================================================*/
static comm_runner_config_t s_cfg;
static hall_3ch_handle_t    s_hall     = NULL;
static comm_runner_mode_t   s_mode     = COMM_RUNNER_STOP;
static float                s_duty     = 80.0f;
static int                  s_comm_step = 0;
static int                  s_sub_phase = 0;
static uint8_t              s_initialized = 0;
static volatile uint8_t     s_fault_pending = 0;

/* ��定时变量 (mode 1/2 恒� & mode 3/4 飞启共用��) */
static uint64_t s_ol_ramp_start_us  = 0;
static uint64_t s_ol_last_step_us   = 0;
static uint32_t s_ol_interval_us    = 0;
static uint32_t s_ol_start_interval = 0;
static uint32_t s_ol_target_interval = 0;
static uint32_t s_ol_ramp_duration_ms = 0;

/*=============================================================================
 * Calibration globals (volatile �?? Keil Watch accessible)
 *=============================================================================*/
volatile calib_status_t      g_calib_status          = CALIB_IDLE;
volatile calib_error_detail_t g_calib_error          = {0, 0, 0, 0};
volatile uint8_t  g_calib_table[8]                   = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
volatile uint8_t  g_calib_valid_cycles               = 0;
volatile uint8_t  g_calib_total_steps                = 0;
volatile uint8_t  g_calib_cycle_confidence[6]        = {0,0,0,0,0,0};
volatile uint8_t  g_calib_hall_obs[8]                = {0,0,0,0,0,0,0,0};
volatile uint8_t  g_calib_hall_seen[8]               = {0,0,0,0,0,0,0,0};
volatile uint8_t  g_calib_hall_first_step[8]         = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
volatile uint8_t  g_calib_hall_first_cycle[8]        = {0,0,0,0,0,0,0,0};
volatile uint8_t  g_calib_cycle_rejected             = 0;
volatile uint8_t  g_calib_cycle_total                = 0;

/*=============================================================================
 * Calibration internal state (static, not visible in Keil Watch)
 *=============================================================================*/
static int      s_calib_prev_hall;          /* Previous Hall reading for edge detection */
static uint64_t s_calib_start_time;         /* Calibration start timestamp (tickTimer ms) */
static uint64_t s_calib_last_hall_change;   /* Timestamp of last Hall change for stall detect */
static uint8_t  s_calib_invalid_consec;     /* Consecutive invalid Hall readings */
static NonBlockingDelay_t s_calib_settle_delay;  /* 500ms settling delay before sampling */
static uint8_t  s_calib_just_settled;       /* 1 = settling just completed, need one-shot init */
static uint8_t  s_calib_ref_table[8];       /* Reference Hall->step table (first full set) */
static uint8_t  s_calib_ref_valid;          /* 1 if reference table is locked */
static uint8_t  s_calib_pending_map[8];     /* Current pending Hall->step mapping */

#define CALIB_SETTLE_MS        500U         /* Motor settle time before Hall sampling begins */
#define CALIB_STEP_INTERVAL_US   5000UL     /* 5ms/step, matches OPEN_FW default speed */
#define CALIB_TARGET_CYCLES     12U         /* Consistent cycles needed for success */
#define CALIB_TIMEOUT_MS         3000UL    /* 3 second overall timeout (tickTimer ms) */
#define CALIB_MAX_INVALID       10U         /* Consecutive invalid reads before failing */
#define CALIB_STALL_INTERVAL_US 500000UL    /* 500ms without Hall change = stalled */

/*=============================================================================
 * Calibration internal helpers (forward declarations)
 *=============================================================================*/
static void calib_reset(void);
static void calib_finalize(void);
static void calib_fail(calib_status_t reason, uint8_t hall, uint8_t step_a, uint8_t step_b);

/*=============================================================================
 * Hall 回调 (ISR 上下�??)
 *=============================================================================*/
static void runner_on_hall_step(uint8_t step, hall3_direction_t dir)
{
    (void)dir;
    Commutation_Step(step, s_cfg.pwm_freq_hz, s_duty);
}

static void runner_on_hall_fault(uint8_t hall_state)
{
    (void)hall_state;
    /* ISR 上下�??: ��标志, 不执行停�?? (避免 ISR 内大量寄存器操作 + 重入风险) */
    s_fault_pending = 1;
}

/*=============================================================================
 * 内部: �动开�强�?? (mode 1/2 �?? mode 3/4 飞启阶�共�)
 *=============================================================================*/
static void start_open_loop(uint32_t start_interval, uint32_t target_interval,
                            uint32_t ramp_ms, int dir_fw)
{
    s_ol_start_interval   = start_interval;
    s_ol_target_interval  = target_interval;
    s_ol_ramp_duration_ms = ramp_ms;

    s_comm_step = 0;
    s_ol_ramp_start_us = Timer6_Timebase_GetTimestamp();
    s_ol_interval_us   = start_interval;
    s_ol_last_step_us  = s_ol_ramp_start_us;

    Commutation_Init();
    /* ���??: UH_VL */
    COMM_STEP_UH_VL(s_cfg.pwm_freq_hz, s_duty);

    if (dir_fw) {
        MAIN_D("[CommRunner] Open-loop FW start: %lu->%lu us, ramp=%lu ms",
               start_interval, target_interval, ramp_ms);
    } else {
        MAIN_D("[CommRunner] Open-loop RV start: %lu->%lu us, ramp=%lu ms",
               start_interval, target_interval, ramp_ms);
    }
}

/*=============================================================================
 * 内部: ��斜坡计算 + 定时换相 (�?? Update 调用)
 *=============================================================================*/
static void open_loop_tick(uint64_t now, int dir_fw)
{
    uint64_t ramp_elapsed = now - s_ol_ramp_start_us;
    uint64_t ramp_total   = (uint64_t)s_ol_ramp_duration_ms * 1000UL;

    /* 线�斜� */
    if (ramp_elapsed < ramp_total) {
        s_ol_interval_us = s_ol_start_interval
            - (uint32_t)((s_ol_start_interval - s_ol_target_interval)
                         * ramp_elapsed / ramp_total);
    } else {
        s_ol_interval_us = s_ol_target_interval;
    }

    /* 到时间就���?? */
    if ((now - s_ol_last_step_us) >= s_ol_interval_us) {
        s_ol_last_step_us = now;
        if (dir_fw) {
            s_comm_step = (s_comm_step + 1) % 6;
        } else {
            s_comm_step = (s_comm_step + 5) % 6;
        }
        Commutation_Step((uint8_t)s_comm_step, s_cfg.pwm_freq_hz, s_duty);
    }
}

/*=============================================================================
 * 内部: 停� -> 全至关断
 *=============================================================================*/
static void do_stop(void)
{
    int ch;
    s_sub_phase = 0;
    if (s_hall) {
        hall_3ch_stop(s_hall);
    }
    for (ch = 0; ch < 3; ch++) {
        TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch, TMR4_MODE_SYNC, 98.0f);
    }
    MAIN_D("[CommRunner] STOP");
}

/*=============================================================================
 * calib_reset �?? arm calibration open-loop and clear accumulators
 *=============================================================================*/
static void calib_reset(void)
{
    uint8_t i;

    /* Clear globals */
    g_calib_status       = CALIB_RUNNING;
    g_calib_valid_cycles = 0;
    g_calib_total_steps  = 0;
    for (i = 0; i < 8; i++) {
        g_calib_table[i]            = 0xFFu;
        g_calib_hall_obs[i]         = 0;
        g_calib_hall_seen[i]        = 0;
        g_calib_hall_first_step[i]  = 0xFFu;
        g_calib_hall_first_cycle[i] = 0;
        s_calib_pending_map[i]      = 0xFFu;
        s_calib_ref_table[i]        = 0xFFu;
    }
    for (i = 0; i < 6; i++) {
        g_calib_cycle_confidence[i] = 0;
    }
    g_calib_cycle_rejected = 0;
    g_calib_cycle_total    = 0;
    g_calib_error.hall_state = 0;
    g_calib_error.step_a     = 0;
    g_calib_error.step_b     = 0;
    g_calib_error.reserved   = 0;

    /* Clear internals */
    s_calib_ref_valid     = 0;
    s_calib_prev_hall     = -1;
    s_calib_invalid_consec = 0;
    s_calib_just_settled   = 1;
    s_calib_start_time    = tickTimer_GetCount();
    s_calib_last_hall_change = Timer6_Timebase_GetTimestamp();

    /* Stop Hall FSM (ISRs still fire but no callbacks; we detect edges in sw) */
    if (s_hall) hall_3ch_stop(s_hall);

    /* Arm constant-speed open-loop at calibration interval, FW direction */
    start_open_loop(CALIB_STEP_INTERVAL_US, CALIB_STEP_INTERVAL_US, 0, 1);

    /* Start settling delay */
    nbDelay_Init(&s_calib_settle_delay, CALIB_SETTLE_MS);
    nbDelay_Start(&s_calib_settle_delay);

    MAIN_D("[CommRunner] Calibration started, settling=%u ms, interval=%lu us/step",
           (unsigned)CALIB_SETTLE_MS, CALIB_STEP_INTERVAL_US);
}

/*=============================================================================
 * calib_finalize �?? copy reference table to output and stop
 *=============================================================================*/
static void calib_finalize(void)
{
    uint8_t h;
    for (h = 0; h < 8; h++) {
        g_calib_table[h] = s_calib_ref_table[h];
    }
    g_calib_status = CALIB_SUCCESS;
    Commutation_Stop();
    s_mode = COMM_RUNNER_STOP;

    MAIN_D("[CommRunner] Calibration SUCCESS. table=[%d,%d,%d,%d,%d,%d]",
           (int)g_calib_table[1], (int)g_calib_table[2], (int)g_calib_table[3],
           (int)g_calib_table[4], (int)g_calib_table[5], (int)g_calib_table[6]);
}

/*=============================================================================
 * calib_fail �?? stop calibration and populate error detail
 *=============================================================================*/
static void calib_fail(calib_status_t reason, uint8_t hall, uint8_t step_a, uint8_t step_b)
{
    g_calib_status            = reason;
    g_calib_error.hall_state  = hall;
    g_calib_error.step_a      = step_a;
    g_calib_error.step_b      = step_b;
    if (hall <= 7) {
        g_calib_error.reserved = g_calib_hall_obs[hall];
    } else {
        g_calib_error.reserved = 0;
    }
    Commutation_Stop();
    s_mode = COMM_RUNNER_STOP;

    MAIN_D("[CommRunner] Calibration FAILED: status=%d hall=%d step_a=%d step_b=%d",
           (int)reason, (int)hall, (int)step_a, (int)step_b);
}

/*=============================================================================
 * calib_update �?? event-driven: on each Hall edge, record (hall, s_comm_step).
 * The 0-offset mapping is directly observed, no majority voting needed.
 *=============================================================================*/
static void calib_update(uint64_t now)
{
    uint8_t hall;
    int prev_step;

    /* --- Overall timeout --- */
    {
        uint64_t elapsed_ms = tickTimer_GetCount() - s_calib_start_time;
        if (elapsed_ms > CALIB_TIMEOUT_MS) {
            g_calib_error.hall_state = 0;
            for (uint8_t h = 1; h <= 6; h++) {
                if (!g_calib_hall_seen[h]) {
                    g_calib_error.hall_state |= (1u << h);
                }
            }
            g_calib_error.step_a = g_calib_cycle_rejected;
            g_calib_error.step_b = g_calib_cycle_total;
            MAIN_D("[CommRunner] Calib TIMEOUT at %lu ms. rej=%d, seen=0x%02X",
                   (unsigned long)elapsed_ms,
                   (int)g_calib_cycle_rejected,
                   (unsigned)g_calib_error.hall_state);
            calib_fail(CALIB_FAIL_TIMEOUT,
                       g_calib_error.hall_state,
                       g_calib_cycle_rejected,
                       g_calib_cycle_total);
            return;
        }
    }

    /* --- Run open-loop tick --- */
    prev_step = s_comm_step;
    open_loop_tick(now, 1);

    if (s_comm_step != prev_step) {
        g_calib_total_steps++;
    }

    /* --- Settling phase --- */
    if (!nbDelay_IsComplete_noclose(&s_calib_settle_delay)) {
        return;
    }

    if (s_calib_just_settled) {
        s_calib_just_settled = 0;
        s_calib_prev_hall    = -1;
        s_calib_last_hall_change = now;
        MAIN_D("[CommRunner] Calibration settling done, edge detection begins (steps=%d)",
               (int)g_calib_total_steps);
    }

    /* --- Read raw Hall state --- */
    if (s_hall) {
        hall = hall_3ch_read_raw(s_hall);
    } else {
        calib_fail(CALIB_FAIL_STALL, 0, 0, 0);
        return;
    }

    if (hall <= 7) {
        g_calib_hall_obs[hall]++;
    }

    /* --- Hall EDGE detection (state change = rotor sector boundary) --- */
    if ((int)hall != s_calib_prev_hall) {
        s_calib_prev_hall = (int)hall;
        s_calib_last_hall_change = now;

        if (hall >= 1 && hall <= 6) {
            uint8_t step = (uint8_t)s_comm_step;

            MAIN_D("[CAL] edge: hall=0x%02X step=%d (total_steps=%d)",
                   (unsigned)hall, (int)step, (int)g_calib_total_steps);

            /* Record 0-offset mapping: Hall state -> current stator step */
            s_calib_pending_map[hall] = step;

            if (!g_calib_hall_seen[hall]) {
                g_calib_hall_seen[hall]        = 1;
                g_calib_hall_first_step[hall]  = step;
                g_calib_hall_first_cycle[hall] = (uint8_t)(g_calib_cycle_total + 1);
            }

            /* Check if all 6 Hall states seen */
            {
                uint8_t all_seen = 1;
                uint8_t consistent = 1;
                uint8_t h;
                for (h = 1; h <= 6; h++) {
                    if (s_calib_pending_map[h] == 0xFFu) all_seen = 0;
                    if (s_calib_ref_valid && s_calib_pending_map[h] != s_calib_ref_table[h]) consistent = 0;
                }

                if (all_seen) {
                    g_calib_cycle_total++;
                    uint8_t step_used[6] = {0,0,0,0,0,0};
                    uint8_t duplicate = 0;
                    for (h = 1; h <= 6; h++) {
                        uint8_t s = s_calib_pending_map[h];
                        if (s < 6) {
                            if (step_used[s]) duplicate = 1;
                            step_used[s] = 1;
                        }
                    }

                    if (!duplicate && consistent) {
                        g_calib_valid_cycles++;
                        if (!s_calib_ref_valid) {
                            for (h = 1; h <= 6; h++) s_calib_ref_table[h] = s_calib_pending_map[h];
                            s_calib_ref_valid = 1;
                            MAIN_D("[CommRunner] Calib ref locked: [%d,%d,%d,%d,%d,%d]",
                                   (int)s_calib_ref_table[1], (int)s_calib_ref_table[2],
                                   (int)s_calib_ref_table[3], (int)s_calib_ref_table[4],
                                   (int)s_calib_ref_table[5], (int)s_calib_ref_table[6]);
                        }
                        MAIN_D("[CommRunner] Calib cycle %d/%d OK (seen: %d%d%d%d%d%d)",
                               (int)g_calib_valid_cycles, (int)CALIB_TARGET_CYCLES,
                               (int)g_calib_hall_seen[1], (int)g_calib_hall_seen[2],
                               (int)g_calib_hall_seen[3], (int)g_calib_hall_seen[4],
                               (int)g_calib_hall_seen[5], (int)g_calib_hall_seen[6]);
                        if (g_calib_valid_cycles >= CALIB_TARGET_CYCLES) {
                            calib_finalize();
                            return;
                        }
                    } else {
                        g_calib_cycle_rejected++;
                        MAIN_D("[CommRunner] Calib cycle %d REJ (dup=%d con=%d)",
                               (int)g_calib_cycle_total, (int)duplicate, (int)consistent);
                    }

                    /* Reset pending map for next cycle */
                    for (h = 1; h <= 6; h++) s_calib_pending_map[h] = 0xFFu;
                }
            }
        }
    }

    /* --- Invalid Hall detection --- */
    if (hall == 0x00u || hall == 0x07u) {
        s_calib_invalid_consec++;
        if (s_calib_invalid_consec > CALIB_MAX_INVALID) {
            calib_fail(CALIB_FAIL_INVALID, hall, 0, 0);
            return;
        }
    } else {
        s_calib_invalid_consec = 0;
    }

    /* --- Stall detection --- */
    if ((now - s_calib_last_hall_change) > CALIB_STALL_INTERVAL_US
        && g_calib_total_steps > 6) {
        calib_fail(CALIB_FAIL_STALL, 0, 0, 0);
        return;
    }
}

/*=============================================================================
 * CommRunner_Init
 *=============================================================================*/
void CommRunner_Init(const comm_runner_config_t *cfg)
{
    int ch;

    if (!cfg) return;

    memcpy(&s_cfg, cfg, sizeof(comm_runner_config_t));
    s_duty = cfg->default_duty_pct;

    /* ---- PWM ---- */
    tmr4_pwm_config_t pwm_cfg = {
        .output_type_u = TMR4_OUTPUT_SYNC,
        .output_type_v = TMR4_OUTPUT_SYNC,
        .output_type_w = TMR4_OUTPUT_SYNC,
        .freq_hz       = cfg->pwm_freq_hz,
        .dead_time_ns  = 0,
        .active_high   = true,
    };
    TMR4_PWM_Config(&pwm_cfg);
    TMR4_PWM_StartOutput();

    /* ---- Timebase ---- */
    Timer6_Timebase_Init();
    Timer6_Timebase_Start();

    /* ---- 上电默�: 全高�?? ON (上�全�=刹车, 待机安全) ---- */
    for (ch = 0; ch < 3; ch++) {
        TMR4_PWM_SetChannelMode((tmr4_pwm_channel_t)ch, TMR4_MODE_SYNC, 98.0f);
    }

    /* ---- Hall 传感�?? ---- */
    /* 覆写回调�?? Runner 内部函数 */
    s_cfg.hall_cfg.on_step  = runner_on_hall_step;
    s_cfg.hall_cfg.on_fault = runner_on_hall_fault;
    s_hall = hall_3ch_create(&s_cfg.hall_cfg);
    MAIN_D("[CommRunner] Hall sensor created");

    if (cfg->on_init_done) {
        cfg->on_init_done();
    }

    s_mode        = COMM_RUNNER_STOP;
    s_initialized = 1;

    MAIN_D("[CommRunner] Init done, freq=%u Hz", cfg->pwm_freq_hz);
}

/*=============================================================================
 * calib_build_derived_tables �?? build CW/CCW tables from g_calib_table.
 * CW: offset +5 (matches s_hall2step_cw)
 * CCW: offset +2 (matches s_hall2step_ccw)
 *=============================================================================*/
static void calib_build_derived_tables(void)
{
    uint8_t h;
    for (h = 0; h < 8; h++) {
        if (h >= 1 && h <= 6 && g_calib_table[h] <= 5) {
            g_calib_cw_table[h]  = (g_calib_table[h] + 4u) % 6u;
            g_calib_ccw_table[h] = (g_calib_table[h] + 2u) % 6u;
        } else {
            g_calib_cw_table[h]  = 0xFFu;
            g_calib_ccw_table[h] = 0xFFu;
        }
    }
    MAIN_D("[CommRunner] Calib derived: CW=[%d,%d,%d,%d,%d,%d] CCW=[%d,%d,%d,%d,%d,%d]",
           (int)g_calib_cw_table[1], (int)g_calib_cw_table[2],
           (int)g_calib_cw_table[3], (int)g_calib_cw_table[4],
           (int)g_calib_cw_table[5], (int)g_calib_cw_table[6],
           (int)g_calib_ccw_table[1], (int)g_calib_ccw_table[2],
           (int)g_calib_ccw_table[3], (int)g_calib_ccw_table[4],
           (int)g_calib_ccw_table[5], (int)g_calib_ccw_table[6]);
}

/*=============================================================================
 * CommRunner_SetMode
 *=============================================================================*/
void CommRunner_SetMode(comm_runner_mode_t mode)
{
    if (!s_initialized) return;

    /* Calibration / calib-derived modes always allowed to restart */
    if (mode != COMM_RUNNER_CALIB &&
        mode != COMM_RUNNER_CALIB_CW &&
        mode != COMM_RUNNER_CALIB_CCW &&
        mode == s_mode) return;

    s_mode      = mode;

    switch (mode) {

    case COMM_RUNNER_CALIB:
        calib_reset();
        break;

    case COMM_RUNNER_CALIB_CW:
        if (s_hall) hall_3ch_stop(s_hall);
        calib_build_derived_tables();
        start_open_loop(s_cfg.ol_fly_start_us, s_cfg.ol_fly_target_us,
                        s_cfg.ol_fly_ramp_ms, 1);
        s_sub_phase = 0;
        MAIN_D("[CommRunner] Mode=CALIB_CW: 500ms ramp -> closed-loop CW");
        break;

    case COMM_RUNNER_CALIB_CCW:
        if (s_hall) hall_3ch_stop(s_hall);
        calib_build_derived_tables();
        start_open_loop(s_cfg.ol_fly_start_us, s_cfg.ol_fly_target_us,
                        s_cfg.ol_fly_ramp_ms, 0);
        s_sub_phase = 0;
        MAIN_D("[CommRunner] Mode=CALIB_CCW: 500ms ramp -> closed-loop CCW");
        break;

    case COMM_RUNNER_STOP:
        do_stop();
        break;

    case COMM_RUNNER_OPEN_FW:
        if (s_hall) hall_3ch_stop(s_hall);
        start_open_loop(s_cfg.ol_const_start_us, s_cfg.ol_const_target_us,
                        s_cfg.ol_const_ramp_ms, 1);
        s_sub_phase = 0;
        break;

    case COMM_RUNNER_OPEN_RV:
        if (s_hall) hall_3ch_stop(s_hall);
        start_open_loop(s_cfg.ol_const_start_us, s_cfg.ol_const_target_us,
                        s_cfg.ol_const_ramp_ms, 0);
        s_sub_phase = 0;
        break;

    case COMM_RUNNER_CLOSED_FW:
        if (s_hall) hall_3ch_stop(s_hall);
        start_open_loop(s_cfg.ol_fly_start_us, s_cfg.ol_fly_target_us,
                        s_cfg.ol_fly_ramp_ms, 1);
        s_sub_phase = 0;
        MAIN_D("[CommRunner] Mode=CLOSED_FW: Open-loop ramp -> flying start");
        break;

    case COMM_RUNNER_CLOSED_RV:
        if (s_hall) hall_3ch_stop(s_hall);
        start_open_loop(s_cfg.ol_fly_start_us, s_cfg.ol_fly_target_us,
                        s_cfg.ol_fly_ramp_ms, 0);
        s_sub_phase = 0;
        MAIN_D("[CommRunner] Mode=CLOSED_RV: Open-loop ramp -> flying start");
        break;
    }
}

/*=============================================================================
 * CommRunner_GetMode
 *=============================================================================*/
comm_runner_mode_t CommRunner_GetMode(void)
{
    return s_mode;
}

/*=============================================================================
 * CommRunner_SetDuty
 *=============================================================================*/
void CommRunner_SetDuty(float duty_pct)
{
    if (duty_pct < 2.0f) duty_pct = 2.0f;
    if (duty_pct > 98.0f) duty_pct = 98.0f;
    s_duty = duty_pct;

    /* ��模式�??, 下� Hall ISR 触发 on_step 时自然带新占空比 */
    /* ��模式�??, 下�定时换相时带新占空� */
}

/*=============================================================================
 * CommRunner_GetDuty
 *=============================================================================*/
float CommRunner_GetDuty(void)
{
    return s_duty;
}

/*=============================================================================
 * CommRunner_Update - 主循�?? 1ms 调用
 *=============================================================================*/
/*=============================================================================
 * CommRunner_Update - ��ѭ�� 1ms ����
 *=============================================================================*/
void CommRunner_Update(void)
{
    if (!s_initialized) return;

    /* ---- Calibration mode: fully self-contained path ---- */
    if (s_mode == COMM_RUNNER_CALIB) {
        if (s_fault_pending) {
            s_fault_pending = 0;
            MAIN_D("[CommRunner] Hall fault during calibration, aborting");
            calib_fail(CALIB_FAIL_INVALID, 0, 0, 0);
            return;
        }
        Timer6_Timebase_UpdateTimestamp();
        calib_update(Timer6_Timebase_GetTimestamp());
        return;
    }

    /* ���� ISR �����? Hall ���� (000/111) */
    if (s_fault_pending) {
        s_fault_pending = 0;
        MAIN_D("[CommRunner] Hall fault, coast");
        CommRunner_SetMode(COMM_RUNNER_STOP);
    }

    Timer6_Timebase_UpdateTimestamp();
    uint64_t now = Timer6_Timebase_GetTimestamp();

    switch (s_mode) {

    /* ---- �������� (mode 1/2) ---- */
    case COMM_RUNNER_OPEN_FW:
        open_loop_tick(now, 1);
        break;

    case COMM_RUNNER_OPEN_RV:
        open_loop_tick(now, 0);
        break;

    /* ---- �ɳ�->�ջ� (mode 3/4) ---- */
    case COMM_RUNNER_CLOSED_FW:
    case COMM_RUNNER_CLOSED_RV: {
        int is_fw = (s_mode == COMM_RUNNER_CLOSED_FW);

        if (s_sub_phase == 0) {
            /* === �׶�0: ����ǿ�� + б�� === */
            open_loop_tick(now, is_fw);

            /* б�½��� -> �ɳ�����ջ�? */
            uint64_t ramp_elapsed = now - s_ol_ramp_start_us;
            uint64_t ramp_total   = (uint64_t)s_ol_ramp_duration_ms * 1000UL;
            if (ramp_elapsed >= ramp_total) {
                if (is_fw) {
                    /* FW = CW rotation �? sector -90° �? s_hall2step_cw */
                    hall_3ch_set_table(s_hall, s_hall2step_cw);
                    hall_3ch_start_flying(s_hall, HALL3_DIR_FORWARD);
                } else {
                    /* RV = CCW rotation �? sector +90° �? s_hall2step_ccw */
                    hall_3ch_set_table(s_hall, s_hall2step_ccw);
                    hall_3ch_start_flying(s_hall, HALL3_DIR_REVERSE);
                }
                s_sub_phase = 1;
                MAIN_D("[CommRunner] Flying start -> closed-loop (%s)",
                       is_fw ? "FW" : "RV");
            }
        } else {
            /* === �׶�1: �ջ����� (Hall ISR ��������) === */
            hall_3ch_update(s_hall);
            if (hall_3ch_is_stalled(s_hall)) {
                MAIN_D("[CommRunner] Closed-loop stall, coast");
                CommRunner_SetMode(COMM_RUNNER_STOP);
            }
        }
        break;
    }

    /* ---- Calib-derived closed-loop (mode 6/7) ---- */
    case COMM_RUNNER_CALIB_CW:
    case COMM_RUNNER_CALIB_CCW: {
        int is_fw = (s_mode == COMM_RUNNER_CALIB_CW);

        if (s_sub_phase == 0) {
            /* Phase 0: open-loop ramp */
            open_loop_tick(now, is_fw);

            uint64_t ramp_elapsed = now - s_ol_ramp_start_us;
            uint64_t ramp_total   = (uint64_t)s_ol_ramp_duration_ms * 1000UL;
            if (ramp_elapsed >= ramp_total) {
                if (is_fw) {
                    hall_3ch_set_table(s_hall, g_calib_cw_table);
                    hall_3ch_start_flying(s_hall, HALL3_DIR_FORWARD);
                } else {
                    hall_3ch_set_table(s_hall, g_calib_ccw_table);
                    hall_3ch_start_flying(s_hall, HALL3_DIR_REVERSE);
                }
                s_sub_phase = 1;
                MAIN_D("[CommRunner] Calib fly-start -> closed-loop (%s)",
                       is_fw ? "CW" : "CCW");
            }
        } else {
            /* Phase 1: closed-loop (Hall ISR driven) */
            hall_3ch_update(s_hall);
            if (hall_3ch_is_stalled(s_hall)) {
                MAIN_D("[CommRunner] Calib closed-loop stall, coast");
                CommRunner_SetMode(COMM_RUNNER_STOP);
            }
        }
        break;
    }

    case COMM_RUNNER_STOP:
    default:
        break;
    }
}

/*=============================================================================
 * CommRunner_GetRPM
 *=============================================================================*/
float CommRunner_GetRPM(void)
{
    if (!s_hall) return 0.0f;
    return hall_3ch_get_rpm(s_hall);
}

/*=============================================================================
 * CommRunner_IsRunning
 *=============================================================================*/
uint8_t CommRunner_IsRunning(void)
{
    if (!s_hall) return 0;
    return hall_3ch_is_running(s_hall);
}

/*=============================================================================
 * CommRunner_IsStalled
 *=============================================================================*/
uint8_t CommRunner_IsStalled(void)
{
    if (!s_hall) return 0;
    return hall_3ch_is_stalled(s_hall);
}

/*=============================================================================
 * CommRunner_GetCalibStatus
 *=============================================================================*/
calib_status_t CommRunner_GetCalibStatus(void)
{
    return g_calib_status;
}

/*=============================================================================
 * CommRunner_GetCalibTable
 *=============================================================================*/
const uint8_t* CommRunner_GetCalibTable(void)
{
    return (const uint8_t*)g_calib_table;
}

/*=============================================================================
 * CommRunner_CalibAbort
 *=============================================================================*/
void CommRunner_CalibAbort(void)
{
    if (g_calib_status == CALIB_RUNNING) {
        calib_fail(CALIB_FAIL_TIMEOUT, 0, 0, 0);
    }
}
