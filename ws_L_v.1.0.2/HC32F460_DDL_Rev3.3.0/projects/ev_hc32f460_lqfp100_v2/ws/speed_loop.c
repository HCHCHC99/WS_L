#include "speed_loop.h"
#include "motor_config.h"

/* Keil Watch: current limit for cascade (mA) */
volatile float g_i_ref_max_ma = 1500.0f;

/* Default gains = legacy single-loop speed values; for cascade raise Kp/Ki */
pid_config_t g_spd_pid_cfg = {
    .enabled      = true,
    .p_valid      = true,
    .i_valid      = true,
    .d_valid      = false,
    .kp           = 0.02f,
    .ki           = 0.005f,
    .kd           = 0.0f,
    .output_min   = 2.0f,    /* adjusted by topology at init */
    .output_max   = 98.0f,
    .integral_max = 200.0f,
    .i_term_max   = 0.0f,
    .update_ms    = 50,
};

volatile float g_scope_spd_ref = 0.0f;
volatile float g_scope_spd_rpm = 0.0f;
volatile float g_scope_spd_err = 0.0f;
volatile float g_scope_spd_out = 0.0f;

static pid_state_t s_spd_pid;
static float s_target_rpm = 0.0f;
static float s_output     = 0.0f;
static uint8_t s_inited   = 0;

void SpeedLoop_Init(void)
{
    if (s_inited) return;

#if MOTOR_LOOP_CURRENT_ENABLE
    /* cascade: output = current reference (mA), 0 .. limit */
    g_spd_pid_cfg.output_min = 0.0f;
    g_spd_pid_cfg.output_max = g_i_ref_max_ma;
#else
    /* speed-only: output = duty (%) */
    g_spd_pid_cfg.output_min = 2.0f;
    g_spd_pid_cfg.output_max = 98.0f;
#endif

    PID_Init(&s_spd_pid, &g_spd_pid_cfg);
    s_inited = 1;
}

void SpeedLoop_SetTarget(float target_rpm) { s_target_rpm = target_rpm; }

float SpeedLoop_Update(float measured_rpm)
{
    if (!s_inited) SpeedLoop_Init();
#if MOTOR_LOOP_CURRENT_ENABLE
    g_spd_pid_cfg.output_max = g_i_ref_max_ma;   /* keep limit live for Keil Watch */
#endif
    g_scope_spd_rpm = measured_rpm;
    s_output = PID_Update(&s_spd_pid, s_target_rpm, measured_rpm);
    g_scope_spd_ref = s_target_rpm;
    g_scope_spd_err = s_target_rpm - measured_rpm;
    g_scope_spd_out = s_output;
    return s_output;
}

float SpeedLoop_GetOutput(void) { return s_output; }

void SpeedLoop_Seed(float output, float measured_rpm)
{
    if (!s_inited) SpeedLoop_Init();
    PID_Seed(&s_spd_pid, s_target_rpm, measured_rpm, output);
    s_output = output;
    g_scope_spd_out = output;
}
