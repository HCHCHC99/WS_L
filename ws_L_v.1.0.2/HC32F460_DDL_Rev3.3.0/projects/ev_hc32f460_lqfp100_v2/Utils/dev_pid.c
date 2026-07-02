/**
 *******************************************************************************
 * @file  Utils/dev_pid.c
 * @brief Generic PID controller implementation.
 *
 *        Parallel-form PID with anti-windup and derivative-on-measurement.
 *        All parameters read from volatile pid_config_t each call — Keil Watch
 *        changes take effect on the next PID_Update, zero polling required.
 *******************************************************************************
 */

#include "dev_pid.h"
#include "TickTimer.h"
#include <string.h>

/*=============================================================================
 * PID_Init
 *=============================================================================*/
void PID_Init(pid_state_t *pid, pid_config_t *cfg)
{
    if (!pid) return;
    memset(pid, 0, sizeof(*pid));
    pid->cfg          = cfg;
    pid->first_sample = true;
    pid->last_output  = (cfg && cfg->enabled) ? cfg->output_min : 0.0f;
}

/*=============================================================================
 * PID_Reset
 *=============================================================================*/
void PID_Reset(pid_state_t *pid)
{
    if (!pid) return;
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_sample     = true;
    pid->last_update_ms   = 0;
    if (pid->cfg && pid->cfg->enabled) {
        pid->last_output  = pid->cfg->output_min;
    } else {
        pid->last_output  = 0.0f;
    }
}

/*=============================================================================
 * PID_Update — parallel-form with anti-windup, derivative-on-measurement.
 *
 * Reads live parameters from pid->cfg each call. Snapshot volatile fields
 * once at the top so one computation uses a consistent set of parameters.
 *=============================================================================*/
float PID_Update(pid_state_t *pid, float setpoint, float measurement)
{
    if (!pid || !pid->cfg || !pid->cfg->enabled) {
        return 0.0f;
    }

    /* Snapshot all volatile config fields once */
    bool     p_valid      = pid->cfg->p_valid;
    bool     i_valid      = pid->cfg->i_valid;
    bool     d_valid      = pid->cfg->d_valid;
    float    kp           = pid->cfg->kp;
    float    ki           = pid->cfg->ki;
    float    kd           = pid->cfg->kd;
    float    output_min   = pid->cfg->output_min;
    float    output_max   = pid->cfg->output_max;
    float    integral_max = pid->cfg->integral_max;
    uint32_t update_ms    = pid->cfg->update_ms;

    /* --- Throttle check --- */
    uint32_t now = (uint32_t)tickTimer_GetCount();
    if (update_ms > 0u && pid->last_update_ms != 0u) {
        uint32_t dt_ms = now - pid->last_update_ms;
        if (dt_ms < update_ms) {
            return pid->last_output;
        }
    }

    /* --- dt in seconds (clamped) --- */
    uint32_t dt_ms;
    if (pid->last_update_ms == 0u) {
        dt_ms = update_ms;
    } else {
        dt_ms = now - pid->last_update_ms;
    }
    if (dt_ms > 1000u) dt_ms = 1000u;
    float dt_s = (float)dt_ms / 1000.0f;

    /* --- Error --- */
    float error = setpoint - measurement;

    /* --- P term --- */
    float p_term = 0.0f;
    if (p_valid) {
        p_term = kp * error;
    }

    /* --- I term (anti-windup) --- */
    float i_term = 0.0f;
    if (i_valid) {
        pid->integral += error * dt_s;
        if (pid->integral > integral_max)  pid->integral = integral_max;
        if (pid->integral < -integral_max) pid->integral = -integral_max;
        i_term = ki * pid->integral;
    }

    /* --- D term (on measurement) --- */
    float d_term = 0.0f;
    if (d_valid && !pid->first_sample && dt_s > 1e-6f) {
        d_term = kd * (pid->prev_measurement - measurement) / dt_s;
    }

    /* --- Sum and clamp --- */
    float output = p_term + i_term + d_term;
    if (output > output_max) output = output_max;
    if (output < output_min) output = output_min;

    /* --- Save state --- */
    pid->prev_measurement  = measurement;
    pid->first_sample      = false;
    pid->last_output       = output;
    pid->last_update_ms    = now;

    return output;
}

/*=============================================================================
 * PID_GetOutput
 *=============================================================================*/
float PID_GetOutput(const pid_state_t *pid)
{
    if (!pid) return 0.0f;
    return pid->last_output;
}
