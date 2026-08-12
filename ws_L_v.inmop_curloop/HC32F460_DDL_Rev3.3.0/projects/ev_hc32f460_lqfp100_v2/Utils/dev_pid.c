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

void PID_Seed(pid_state_t *pid, float setpoint, float measurement, float output)
{
    if (!pid || !pid->cfg || !pid->cfg->enabled) {
        return;
    }

    if (output < pid->cfg->output_min) output = pid->cfg->output_min;
    if (output > pid->cfg->output_max) output = pid->cfg->output_max;

    float error = setpoint - measurement;
    float p_term = pid->cfg->p_valid ? (pid->cfg->kp * error) : 0.0f;

    float integral = 0.0f;
    if (pid->cfg->i_valid && pid->cfg->ki > 0.0f) {
        integral = (output - p_term) / pid->cfg->ki;
        float i_clamp = (pid->cfg->i_term_max > 0.0f)
                      ? (pid->cfg->i_term_max / pid->cfg->ki)
                      : pid->cfg->integral_max;
        if (integral >  i_clamp) integral =  i_clamp;
        if (integral < -i_clamp) integral = -i_clamp;
    }

    pid->integral          = integral;
    pid->prev_measurement  = measurement;
    pid->first_sample      = false;
    pid->last_output       = output;
    pid->last_update_ms    = (uint32_t)tickTimer_GetCount();  /* next Update throttled -> returns seed */
    pid->p_term            = p_term;
    pid->i_term            = pid->cfg->i_valid ? (pid->cfg->ki * integral) : 0.0f;
    pid->d_term            = 0.0f;
    pid->last_update_us    = 0;
}

/*=============================================================================
 * PID_Update — parallel-form with anti-windup, derivative-on-measurement.
 *
 * Reads live parameters from pid->cfg each call. Snapshot volatile fields
 * once at the top so one computation uses a consistent set of parameters.
 *=============================================================================*/
/* Core: one PID iteration with already-computed dt. Assumes cfg valid + enabled. */
static float pid_compute(pid_state_t *pid, float setpoint, float measurement, float dt_s)
{
    pid_config_t *cfg = pid->cfg;

    bool     p_valid      = cfg->p_valid;
    bool     i_valid      = cfg->i_valid;
    bool     d_valid      = cfg->d_valid;
    float    kp           = cfg->kp;
    float    ki           = cfg->ki;
    float    kd           = cfg->kd;
    float    output_min   = cfg->output_min;
    float    output_max   = cfg->output_max;
    float    integral_max = cfg->integral_max;
    float    i_term_max   = cfg->i_term_max;

    float error = setpoint - measurement;

    float p_term = 0.0f;
    if (p_valid) {
        p_term = kp * error;
    }

    float i_term = 0.0f;
    if (i_valid) {
        pid->integral += error * dt_s;
        if (pid->integral >  integral_max) pid->integral =  integral_max;
        if (pid->integral < -integral_max) pid->integral = -integral_max;
        i_term = ki * pid->integral;
        if (i_term_max > 0.0f) {
            if (i_term >  i_term_max) i_term =  i_term_max;
            if (i_term < -i_term_max) i_term = -i_term_max;
        }
    }

    float d_term = 0.0f;
    if (d_valid && !pid->first_sample && dt_s > 1e-6f) {
        d_term = kd * (pid->prev_measurement - measurement) / dt_s;
    }

    float output = p_term + i_term + d_term;
    if (output > output_max) output = output_max;
    if (output < output_min) output = output_min;

    pid->prev_measurement  = measurement;
    pid->first_sample      = false;
    pid->last_output       = output;
    pid->p_term            = p_term;
    pid->i_term            = i_term;
    pid->d_term            = d_term;
    return output;
}

float PID_Update(pid_state_t *pid, float setpoint, float measurement)
{
    if (!pid || !pid->cfg || !pid->cfg->enabled) {
        return 0.0f;
    }

    uint32_t update_ms = pid->cfg->update_ms;
    uint32_t now = (uint32_t)tickTimer_GetCount();

    /* --- Throttle check --- */
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

    float output = pid_compute(pid, setpoint, measurement, dt_s);
    pid->last_update_ms = now;
    return output;
}

float PID_UpdateUs(pid_state_t *pid, float setpoint, float measurement, uint32_t dt_us)
{
    if (!pid || !pid->cfg || !pid->cfg->enabled) {
        return 0.0f;
    }

    /* --- Throttle (accumulated us). First call (first_sample) always executes. --- */
    uint32_t update_ms = pid->cfg->update_ms;
    if (update_ms > 0u && !pid->first_sample) {
        pid->last_update_us += dt_us;
        if (pid->last_update_us < update_ms * 1000u) {
            return pid->last_output;
        }
        pid->last_update_us = 0u;
    }

    /* --- dt clamp: max 1s (protects after debugger halt) --- */
    if (dt_us > 1000000u) dt_us = 1000000u;
    float dt_s = (float)dt_us / 1000000.0f;

    return pid_compute(pid, setpoint, measurement, dt_s);
}


/*=============================================================================
 * PID_GetOutput
 *=============================================================================*/
float PID_GetOutput(const pid_state_t *pid)
{
    if (!pid) return 0.0f;
    return pid->last_output;
}
