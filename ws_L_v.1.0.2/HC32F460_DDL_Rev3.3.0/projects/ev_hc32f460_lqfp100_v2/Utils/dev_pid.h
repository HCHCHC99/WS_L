/**
 *******************************************************************************
 * @file  Utils/dev_pid.h
 * @brief Generic PID controller — parallel form with anti-windup,
 *        derivative-on-measurement, and bitmask term selection.
 *
 *        Each PID instance points to a pid_config_t. Fields are volatile so
 *        Keil Watch can modify them at runtime with zero polling overhead.
 *
 *        Each term independently enabled via p_valid / i_valid / d_valid.
 *        No hardware dependencies — pure float math, reusable across modules.
 *
 *        Usage:
 *          pid_config_t g_pid_cfg = { .enabled=true, .p_valid=1, .i_valid=1,
 *                                     .kp=0.02f, .ki=0.005f, ... };
 *          pid_state_t  s_pid;
 *          PID_Init(&s_pid, &g_pid_cfg);
 *          float duty = PID_Update(&s_pid, setpoint, measurement);
 *******************************************************************************
 */

#ifndef __DEV_PID_H__
#define __DEV_PID_H__

#include <stdint.h>
#include <stdbool.h>

/*=============================================================================
 * PID configuration — volatile so Keil Watch can tune at runtime
 *=============================================================================*/
typedef struct {
    volatile bool       enabled;       /* false = PID_Update returns 0.0f */
    volatile bool       p_valid;       /* 1 = P term active */
    volatile bool       i_valid;       /* 1 = I term active */
    volatile bool       d_valid;       /* 1 = D term active */
    volatile float      kp;            /* Proportional gain */
    volatile float      ki;            /* Integral gain */
    volatile float      kd;            /* Derivative gain */
    volatile float      output_min;    /* Output clamp min (e.g. 2.0f) */
    volatile float      output_max;    /* Output clamp max (e.g. 98.0f) */
    volatile float      integral_max;  /* Anti-windup clamp */
    volatile uint32_t   update_ms;     /* Minimum update interval (ms) */
} pid_config_t;

/*=============================================================================
 * PID runtime state — caller allocates, PID_Init / PID_Reset manage
 *=============================================================================*/
typedef struct {
    pid_config_t *cfg;            /* Points to live config (NULL = disabled) */
    float         integral;
    float         prev_measurement;
    float         last_output;
    bool          first_sample;
    uint32_t      last_update_ms;
} pid_state_t;

/*=============================================================================
 * API
 *=============================================================================*/

/* Bind state to config. cfg=NULL disables the PID. */
void PID_Init(pid_state_t *pid, pid_config_t *cfg);

/* Reset accumulators. Call on mode transitions. */
void PID_Reset(pid_state_t *pid);

/* Run one PID iteration. Reads live params from cfg each call. */
float PID_Update(pid_state_t *pid, float setpoint, float measurement);

/* Get last output */
float PID_GetOutput(const pid_state_t *pid);

#endif /* __DEV_PID_H__ */
