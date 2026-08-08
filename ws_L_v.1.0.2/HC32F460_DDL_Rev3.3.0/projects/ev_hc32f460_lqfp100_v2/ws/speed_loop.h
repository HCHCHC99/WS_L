#ifndef __SPEED_LOOP_H__
#define __SPEED_LOOP_H__

#include "../Utils/dev_pid.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Keil Watch: current limit when current loop is enabled (mA) */
extern volatile float g_i_ref_max_ma;

/* Speed-loop PID config (Keil Watch tunable) */
extern pid_config_t g_spd_pid_cfg;

/* J-Scope observability */
extern volatile float g_scope_spd_ref;
extern volatile float g_scope_spd_rpm;
extern volatile float g_scope_spd_err;
extern volatile float g_scope_spd_out;

void  SpeedLoop_Init(void);
void  SpeedLoop_SetTarget(float target_rpm);
/* Run one speed-loop iteration (throttled internally by update_ms).
 * Returns output: current ref (mA) when current loop enabled, else duty (%). */
float SpeedLoop_Update(float measured_rpm);
float SpeedLoop_GetOutput(void);
/* Seed output for bumpless handoff (e.g. start from measured current). */
void  SpeedLoop_Seed(float output);

#ifdef __cplusplus
}
#endif

#endif /* __SPEED_LOOP_H__ */
