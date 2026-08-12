# PID Speed Controller

## Files

| File | Role |
|------|------|
| `Utils/dev_pid.h` | PID config struct + API |
| `Utils/dev_pid.c` | PID algorithm (parallel form, anti-windup, D-on-measurement) |
| `ws/dev_comm_runner.h` | `COMM_RUNNER_PID_CW=8`, `COMM_RUNNER_PID_CCW=9`, `pid_cfg` in config, `SetTargetRPM/GetTargetRPM` API |
| `ws/dev_comm_runner.c` | Mode 8/9 state machine, PID integration in sub_phase 1 |
| `template/source/main.c` | `g_pid_cfg` global config + `g_target_rpm` Keil Watch variable |

## Data Structures

### `pid_config_t` — Live PID configuration (all fields volatile, Keil Watch modifiable)

```c
typedef struct {
    volatile bool       enabled;       // false = PID_Update returns 0.0f
    volatile bool       p_valid;       // 1 = P term active
    volatile bool       i_valid;       // 1 = I term active
    volatile bool       d_valid;       // 1 = D term active
    volatile float      kp;            // Proportional gain (duty_pct per rpm error)
    volatile float      ki;            // Integral gain (duty_pct per rpm*s)
    volatile float      kd;            // Derivative gain (duty_pct per rpm/s)
    volatile float      output_min;    // Output clamp minimum (2.0 %)
    volatile float      output_max;    // Output clamp maximum (98.0 %)
    volatile float      integral_max;  // Anti-windup: absolute integral clamp
    volatile uint32_t   update_ms;     // PID update interval (ms)
} pid_config_t;
```

### `pid_state_t` — PID runtime state (internal, not Keil Watch visible)

```c
typedef struct {
    pid_config_t *cfg;            // Points to live config (NULL = disabled)
    float         integral;       // Accumulated integral term
    float         prev_measurement;
    float         last_output;    // Cached output for throttled intervals
    bool          first_sample;
    uint32_t      last_update_ms;
} pid_state_t;
```

## PID API

```c
void  PID_Init(pid_state_t *pid, pid_config_t *cfg);  // Bind state to config
void  PID_Reset(pid_state_t *pid);                     // Clear integral + history
float PID_Update(pid_state_t *pid, float setpoint, float measurement);  // Returns duty_pct
float PID_GetOutput(const pid_state_t *pid);           // Last computed output
```

## Algorithm

Parallel-form PID executed every `update_ms` milliseconds:

```
error = setpoint - measurement

P_term = p_valid ? (kp * error) : 0

I_term:
  if i_valid:
    integral += error * dt_s
    integral = clamp(integral, -integral_max, +integral_max)
    I_term = ki * integral
  else:
    I_term = 0

D_term:
  if d_valid AND not first_sample:
    D_term = kd * (prev_measurement - measurement) / dt_s   // on measurement
  else:
    D_term = 0

output = clamp(P_term + I_term + D_term, output_min, output_max)
```

Key features:
- **Derivative on measurement**: uses `-d_measurement/dt`, avoids kick on setpoint changes
- **Anti-windup**: integral clamped to `+/-integral_max` before multiplying by Ki
- **Internal throttling**: if called before `update_ms` elapsed, returns cached `last_output`
- **dt clamp**: max 1000ms to prevent windup after debugger halt
- **All config fields volatile**: Keil Watch changes take effect on next `PID_Update`

## comm_mode=8/9 Flow

### Precondition
`comm_mode=5` (calibration) must complete successfully → `g_calib_status == 2` → `g_calib_table[1..6]` valid.

### Mode Entry (`CommRunner_SetMode`)

```
comm_mode=8 (PID_CW):                      comm_mode=9 (PID_CCW):
  hall_3ch_stop()                            hall_3ch_stop()
  calib_build_derived_tables()               calib_build_derived_tables()
  start_open_loop(dir_fw=1)                  start_open_loop(dir_fw=0)
  s_sub_phase = 0                            s_sub_phase = 0
  PID_Reset()                                PID_Reset()
```

### Runtime (`CommRunner_Update`)

**sub_phase 0 — Open-loop ramp:**
```
open_loop_tick(now, is_fw)
    → step timer: 20000us -> 3000us ramp over 2000ms
    → Commutation_Step(step, 50000Hz, s_duty)

Ramp complete (elapsed >= 2000ms):
    PID_CW:                                      PID_CCW:
      hall_3ch_set_table(g_calib_cw_table)         hall_3ch_set_table(g_calib_ccw_table)
      hall_3ch_start_flying(HALL3_DIR_FORWARD)     hall_3ch_start_flying(HALL3_DIR_REVERSE)
    s_sub_phase = 1
```

**sub_phase 1 — Closed-loop + PID:**
```
hall_3ch_update(s_hall)        → stall detection + RPM update

if stalled → CommRunner_SetMode(STOP) → exit

PID speed control:
    rpm  = hall_3ch_get_rpm(s_hall)
    duty = PID_Update(&s_pid, s_target_rpm, rpm)
    if duty > 0.0f:
        CommRunner_SetDuty(duty)    → next Hall ISR applies new duty
```

### Derivative Tables

```
g_calib_table[hall]        → 0-offset mapping (from calibration, mode 5)
g_calib_cw_table[hall]     = (g_calib_table[hall] + 4) % 6   → sector -90 deg
g_calib_ccw_table[hall]    = (g_calib_table[hall] + 2) % 6   → sector +90 deg
```

## Keil Watch Variables

### Setpoint (in main.c)
| Variable | Type | Default | Meaning |
|----------|------|---------|---------|
| `g_target_rpm` | `volatile float` | `3000.0f` | Target speed (r/min) |

### PID Config (in main.c, `g_pid_cfg.*`)
| Variable | Type | Default | Keil Watch path |
|----------|------|---------|-----------------|
| `enabled` | `volatile bool` | `true` | `g_pid_cfg.enabled` |
| `p_valid` | `volatile bool` | `true` | `g_pid_cfg.p_valid` |
| `i_valid` | `volatile bool` | `true` | `g_pid_cfg.i_valid` |
| `d_valid` | `volatile bool` | `false` | `g_pid_cfg.d_valid` |
| `kp` | `volatile float` | `0.02` | `g_pid_cfg.kp` |
| `ki` | `volatile float` | `0.005` | `g_pid_cfg.ki` |
| `kd` | `volatile float` | `0.0` | `g_pid_cfg.kd` |
| `output_min` | `volatile float` | `2.0` | `g_pid_cfg.output_min` |
| `output_max` | `volatile float` | `98.0` | `g_pid_cfg.output_max` |
| `integral_max` | `volatile float` | `50.0` | `g_pid_cfg.integral_max` |
| `update_ms` | `volatile uint32_t` | `50` | `g_pid_cfg.update_ms` |

### J-Scope (in dev_comm_runner.c)
| Variable | Type | Meaning |
|----------|------|---------|
| `g_scope_pid_target` | `volatile float` | Target RPM |
| `g_scope_pid_error` | `volatile float` | Speed error (RPM) |
| `g_scope_pid_duty` | `volatile float` | PID output duty (%) |
| `g_scope_pid_i_term` | `volatile float` | Integral accumulator |

### RTT Debug Print (every 500ms, mode 8/9 sub_phase 1 only)
```
[PID] Tgt=3000 Act=2850 Err=150 Duty=638 Int=25
```
- `Duty` = duty_pct × 10 (638 = 63.8%)
- All integer format (MAIN_D constraint)

## Usage Flow

```
1. comm_mode=5 → wait for g_calib_status=2 (calibration success)
2. Set g_target_rpm to desired speed in Keil Watch
3. comm_mode=8 (CW) or comm_mode=9 (CCW)
4. Motor spins: 500ms open-loop ramp → flying start → Hall closed-loop + PID
5. Tune g_pid_cfg.kp / .ki / .p_valid / .i_valid in Keil Watch as needed
6. comm_mode=0 to stop
```
