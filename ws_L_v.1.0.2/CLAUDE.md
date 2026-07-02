# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

BLDC motor controller firmware for the HC32F460 (HDSC ARM Cortex-M4) MCU on the `ev_hc32f460_lqfp100_v2` evaluation board. Uses 6288T-MNS gate driver, 3-channel Hall-effect sensors (120° placement), trapezoidal 6-step commutation, and RS485/Modbus RTU for external control.

## Build & Toolchain

- **Primary IDE**: Keil MDK (ARM Compiler 5/6). Open `HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template.uvprojx`
- **Alternative**: IAR EWARM (`.ewp`/`.eww` in `template/EWARM/`) and Eclipse GCC (`.cproject`/`.project` in `template/GCC/`)
- **SDK**: HC32F460 DDL (Device Driver Library) Rev3.3.0, located at `HC32F460_DDL_Rev3.3.0/`
- **Build output**: `template/MDK/output/debug/template.axf` (ELF for J-Link/J-Scope), `template.bin` (flash image)
- **Debug print**: J-Link RTT (SEGGER RTT library at `projects/.../RTT/`). Debug macros are gated by `APP_COMM_DBG`, `DEV_EVENT_BUS`, `DEV_EVENT_BUS_VERBOSE`, `APP_PARAMS_DBG`, etc.
- **Build from CLI**: Open Keil MDK, load the `.uvprojx` project, press F7 (Build) or Ctrl+F5 (Debug). There is no standalone Makefile or CMake build — the Keil project file is the sole build definition.
- **Flash/debug hardware**: J-Link debug probe. Debug config in `template/MDK/JLinkSettings.ini` and `config/debug_init.ini`.

## Architecture

### Source Code Layout

```
projects/ev_hc32f460_lqfp100_v2/
├── template/source/main.c        # Entry point, main loop, mode dispatch via Keil Watch variables
├── Adp/                          # Adapter layer — MCU peripheral wrappers
│   ├── Hardware.c/h              # Top-level hardware init (clocks, GPIO, AOS, timers)
│   ├── rs485.c/h                 # USART4 + PA03 + interrupt + callback registration (physical layer)
│   ├── Comm_HAL.c/h              # Ring buffer, frame assembly, TX drain, frame timeout (HAL layer)
│   ├── tmr4_pwm.c/h              # TMR4 PWM abstraction: OFF/HIGH_SIDE/LOW_SIDE modes, shadow registers, THROUGH mode (6288T-MNS compatible)
│   ├── Aos.c/h                   # AOS (Analog Output Switch) routing for ADC/DMA triggers
│   ├── Dma.c/h                   # DMA driver with incremental init support
│   ├── timer6_timebase.c/h       # Timer6 microsecond timebase for Hall edge interval measurement
│   ├── Sysclk.c/h, Gpio_io.c/h, Adc.c/h, Motor_hall.c/h, hc32f46x_flash.c/h, Timer0_Unit1.c/h, Timer0_Unit2.c/h
├── App/                          # Application layer
│   ├── App_Comm.c/h              # Register read/write callbacks, Flash persistence for parameters (top of comm stack)
│   ├── App_FaultHandler.c/h      # Fault bit management, fault clear dispatch
│   ├── App_Motor_Project.c/h     # Motor project orchestration
│   ├── App_Realtime.c/h          # Real-time data (speed, angle, voltage, current, direction, faults)
│   ├── Protocol_ModbusRtu.c/h    # Modbus RTU: CRC16, frame parse, function 0x03/0x06/0x10, exception frames
├── Dev/                          # Device layer — uses EventBus for inter-device communication
│   ├── device_manager.c/h        # Device registry and arbitration
│   ├── EventBus.c/h              # Publish/subscribe event system with priority
│   ├── dev_motor.c/h             # Motor arbitration: block_fwd/block_rev bitmask, multi-source device lock
│   ├── dev_rturn.c/h             # Rotary-turn device: overcurrent handling, angle limits
│   ├── dev_sensor.c/h            # Current sensor: overcurrent detection (sample-count & time-window modes), hysteresis
│   ├── dev_hall.c/h, dev_motor_hall.c/h, dev_adc.c/h, dev_voltage.c/h, dev_power.c/h, dev_pwm.c/h, dev_io.c/h
├── ws/                           # Workspace — commutation engine
│   ├── dev_comm_runner.c/h       # Top-level controller: CW/CCW step tables, mode state machine (STOP/OPEN_FW/OPEN_RV/CLOSED_FW/CLOSED_RV/CALIB/CALIB_CW/CALIB_CCW), calibration state machine, flying-start ramp
│   ├── dev_commutation.c/h       # Six-step commutation state table: step→PWM mode/duty, lazy-update cache
│   ├── hall_sensor_3ch.c/h       # 3-channel Hall driver: ISR, Hall→step lookup, M-method RPM, J-Scope globals, 5-layer noise defense
│   ├── I.c/h                     # 3-phase current sensing: ADC1 SEQ_B (CH5/6/7), PWM-peak triggered, EMA-filtered, zero-offset calibration
│   ├── Bemf.c/h                  # BEMF observer: 4-channel ADC via DMA, PWM-peak triggered sampling (observer-only, no sensorless control)
├── Utils/                        # Utilities
│   ├── param_manager.c/h         # Flash parameter persistence with CRC32 + magic header/tail
│   ├── Params.c/h                # Parameter struct + register address definitions (Modbus holding registers)
│   ├── ring_buf.c/h, msg_queue.c/h, lock.c/h, TickTimer.c/h, rtt_manager.c/h
```

### Init Order (Critical)

The order in `main.c` is mandatory — modules depend on their predecessors:

```
Hardware_Init()        → Clocks, GPIO, AOS (Timer0→ADC1 via AOS_ADC1_0), tick timers
App_Comm_Init()        → RS485 comms (independent of motor)
tickTimer_DelayMs(5)   → Let hardware settle
CommRunner_Init()      → TMR4_PWM_Config + TMR4_PWM_StartOutput (PWM now running!)
Bemf_Init()            → AOS routing (overwrites AOS_ADC1_0 to TMR4_SCMP0) + ADC1_SEQ_A + DMA
I_Init()               → ADC1_SEQ_B (CH5/6/7) + EOCB ISR (shares SCMP0 trigger with BEMF)
I_Calibrate()          → Blocking 500ms, PWM forced OFF, averages zero-current offset
EventBus_Enable()      → Release buffered events after all modules ready
```

**Key AOS constraint**: `AOS_InitForBemf()` in `Bemf_Init()` overwrites the `AOS_ADC1_0` connection that `AOS_Init()` (called from `Hardware_Init()`) set up for `Timer0_CMP_B → ADC1`. If you re-enable PA6 ADC via `Adc.c`, this breaks BEMF timing.

### Communication Stack (4 Layers)

```
App_Comm        → Register callbacks + Flash persistence + control business
Protocol_ModbusRtu → CRC16 + frame structure + function 0x03/0x06/0x10
Comm_HAL        → Ring buffer + frame assembly + frame timeout + TX drain
rs485           → USART4 + PA03 + interrupt + DE direction control
```

**Key design principle**: Each layer has strictly bounded knowledge. `App_Comm` doesn't know about RS485 vs CAN. `Protocol_ModbusRtu` doesn't know register meanings. `Comm_HAL` doesn't know protocol format. `rs485` doesn't know about framing. To switch to a different protocol (e.g., CANopen), create a new Protocol layer file implementing the same callback interface and swap it in `App_Comm_Init`. To switch to a different physical layer (e.g., RS232), create a new PHY file implementing the same HW interface signature.

### Commutation Architecture (CommRunner)

`CommRunner` is the central state machine orchestrating BLDC commutation. Mode flow:

```
STOP(0) → OPEN_FW(1)/OPEN_RV(2) → open-loop ramp at constant interval
                 OR
       → CLOSED_FW(3)/CLOSED_RV(4) → open-loop ramp → flying-start capture → Hall ISR takeover (closed-loop)
                 OR
       → CALIB(5) → open-loop + Hall edge detection → derive 0° offset table → motor stops automatically
       → CALIB_CW(6)/CALIB_CCW(7) → load derived tables + offsets → closed-loop
```

- **Hall→Step tables**: The CW table (`s_hall2step_cw`) uses `reverse_map` (sector -90°), CCW table uses `forward_map` (sector +90°). These are **swapped** relative to their names because this motor's CW rotation corresponds to decreasing electrical angle.
- **Calibration offsets**: `g_calib_cw_table = (g_calib_table + 4) % 6` and `g_calib_ccw_table = (g_calib_table + 2) % 6`. The offsets (+4 CW, +2 CCW) are determined by the control board hardware (6288T-MNS truth table + TMR4 wiring), not the motor — changing the motor does not require re-tuning these.
- **6288T-MNS driver logic**: All PWM in THROUGH mode. HIGH_SIDE mode = H OC enabled (PWM) + L OC disabled (LOW). LOW_SIDE mode = H OC disabled (LOW) + L OC enabled (100%). OFF mode = both disabled (LOW). The 6288T-MNS conducts when H≠L (differential) and turns off when H=L (same level) — opposite of SDH21263.

### Flying Start (`hall_3ch_start_flying`)

For closed-loop modes (3/4/6/7), the motor is already spinning from the open-loop ramp. Flying start captures the current Hall position:
1. Read Hall GPIO twice with a small delay between reads
2. If values differ (rotor moving between samples), use the latest reading
3. Look up step from `hall_to_step` table; fall back to step 0 if invalid code
4. Set FSM directly to `STATE_RUNNING` (bypasses ALIGNING)
5. First Hall ISR will correct any initial position error

### Event System

Devices communicate via `EventBus` publish/subscribe with topic-based routing and subscriber priority. Key topics: `TOPIC_CURRENT_ALARM`, `TOPIC_VOLTAGE_ALARM`, `TOPIC_RTURN_LIMIT`, `TOPIC_FAULT_CLEAR`. Events are buffered until `EventBus_Enable()` is called after all modules initialize.

### Debug Interface (Keil Watch)

Main loop dispatches mode changes via two volatile globals set from the Keil debugger:
- `comm_mode` (0–7): Commutation mode
- `g_comm_duty_pct` (2.0–98.0): PWM duty cycle

`CommRunner_Update()` syncs actual mode back to `comm_mode` for stall-triggered STOP reflection.

## Key Design Patterns

- **Device locking**: `dev_motor` uses `block_fwd`/`block_rev` bitmask fields. Multiple devices (overcurrent, RTurn limit) set bits independently; motor runs only when the mask is clear. This provides multi-source arbitration without priority ordering.
- **Dual protection on overcurrent**: Both `dev_motor` (via `DEV_ID_OVERCUR_FWD`) and `dev_rturn` (via `DEV_ID_RTURN_FWD` + `TOPIC_RTURN_LIMIT`) block forward rotation on overcurrent alarm. If one fails, the other still protects.
- **Lazy PWM update**: `dev_commutation.c` caches last channel modes and only calls `TMR4_PWM_SetChannelMode()` when the mode actually changes. Same-mode duty-only changes use `TMR4_PWM_SetDutyFloat()`. This avoids unnecessary register writes on every commutation step.
- **Hall noise defense**: 5-layer filter in ISR: (1) hardware EXTINT filter DIV64, (2) state-unchanged early return, (3) <50µs interval rejection, (4) invalid code 000/111 rejection, (5) step adjacency check (diff must be ±1). PWM noise bursts produce ~13% BAD rate but are safely intercepted.
- **Parameter persistence**: `AppParamRecord_t` stored in Flash with `head_magic`/`tail_magic` markers and CRC32 checksum. Supports wear-leveling via erase count tracking. Baud rate changes are persisted and apply on next init.
- **Incremental DMA init**: `Dma_Init()` was modified to iterate all instances and skip only those already initialized, enabling multiple independent DMA users (BEMF + future modules) without conflicting.

## Pin Mapping (Critical I/O)

| Signal | Pin | Peripheral |
|--------|-----|------------|
| Hall U (HA) | PA10 | EXTINT_CH10, INT010_IRQn |
| Hall V (HB) | PA9 | EXTINT_CH09, INT009_IRQn |
| Hall W (HC) | PA8 | EXTINT_CH08, INT008_IRQn |
| PWM UH/UL | PB9/PB8 | TMR4_3 |
| PWM VH/VL | PB7/PB6 | TMR4_3 |
| PWM WH/WL | PB5/PB4 | TMR4_3 |
| RS485 TX/RX/DE | PA03 | USART4 |
| BEMF M/U/V/W | PA0/PA1/PA2/PA3 | ADC1 CH0–CH3, DMA1 CH0–CH3 |
| Current IU/IV/IW | PA5/PA6/PA7 | ADC1 CH5–CH7 (SEQ_B) |

### AOS Routing (Critical for BEMF + Current)

| Source | Target | Purpose |
|--------|--------|---------|
| `EVT_SRC_TMR4_3_SCMP0` | `AOS_ADC1_0` | PWM peak → ADC trigger (shared by SEQ_A and SEQ_B) |
| `EVT_SRC_ADC1_EOCA` | `AOS_DMA1_0/1/2/3` | ADC done → DMA CH0–CH3 (BEMF data) |

Current sensing (I.c) shares the same SCMP0 trigger via EVT0 — both SEQ_A (BEMF) and SEQ_B (current) fire at every PWM peak.

## Calibration System

### States (`calib_status_t`)

| Value | State | Meaning |
|-------|-------|---------|
| 0 | CALIB_IDLE | Not running |
| 1 | CALIB_RUNNING | Sampling in progress |
| 2 | CALIB_SUCCESS | Complete, `g_calib_table` valid |
| 3 | CALIB_FAIL_TIMEOUT | Did not complete in 3s |
| 4 | CALIB_FAIL_STALL | Hall states stopped changing |
| 5 | CALIB_FAIL_MISSING | Some Hall state never observed |
| 6 | CALIB_FAIL_DUPLICATE | Hall state mapped to multiple steps |
| 7 | CALIB_FAIL_INVALID | Detected invalid Hall code 0x00 or 0x07 |
| 8 | CALIB_FAIL_AMBIGUOUS | Majority vote below threshold |

### Process

1. 500ms settling delay (motor reaches constant speed)
2. Edge detection on Hall state changes — records `(hall_state, current_open_loop_step)`
3. After all 6 Hall states seen → one cycle complete
4. First valid cycle → lock reference table (`s_calib_ref_table`)
5. Need 12 consecutive cycles matching reference → success
6. On success: `g_calib_table[1..6]` populated with 0-offset mapping, entries 0/7 = 0xFF

### Derived Tables

- `g_calib_cw_table[h] = (g_calib_table[h] + 4) % 6` — sector -90°, used by CALIB_CW (mode 6)
- `g_calib_ccw_table[h] = (g_calib_table[h] + 2) % 6` — sector +90°, used by CALIB_CCW (mode 7)

### Running Calibration

Set `comm_mode = 5` in Keil Watch → wait for `g_calib_status = 2` → verify `g_calib_table[1..6]` has all unique values 0–5 → then use `comm_mode = 6` (CW) or `7` (CCW) for closed-loop with calibrated table.

## BEMF Module (bemf.md)

BEMF is **observer-only** — no zero-crossing detection or sensorless commutation. Key design notes:

- **4-channel DMA**: ADC1 CH0–CH3 via DMA1 CH0–CH3, 8-sample buffer per channel. BTC interrupt fires every 160µs (6.25kHz).
- **Trigger**: TMR4_3 SCMP0 at PWM counter peak (center-aligned triangle wave, 50kHz). EVT channel shares UH PWM channel — separate register sets, no known conflict.
- **Known issue**: BEMF reads driven phase voltage when motor is stopped. The correct approach is to only read the floating phase during six-step commutation (e.g., step 0 reads V, step 1 reads W, etc.). API `Bemf_GetFloatingPhaseBemf(uint8_t step)` is planned but not yet implemented.
- **Voltage calculation ignores resistor divider ratio**: Current mV conversion uses raw `ADC * 3300 / 4096` which gives ADC pin voltage, NOT actual phase voltage. Real circuit has resistor dividers — component values needed from schematic. See `bemf.md` for full details.
- **PA0–PA3 conflict risk**: PA2 may conflict with USART2 alternate functions.

## Modbus Test Tool

Run `python modbus_test_cmds.py` from the project root to generate Modbus RTU frame hex strings. Edit the `OPERATION` and register values in the script, then send the output via RS485.

Operations supported:
- `READ` — Read single register (0x03)
- `WRITE` — Write single register (0x06)
- `WRITE_MULTI` — Write multiple contiguous registers (0x10), saves to Flash in one erase
- `CLEAR_FAULT` — Clear fault bits in REG_FAULT_STATUS (0x2740)
- `CTRL_CMD` — Send control command to REG_CTRL_CMD (0x2720): START/STOP/ESTOP/FWD/REV

### Modbus Register Map (Key Registers)

| Address | Name | Type |
|---------|------|------|
| 0x2710 | REG_NODE_ID | R/W param |
| 0x2711 | REG_TARGET_SPEED | R/W param (r/min) |
| 0x2712 | REG_TARGET_ANGLE | R/W param (0.1°) |
| 0x2714 | REG_VOLTAGE_UPPER_LIMIT | R/W param (0.1V) |
| 0x2715 | REG_VOLTAGE_LOWER_LIMIT | R/W param (0.1V) |
| 0x2716 | REG_CURRENT_UPPER_LIMIT | R/W param (mA) |
| 0x2720 | REG_CTRL_CMD | W only (START/STOP/ESTOP/FWD/REV) |
| 0x2730 | REG_REAL_SPEED | R only |
| 0x2731 | REG_REAL_ANGLE | R only |
| 0x2732 | REG_REAL_VOLTAGE | R only |
| 0x2733 | REG_REAL_CURRENT | R only |
| 0x2737 | REG_REAL_DIRECTION | R only |
| 0x2740 | REG_FAULT_STATUS | R/W (write to clear) |

### RS485 Control Flow

1. Send START (0x0001) to REG_CTRL_CMD → enables RS485 control
2. Send FWD (0x0011) or REV (0x0021) — bit0 (START) must remain set
3. Send STOP (0x0002) to disable RS485 control
4. Send ESTOP (0x0004) to cancel rotation without disabling RS485 control

## Viewing Real-Time Waveforms

J-Link + J-Scope in HSS mode, loading `template/MDK/output/debug/template.axf`. Key scope variables:

**Hall** (in `hall_sensor_3ch.c`):
- `g_scope_ha`, `g_scope_hb`, `g_scope_hc` — Hall GPIO levels (0/1)
- `g_scope_step` — current commutation step (0–5)
- `g_scope_rpm` — filtered RPM

**BEMF** (in `Bemf.c`):
- `g_bemf_u_mv`, `g_bemf_v_mv`, `g_bemf_w_mv` — phase BEMF voltage (mV relative to neutral)
- `g_bemf_m_mv` — neutral point absolute voltage
- `g_bemf_u_raw`, `g_bemf_v_raw`, `g_bemf_w_raw` — raw ADC

**Current** (in `I.c`):
- `g_i_iu_filt`, `g_i_iv_filt`, `g_i_iw_filt` — EMA-filtered current (Q8: divide by 256 for mA)
- `g_i_iu_disp`, `g_i_iv_disp`, `g_i_iw_disp` — display-friendly (mA + 10000 offset)
- `g_i_uvw_ma` — three-phase sum (should be ~0)

## Common Tasks

### Adding a new Modbus register
Edit only `App/App_Comm.c`:
1. Add `case REG_NEW: *value = ...; break;` in `App_Comm_OnReadReg()`
2. Add corresponding write handler in `App_Comm_OnWriteReg()`
No changes needed in protocol, HAL, or PHY layers.

### Changing PWM mode/strategy for a different gate driver
Edit only `Adp/tmr4_pwm.c` and `Adp/tmr4_pwm.h` (mode enum). The upper layers (`dev_commutation.c`, `dev_comm_runner.c`) are abstracted through mode enums and unaffected by driver chip changes.

### Adding a new commutation mode
1. Add enum value to `comm_runner_mode_t` in `dev_comm_runner.h`
2. Add case in `CommRunner_SetMode()` for entry logic
3. Add case in `CommRunner_Update()` for runtime behavior
4. Update `main.c` if new Keil Watch value needed
