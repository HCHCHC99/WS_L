# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

BLDC motor controller firmware for the HC32F460 (HDSC ARM Cortex-M4) MCU on the `ev_hc32f460_lqfp100_v2` evaluation board. Uses 6288T-MNS gate driver, 3-channel Hall-effect sensors (120° placement), trapezoidal 6-step commutation, and RS485/Modbus RTU for external control.

## Build & Toolchain

- **Primary IDE**: Keil MDK (ARM Compiler 5/6). Open `HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/template/MDK/template.uvprojx`
- **Alternative**: IAR EWARM (`.ewp`/`.eww` in `template/EWARM/`) and Eclipse GCC (`.cproject`/`.project` in `template/GCC/`)
- **SDK**: HC32F460 DDL (Device Driver Library) Rev3.3.0, located at `HC32F460_DDL_Rev3.3.0/`
- **Build output**: `template/MDK/output/debug/template.axf` (ELF for J-Link/J-Scope)
- **Debug print**: J-Link RTT (SEGGER RTT library at `projects/.../RTT/`). Debug macros are gated by `APP_COMM_DBG`, `DEV_EVENT_BUS`, `DEV_EVENT_BUS_VERBOSE`, `APP_PARAMS_DBG`, etc.

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
│   ├── Bemf.c/h                  # BEMF observer: 4-channel ADC via DMA, PWM-peak triggered sampling (observer-only, no sensorless control)
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
├── Utils/                        # Utilities
│   ├── param_manager.c/h         # Flash parameter persistence with CRC32 + magic header/tail
│   ├── Params.c/h                # Parameter struct + register address definitions (Modbus holding registers)
│   ├── ring_buf.c/h, msg_queue.c/h, lock.c/h, TickTimer.c/h, rtt_manager.c/h
```

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
- **Calibration offsets**: `g_calib_cw_table = (g_calib_table + 5) % 6` and `g_calib_ccw_table = (g_calib_table + 2) % 6`. The offsets (+5 CW, +2 CCW) are determined by the control board hardware (6288T-MNS truth table + TMR4 wiring), not the motor — changing the motor does not require re-tuning these.
- **6288T-MNS driver logic**: All PWM in THROUGH mode. HIGH_SIDE mode = H OC enabled (PWM) + L OC disabled (LOW). LOW_SIDE mode = H OC disabled (LOW) + L OC enabled (100%). OFF mode = both disabled (LOW). The 6288T-MNS conducts when H≠L (differential) and turns off when H=L (same level) — opposite of SDH21263.

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
| BEMF M/U/V/W | PA0/PA1/PA2/PA3 | ADC1 CH0–CH3 |

## Common Tasks

### Adding a new Modbus register
Edit only `App/App_Comm.c`:
1. Add `case REG_NEW: *value = ...; break;` in `App_Comm_OnReadReg()`
2. Add corresponding write handler in `App_Comm_OnWriteReg()`
No changes needed in protocol, HAL, or PHY layers.

### Changing PWM mode/strategy for a different gate driver
Edit only `Adp/tmr4_pwm.c` and `Adp/tmr4_pwm.h` (mode enum). The upper layers (`dev_commutation.c`, `dev_comm_runner.c`) are abstracted through mode enums and unaffected by driver chip changes.

### Using the Modbus test tool
Run `python modbus_test_cmds.py` from the project root to send Modbus commands to the device via RS485. Allows reading real-time data registers (0x2730–0x2740) and writing control/parameter registers.

### Viewing real-time waveforms
J-Link + J-Scope in HSS mode, loading `template/MDK/output/debug/template.axf`. Key scope variables: `g_scope_ha`, `g_scope_hb`, `g_scope_hc`, `g_scope_step`, `g_scope_rpm` (all `volatile`, in `hall_sensor_3ch.c`).

### Running calibration
Set `comm_mode = 5` in Keil Watch → wait for `g_calib_status = 2` → verify `g_calib_table[1..6]` has all unique values 0–5 → then use `comm_mode = 6` (CW) or `7` (CCW) for closed-loop with calibrated table.
