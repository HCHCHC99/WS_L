# foc_exp - headless FOC experiment runner

Automates the three diagnostic experiments from the HC32F460 FOC current-loop
handoff using only J-Link Commander (no Keil GUI, no Python). It:

1. parses the symbol addresses fresh from `template.map` (safe across rebuilds),
2. writes the Watch-tunable variables directly into RAM (halt only while PWM
   is off),
3. starts comm_mode 21/22/23,
4. samples `g_foc_*` observables once per second while the core is running
   (background RAM reads through J-Link),
5. writes a CSV and prints a summary (vq saturation, spd range, handover/
   timeout verdict).

## Prerequisites

- J-Link on SWD, board powered (12 V bus), motor free to spin.
- `F:\SEGGER\JLink\JLink.exe` (V7.92d is installed).
- After ANY firmware rebuild, flash the new image and use the matching map
  (both live in `template/MDK/output/debug/`). Use `-FlashHex` on the first
  run so the flashed image matches the map.

## Usage

PowerShell:

    cd D:\WS_L\ws_L_v.inmop_curloop\scripts\foc_exp

    # 1) mode21 control: does the 5 Hz field spin the rotor forward?
    .\foc_exp.ps1 -Mode 21 -FlashHex -DurationSec 12

    # 2) four direction combos (reflash not needed after mode21 run)
    .\foc_exp.ps1 -Mode 22 -PiOff180 0 -EncDir  1 -DurationSec 20
    .\foc_exp.ps1 -Mode 22 -PiOff180 0 -EncDir -1 -DurationSec 20
    .\foc_exp.ps1 -Mode 22 -PiOff180 1 -EncDir  1 -DurationSec 20
    .\foc_exp.ps1 -Mode 22 -PiOff180 1 -EncDir -1 -DurationSec 20

    # 3) tuned parameter run (the handoff's next-step values are the defaults)
    .\foc_exp.ps1 -Mode 22 -PiOff180 1 -EncDir -1 -DurationSec 20 `
        -IqRefMa 2000 -VmaxV 1.0 -VrampVS 0.5 -HoldIqMa 800 -Kp 0.1 -Ki 240 -OutputMaxV 1.0

## What each run reports

| field      | meaning                                                        |
|------------|----------------------------------------------------------------|
| spd_rpm    | encoder speed sign + value; must be +/-30 rpm at a 5 Hz field  |
| vq_mV      | q-axis PI output; pinned at +/-output_max while iq < iqref = voltage saturation |
| vlim_mV    | voltage envelope (ramps to g_foc_vmax_v)                       |
| diff_mrad  | wrapped enc_elec - theta angle difference                     |
| sweep_cHz  | live diff wrap rate (0 when rotor follows the field)           |
| evt / fault| evt=2 handover OK; fault=2 I-F timeout; fault=1 over-current   |

The CSV is in `out/run_<timestamp>.csv`; the raw J-Link session log and the
generated J-Link command file are in the same folder.

## Manual Keil fallback (same data, GUI)

1. Flash `template/MDK/output/debug/template.hex` (contains the vq/vlim
   telemetry added in the `main.c` 1 s / IFB / TIMEOUT prints).
2. Debug with J-Link, run, then in Watch set `comm_mode` 0 -> 21/22/23 and the
   sign flags (`g_foc_enc_dir` / `g_foc_pi_off_180` are int32 now).
3. Read `[FOC][IF]`, `[FOC][IFB]`, `[FOC][OL]`, `[FOC] TIMEOUT` lines in the
   RTT Viewer.

## Safety notes

- Never halt the core while the motor is running (frozen PWM = DC into the
  windings). This script halts only while `comm_mode=0`.
- Watch `g_foc_fault` and the OC limit (5.5 A default); kill power immediately
  if the motor screams or the driver heats.
- After changing `g_foc_enc_dir`, always restart with `comm_mode=0 -> 22`
  (the handover anchor is stored using the enc_dir in effect at that moment).