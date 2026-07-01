# IU/IV/IW Current Sensing Module

## Overview

Three-phase BLDC motor current sensing using ADC1 SEQ_B interrupt mode on HC32F460.
Samples IU (PA5), IV (PA6), IW (PA7) at PWM peak (50 kHz), synchronized with the BEMF module.

### Hardware

| Signal | Pin | ADC Channel | Sensor Model |
|--------|-----|-------------|--------------|
| IU | PA5 | ADC1_CH5 | ACSxxx (VOUT = 1650 + IP × 132 mV, ±10 A) |
| IV | PA6 | ADC1_CH6 | ACSxxx (same) |
| IW | PA7 | ADC1_CH7 | ACSxxx (same) |

- ADC reference: 3.3 V, 12-bit resolution (0–4095)
- Zero-bias: 1650 mV → ADC raw ≈ 2048
- Sensitivity: 132 mV/A → ~6.105 mA per ADC count

### Files

| File | Role |
|------|------|
| `ws/I.h` | Module interface, macro definitions, J-Scope globals |
| `ws/I.c` | Full implementation: ADC config, ISR, calibration, EMA filter |
| `Adp/Aos.c` | AOS routing (unused after switching to shared SCMP0 trigger) |
| `Adp/Bemf.c` | ADC mode changed from `ADC_MD_SEQA_SINGLESHOT` to `ADC_MD_SEQA_SEQB_SINGLESHOT` |
| `template/source/main.c` | Calls `I_Init()` then `I_Calibrate()` at startup |

---

## Architecture

### ADC1 Resource Split

```
ADC1:
  SEQ_A (CH0–CH3): BEMF        ─ SCMP0 → AOS_ADC1_0 → DMA1 (EOCA)
  SEQ_B (CH5–CH7): Current     ─ SCMP0 → AOS_ADC1_0 → EOCB ISR
```

Both sequences share the **same trigger** (TMR4_3 SCMP0 at PWM counter peak, 50 kHz).
SEQ_A converts first, then SEQ_B; each generates its own end-of-conversion signal
(EOCA → DMA for BEMF; EOCB → interrupt for current).

### Trigger Chain

```
TMR4_3 counter peak (center-aligned PWM, 50 kHz)
  └→ EVT CH_UH compare match
       └→ SCMP0 event
            └→ AOS_ADC1_0 (ADC1_TRGSEL0)
                 ├→ ADC1 SEQ_A (BEMF, EVT0) → EOCA → DMA1 CH0/1/2/3
                 └→ ADC1 SEQ_B (Current, EVT0) → EOCB → ISR (INT116)
```

### ADC Scan Mode

BEMF's `Bemf_AdcConfig()` sets the ADC1 mode register:

```c
// Before: SEQ_A single-shot, SEQ_B disabled
stcAdcInit.u16ScanMode = ADC_MD_SEQA_SINGLESHOT;

// After: both sequences independent single-shot
stcAdcInit.u16ScanMode = ADC_MD_SEQA_SEQB_SINGLESHOT;
```

> **Critical**: `ADC_MD_SEQA_SINGLESHOT` disables SEQ_B entirely.
> Changing to `ADC_MD_SEQA_SEQB_SINGLESHOT` is required for current sensing to work.

### Interrupt Priority

| ISR | IRQn | NVIC Priority | Rationale |
|-----|------|--------------|-----------|
| Hall U/V/W | INT008/009/010 | `DDL_IRQ_PRIO_02` (2) | Highest — commutation timing is critical |
| Current ADC1 EOCB | INT116 | `DDL_IRQ_PRIO_03` (3) | Mid — overcurrent needs fast response |
| BEMF ADC1 DMA | INT116 (DMA BTC) | `DDL_IRQ_PRIO_03` (3) | Same level, independent ADC resources |
| Timer0 1 ms tick | INT006/007 | `DDL_IRQ_PRIO_04` (4) | Lowest — soft real-time |

> **Fix**: Timer0_Unit1 and Timer0_Unit2 previously used `1UL` (equivalent to `DDL_IRQ_PRIO_01`),
> which was **higher** than Hall. Changed to `DDL_IRQ_PRIO_04`.

---

## Initialization

Called from `main()`:

```c
Hardware_Init();       // Clocks, GPIO, AOS_Init (Timer0 → ADC1)
App_Comm_Init();       // RS485 + Modbus
CommRunner_Init();     // TMR4 PWM started (50 kHz), STOP mode
Bemf_Init();           // ADC1_SEQ_A (CH0–3), DMA, SCMP0 trigger
I_Init();              // ADC1_SEQ_B (CH5–7), EOCB ISR
I_Calibrate();         // 500 ms blocking zero-offset calibration
EventBus_Enable();     // Start event dispatching
while (1) { ... }
```

### `I_Init()` steps

1. **`I_AdcConfig()`** — Sets PA5/PA6/PA7 to analog mode (`PIN_ATTR_ANALOG`).
   Enables ADC1_CH5/CH6/CH7 on SEQ_B. Does **not** call `ADC_Init()` —
   ADC1 mode was already configured by BEMF.

2. **`I_TriggerConfig()`** — Configures ADC1 SEQ_B hardware trigger:
   ```c
   ADC_TriggerConfig(CM_ADC1, ADC_SEQ_B, ADC_HARDTRIG_EVT0);
   ADC_TriggerCmd(CM_ADC1, ADC_SEQ_B, ENABLE);
   ```
   EVT0 maps to AOS_ADC1_0, the same SCMP0 event used by BEMF.

3. **`I_IrqConfig()`** — Registers EOCB callback via `INTC_IrqSignIn`:
   ```c
   INT_SRC_ADC1_EOCB (449) → INT116_IRQn, priority DDL_IRQ_PRIO_03
   ADC_IntCmd(CM_ADC1, ADC_INT_EOCB, ENABLE);
   ```

---

## Zero-Offset Calibration

### Problem

Each current sensor has slightly different zero-bias (resistor tolerance, VREF drift).
Without calibration, three phases show different non-zero readings at idle,
and the three-phase sum deviates from zero.

### `I_Calibrate()` — Blocking 500 ms

1. Forces all three PWM channels to **OFF** (`TMR4_MODE_OFF`):
   ```c
   TMR4_PWM_SetChannelMode(U, TMR4_MODE_OFF, 0.0f);
   TMR4_PWM_SetChannelMode(V, TMR4_MODE_OFF, 0.0f);
   TMR4_PWM_SetChannelMode(W, TMR4_MODE_OFF, 0.0f);
   ```
   This ensures all FETs are off — motor windings are truly floating, current is zero.

2. Sets `g_i_calib_state = 1`. The ISR enters accumulation mode:
   ```c
   if (g_i_calib_state == 1) {
       s_i32CalibSumU += (int32_t)u16IU;
       s_i32CalibSumV += (int32_t)u16IV;
       s_i32CalibSumW += (int32_t)u16IW;
       s_i32CalibCnt++;
   }
   ```

3. Blocks 500 ms via `tickTimer_DelayMs(500)`. During this period,
   the ISR fires ~25,000 times (50 kHz × 0.5 s).

4. Sets `g_i_calib_state = 2`. Computes per-phase average:
   ```c
   g_i_calib_zero_u = s_i32CalibSumU / s_i32CalibCnt;
   g_i_calib_zero_v = s_i32CalibSumV / s_i32CalibCnt;
   g_i_calib_zero_w = s_i32CalibSumW / s_i32CalibCnt;
   ```

5. From this point, the ISR uses the calibrated zero references:
   ```c
   uint16_t u16ZeroU = (g_i_calib_state == 2) ? g_i_calib_zero_u : I_ADC_ZERO; // I_ADC_ZERO = 2048
   int16_t IU_mA = I_ADC_TO_MA_REF(u16IU, u16ZeroU);
   ```

### Current Conversion

```c
// Formula: I_mA = (ADC_raw - zero_ref) × 3300 × 1000 / (4095 × 132)
// Fixed-point integer: I_mA = (diff × 1563) >> 8, error < 0.01%
#define I_MA_PER_ADC  1563   // 3300 × 1000 × 256 / (4095 × 132) = 6.105 × 256 ≈ 1563
#define I_MA_SHIFT    8

#define I_ADC_TO_MA_REF(raw, zero) \
    ((int16_t)(((int32_t)((raw) - (zero)) * I_MA_PER_ADC) >> I_MA_SHIFT))
```

---

## EMA Low-Pass Filter

To suppress PWM switching noise and ADC quantization noise, an exponential moving
average filter runs inside the ISR:

```c
#define I_EMA_ALPHA    8     // α = 8/256 ≈ 3.1%
#define I_EMA_1MALPHA  248   // 1 − α = 248/256

// Q8 fixed-point: state = actual_mA × 256
s_i32EmaU = ((248 * s_i32EmaU) + (raw_mA * 2048)) >> 8;  // 2048 = 8 × 256
```

- **Alpha**: 8/256 ≈ 3.1%
- **Cutoff frequency**: ~250 Hz at 50 kHz sample rate
- **Response**: reaches 95% of step change in ~12 ms (600 samples)
- The BLDC fundamental current (tens to hundreds of Hz) passes through undistorted;
  PWM ripple and ADC noise are attenuated.

On the first sample (`s_bEmaInit == false`), the filter state is seeded directly
to avoid a long ramp-up from zero.

---

## J-Scope Variables

| Variable | Type | Meaning |
|----------|------|---------|
| `g_i_iu_raw` | `uint16_t` | IU raw ADC (0–4095) |
| `g_i_iv_raw` | `uint16_t` | IV raw ADC |
| `g_i_iw_raw` | `uint16_t` | IW raw ADC |
| `g_i_iu_ma` | `int16_t` | IU instantaneous current (mA, signed) |
| `g_i_iv_ma` | `int16_t` | IV instantaneous current |
| `g_i_iw_ma` | `int16_t` | IW instantaneous current |
| `g_i_iu_filt` | `int32_t` | IU EMA-filtered current (Q8: ÷256 = mA) |
| `g_i_iv_filt` | `int32_t` | IV EMA-filtered current (Q8) |
| `g_i_iw_filt` | `int32_t` | IW EMA-filtered current (Q8) |
| `g_i_iu_disp` | `uint16_t` | IU filtered mA + 10000 (always ≥ 0, J-Scope safe) |
| `g_i_iv_disp` | `uint16_t` | IV filtered mA + 10000 |
| `g_i_iw_disp` | `uint16_t` | IW filtered mA + 10000 |
| `g_i_uvw_raw` | `int32_t` | Raw sum IU+IV+IW (should ≈ 3 × 2048 = 6144) |
| `g_i_uvw_ma` | `int32_t` | Current sum IU+IV+IW in mA (should ≈ 0) |
| `g_i_calib_state` | `uint8_t` | 0=idle, 1=calibrating, 2=done |
| `g_i_calib_zero_u` | `uint16_t` | Calibrated zero reference for IU (ADC raw) |
| `g_i_calib_zero_v` | `uint16_t` | Calibrated zero reference for IV |
| `g_i_calib_zero_w` | `uint16_t` | Calibrated zero reference for IW |
| `g_i_sample_cnt` | `uint32_t` | Cumulative sample count (confirms ISR is running) |
| `g_i_running` | `uint8_t` | Module status (0=stopped, 1=running) |

### Recommended J-Scope Display

- **Waveform view**: `g_i_iu_disp`, `g_i_iv_disp`, `g_i_iw_disp` in the same sub-plot
  - Y-axis: 0–20000 (10000 = 0 mA; 20000 = +10 A; 0 = −10 A)
- **Precision value**: `g_i_iu_filt`, `g_i_iv_filt`, `g_i_iw_filt`
  - Y-axis: auto-scale. Multiply displayed value × 0.00390625 (÷256) for mA.
- **Raw check**: `g_i_iu_raw`, `g_i_iv_raw`, `g_i_iw_raw`
  - Y-axis: 0–4095

### Interpreting Values

| Raw ADC | Instant mA | Filtered mA | Meaning |
|---------|-----------|-------------|---------|
| ~calib_zero | ~0 | ~0 | Zero current (floating phase or idle) |
| > calib_zero | > 0 | > 0 | Current flowing INTO motor phase |
| < calib_zero | < 0 | < 0 | Current flowing OUT of motor phase |
| `g_i_uvw_ma` ≈ 0 | — | — | Three-phase balance OK |
| `g_i_uvw_ma` ≠ 0 | — | — | Offset error or sensor fault |

---

## Data Flow Summary

```
Every PWM cycle (20 µs, 50 kHz):

1. TMR4_3 counter reaches peak
2. SCMP0 event fires
3. AOS routes to ADC1_TRGSEL0
4. ADC1 SEQ_A scans CH0→CH1→CH2→CH3 (BEMF, ~1 µs)
5. ADC1 SEQ_B scans CH5(IU)→CH6(IV)→CH7(IW) (Current, ~0.8 µs)
6. ADC1 EOCA → DMA1 BTC → Bemf_DataCallback (BEMF data ready)
7. ADC1 EOCB → I_IrqCallback:
   a. Read DR5, DR6, DR7
   b. Subtract calibrated zero reference
   c. Convert to mA (integer arithmetic)
   d. Apply EMA filter (Q8 accumulation)
   e. Update J-Scope globals
   f. Invoke user callback (if registered)
```

---

## Calibration Checklist

1. Motor must be **stationary** during calibration.
2. Call `I_Calibrate()` after `I_Init()` and before `EventBus_Enable()`.
   (Or ensure PWM channels are OFF during calibration.)
3. Verify RTT output:
   ```
   [I] All PWM channels forced OFF for calibration
   [I] Calibration done: ~25000 samples, zero_ref U=2048 V=2049 W=2047
   ```
   The three zero_ref values should cluster near 2048 (typically 2030–2065).
   A value far outside this range indicates a hardware issue (wiring, sensor fault,
   or pin conflict).
4. After calibration, verify `g_i_uvw_ma ≈ 0` at idle in J-Scope.

---

## Known Limitations

1. **Common gain**: All three phases use the same 132 mV/A slope.
   Individual sensor gain differences are not calibrated.
   Result: small residual imbalance in `g_i_uvw_ma`.

2. **PWM-coupling offset**: Even with FETs OFF, capacitive coupling from
   switching TMR4 pins to adjacent ADC traces may introduce sub-count bias.
   The 500 ms averaging mitigates this.

3. **VREF tolerance**: ADC reference is VDD (nominally 3.3 V), not a precision
   reference. VDD variation directly scales ADC readings.

4. **No overcurrent protection yet**: The ISR only observes. No fault action
   is triggered based on current thresholds.
