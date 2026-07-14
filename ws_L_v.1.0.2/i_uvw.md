# IU/IV/IW Current Sensing Module

## Overview

Three-phase BLDC motor current sensing using ADC1 SEQ_B interrupt mode on HC32F460.
Samples IU (PA5), IV (PA6), IW (PA7) at PWM peak (50 kHz), synchronized with the BEMF module.

### Hardware

| Signal | Pin | ADC Channel | Sensor Model |
|--------|-----|-------------|--------------|
| IU | PA5 | ADC1_CH5 | ACSxxx (VOUT = 1650 + IP x 528 mV, +-2.5 A) |
| IV | PA6 | ADC1_CH6 | ACSxxx (same) |
| IW | PA7 | ADC1_CH7 | ACSxxx (same) |

- ADC reference: 3.3 V, 12-bit resolution (0-4095)
- Zero-bias: 1650 mV -> ADC raw ~ 2048
- Sensitivity: 528 mV/A -> ~1.527 mA per ADC count
- I_MA_PER_ADC = 391 (Q8: 1.527 x 256)

### Files

| File | Role |
|------|------|
| `ws/I.h` | Module interface, macro definitions, J-Scope globals |
| `ws/I.c` | Full implementation: ADC config, ISR, calibration, Biquad filter |
| `Adp/Bemf.c` | ADC mode changed from `ADC_MD_SEQA_SINGLESHOT` to `ADC_MD_SEQA_SEQB_SINGLESHOT` |
| `template/source/main.c` | VOFA+ current send: `(g_i_ix_disp - 10000) / 10` mA |
| `biquad_design.m` | MATLAB script for Biquad coefficient design |

---

## Architecture

### ADC1 Resource Split

```
ADC1:
  SEQ_A (CH0-CH3): BEMF        -- SCMP0 -> AOS_ADC1_0 -> DMA1 (EOCA)
  SEQ_B (CH5-CH7): Current     -- SCMP0 -> AOS_ADC1_0 -> EOCB ISR
```

Both sequences share the **same trigger** (TMR4_3 SCMP0 at PWM counter peak, 50 kHz).
SEQ_A converts first, then SEQ_B; each generates its own end-of-conversion signal
(EOCA -> DMA for BEMF; EOCB -> interrupt for current).

### Trigger Chain

```
TMR4_3 counter peak (center-aligned PWM, 50 kHz)
  └─ EVT CH_UH compare match
       └─ SCMP0 event
            └─ AOS_ADC1_0 (ADC1_TRGSEL0)
                 ├─ ADC1 SEQ_A (BEMF, EVT0) -> EOCA -> DMA1 CH0/1/2/3
                 └─ ADC1 SEQ_B (Current, EVT0) -> EOCB -> ISR (INT116)
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
| Hall U/V/W | INT008/009/010 | `DDL_IRQ_PRIO_02` (2) | Highest -- commutation timing is critical |
| Current ADC1 EOCB | INT116 | `DDL_IRQ_PRIO_03` (3) | Mid -- overcurrent needs fast response |
| BEMF ADC1 DMA | INT116 (DMA BTC) | `DDL_IRQ_PRIO_03` (3) | Same level, independent ADC resources |
| Timer0 1 ms tick | INT006/007 | `DDL_IRQ_PRIO_04` (4) | Lowest -- soft real-time |

> **Fix**: Timer0_Unit1 and Timer0_Unit2 previously used `1UL` (equivalent to `DDL_IRQ_PRIO_01`),
> which was **higher** than Hall. Changed to `DDL_IRQ_PRIO_04`.

---

## Initialization

Called from `main()`:

```c
Hardware_Init();       // Clocks, GPIO, AOS_Init (Timer0 -> ADC1)
App_Comm_Init();       // RS485 + Modbus
CommRunner_Init();     // TMR4 PWM started (50 kHz), STOP mode
Bemf_Init();           // ADC1_SEQ_A (CH0-3), DMA, SCMP0 trigger
I_Init();              // ADC1_SEQ_B (CH5-7), EOCB ISR
I_Calibrate();         // 500 ms blocking zero-offset calibration
EventBus_Enable();     // Start event dispatching
while (1) { ... }
```

### `I_Init()` steps

1. **`I_AdcConfig()`** -- Sets PA5/PA6/PA7 to analog mode (`PIN_ATTR_ANALOG`).
   Enables ADC1_CH5/CH6/CH7 on SEQ_B. Does **not** call `ADC_Init()` --
   ADC1 mode was already configured by BEMF.

2. **`I_TriggerConfig()`** -- Configures ADC1 SEQ_B hardware trigger:
   ```c
   ADC_TriggerConfig(CM_ADC1, ADC_SEQ_B, ADC_HARDTRIG_EVT0);
   ADC_TriggerCmd(CM_ADC1, ADC_SEQ_B, ENABLE);
   ```
   EVT0 maps to AOS_ADC1_0, the same SCMP0 event used by BEMF.

3. **`I_IrqConfig()`** -- Registers EOCB callback via `INTC_IrqSignIn`:
   ```c
   INT_SRC_ADC1_EOCB (449) -> INT116_IRQn, priority DDL_IRQ_PRIO_03
   ADC_IntCmd(CM_ADC1, ADC_INT_EOCB, ENABLE);
   ```

---

## Zero-Offset Calibration

### Problem

Each current sensor has slightly different zero-bias (resistor tolerance, VREF drift).
Without calibration, three phases show different non-zero readings at idle,
and the three-phase sum deviates from zero.

### `I_Calibrate()` -- Blocking 500 ms

1. Forces all three PWM channels to **OFF** (`TMR4_MODE_OFF`):
   ```c
   TMR4_PWM_SetChannelMode(U, TMR4_MODE_OFF, 0.0f);
   TMR4_PWM_SetChannelMode(V, TMR4_MODE_OFF, 0.0f);
   TMR4_PWM_SetChannelMode(W, TMR4_MODE_OFF, 0.0f);
   ```
   This ensures all FETs are off -- motor windings are truly floating, current is zero.

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
   the ISR fires ~25,000 times (50 kHz x 0.5 s).

4. Sets `g_i_calib_state = 2`. Computes per-phase average:
   ```c
   g_i_calib_zero_u = s_i32CalibSumU / s_i32CalibCnt;
   g_i_calib_zero_v = s_i32CalibSumV / s_i32CalibCnt;
   g_i_calib_zero_w = s_i32CalibSumW / s_i32CalibCnt;
   ```

5. From this point, the ISR uses the calibrated zero references:
   ```c
   uint16_t u16ZeroU = (g_i_calib_state == 2) ? g_i_calib_zero_u : I_ADC_ZERO;
   int16_t IU_mA = I_ADC_TO_MA_REF(u16IU, u16ZeroU);
   ```

### Current Conversion

```c
// Formula: I_mA = (ADC_raw - zero_ref) x 3300 x 1000 / (4095 x 528)
// Fixed-point: I_mA = (diff x 391) >> 8
//   where 391 = 3300 x 256 / (4095 x 528) x 1000
//   i.e. 1.527 mA/count x 256 = 391
#define I_MA_PER_ADC  391
#define I_MA_SHIFT    8

#define I_ADC_TO_MA_REF(raw, zero) \
    ((int16_t)(((int32_t)((raw) - (zero)) * I_MA_PER_ADC) >> I_MA_SHIFT))
```

---

## Biquad Low-Pass Filter

Replaced the original EMA (1st-order IIR, fc~250Hz) with a 2nd-order Butterworth IIR.

### Why 2nd-order Butterworth

PWM switching noise (50 kHz) passes through a 1st-order EMA with ~46 dB attenuation,
leaving residual mV-level ripple. A 2nd-order Butterworth at fc=200Hz provides:

- **-40 dB/dec** rolloff (vs -20 dB/dec for EMA)
- PWM at 25kHz: ~84 dB attenuation (vs ~46 dB), i.e. **40000x vs 200x**
- Maximally flat passband -- motor fundamental current (tens to hundreds of Hz) is
  completely undistorted
- Step response settles to 95% in ~12 ms (similar to the old EMA)

### Coefficients (MATLAB: butter(2, 200/25000, 'low'))

```c
#define BIQUAD_B0  0.0001551484f
#define BIQUAD_B1  0.0003102968f
#define BIQUAD_B2  0.0001551484f
#define BIQUAD_A1  (-1.9644605802f)
#define BIQUAD_A2  0.9650811739f
```

### Difference Equation (Direct Form I)

```
y[n] = b0.x[n] + b1.x[n-1] + b2.x[n-2] - a1.y[n-1] - a2.y[n-2]
```

On the first sample, all state variables (x[n-1], x[n-2], y[n-1], y[n-2]) are
seeded with the current value for immediate lock-in (no ramp-up).

### Frequency Response (key points)

| Frequency | Attenuation | Phase |
|-----------|------------|-------|
| 10 Hz | -0.00 dB | -4.1 deg |
| 50 Hz | -0.02 dB | -20.7 deg |
| 100 Hz | -0.26 dB | -43.3 deg |
| 150 Hz | -1.19 dB | -67.6 deg |
| **200 Hz** | **-3.01 dB** | **-90.0 deg** |
| 250 Hz | -5.37 dB | -107.7 deg |
| 500 Hz | -16.03 dB | -146.1 deg |
| 1 kHz | -27.99 dB | -163.6 deg |
| 5 kHz | -56.50 dB | -176.9 deg |

See `biquad_design.m` for the full design and `I滤波.md` for detailed filter documentation.

---

## J-Scope Variables

| Variable | Type | Meaning |
|----------|------|---------|
| `g_i_iu_raw` | `uint16_t` | IU raw ADC (0-4095) |
| `g_i_iv_raw` | `uint16_t` | IV raw ADC |
| `g_i_iw_raw` | `uint16_t` | IW raw ADC |
| `g_i_iu_ma` | `int16_t` | IU instantaneous current (mA, signed, no filter) |
| `g_i_iv_ma` | `int16_t` | IV instantaneous current |
| `g_i_iw_ma` | `int16_t` | IW instantaneous current |
| `g_i_iu_filt` | `int32_t` | IU Biquad-filtered current (Q8: /256 = mA) |
| `g_i_iv_filt` | `int32_t` | IV Biquad-filtered current (Q8) |
| `g_i_iw_filt` | `int32_t` | IW Biquad-filtered current (Q8) |
| `g_i_iu_disp` | `uint16_t` | IU filtered mA x 10 + 10000 (always >= 0, J-Scope safe) |
| `g_i_iv_disp` | `uint16_t` | IV filtered mA x 10 + 10000 |
| `g_i_iw_disp` | `uint16_t` | IW filtered mA x 10 + 10000 |
| `g_i_uvw_raw` | `int32_t` | Raw sum IU+IV+IW (should ~ 3 x 2048 = 6144) |
| `g_i_uvw_ma` | `int32_t` | Current sum IU+IV+IW in mA (should ~ 0) |
| `g_i_calib_state` | `uint8_t` | 0=idle, 1=calibrating, 2=done |
| `g_i_calib_zero_u` | `uint16_t` | Calibrated zero reference for IU (ADC raw) |
| `g_i_calib_zero_v` | `uint16_t` | Calibrated zero reference for IV |
| `g_i_calib_zero_w` | `uint16_t` | Calibrated zero reference for IW |
| `g_i_sample_cnt` | `uint32_t` | Cumulative sample count (confirms ISR is running) |
| `g_i_running` | `uint8_t` | Module status (0=stopped, 1=running) |

### Recommended J-Scope Display

- **Waveform view**: `g_i_iu_disp`, `g_i_iv_disp`, `g_i_iw_disp` in the same sub-plot
  - Y-axis: 0-20000 (10000 = 0 mA; 20000 = +1000 mA; 0 = -1000 mA)
  - Expression: `(g_i_iu_disp - 10000) / 10` for real mA
- **Precision value**: `g_i_iu_filt`, `g_i_iv_filt`, `g_i_iw_filt`
  - Expression: `g_i_iu_filt / 256` for real mA
- **Raw check**: `g_i_iu_raw`, `g_i_iv_raw`, `g_i_iw_raw`
  - Y-axis: 0-4095
- **Unfiltered check**: `g_i_iu_ma`, `g_i_iv_ma`, `g_i_iw_ma`
  - Compare with filtered values to see Biquad effect

### Interpreting Values

| Raw ADC | Instant mA | Filtered mA | Meaning |
|---------|-----------|-------------|---------|
| ~calib_zero | ~0 | ~0 | Zero current (floating phase or idle) |
| > calib_zero | > 0 | > 0 | Current flowing INTO motor phase |
| < calib_zero | < 0 | < 0 | Current flowing OUT of motor phase |
| `g_i_uvw_ma` ~ 0 | -- | -- | Three-phase balance OK |
| `g_i_uvw_ma` != 0 | -- | -- | Offset error or sensor fault |

---

## Data Flow Summary

```
Every PWM cycle (20 us, 50 kHz):

1. TMR4_3 counter reaches peak
2. SCMP0 event fires
3. AOS routes to ADC1_TRGSEL0
4. ADC1 SEQ_A scans CH0->CH1->CH2->CH3 (BEMF, ~1 us)
5. ADC1 SEQ_B scans CH5(IU)->CH6(IV)->CH7(IW) (Current, ~0.8 us)
6. ADC1 EOCA -> DMA1 BTC -> Bemf_DataCallback (BEMF data ready)
7. ADC1 EOCB -> I_IrqCallback:
   a. Read DR5, DR6, DR7
   b. Subtract calibrated zero reference
   c. Convert to mA (integer arithmetic, I_ADC_TO_MA_REF)
   d. Apply 2nd-order Butterworth Biquad filter (float32, FPU)
   e. Output: g_i_iu_filt (Q8) + g_i_iu_disp (x10 + 10000)
   f. Update J-Scope globals
   g. Invoke user callback (if registered)
```

### VOFA+ Data Path

```
main loop @ 1kHz (1ms):
  cur[0] = (g_i_iu_disp - 10000) / 10   // recover mA
  Usart3_Vofa_SendScaled(cur, 3, 0.001) // mA -> A via JustFloat DMA
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
   The three zero_ref values should cluster near 2048 (typically 2030-2065).
   A value far outside this range indicates a hardware issue (wiring, sensor fault,
   or pin conflict).
4. After calibration, verify `g_i_uvw_ma ~ 0` at idle in J-Scope.

---

## Known Limitations

1. **Common gain**: All three phases use the same 528 mV/A slope.
   Individual sensor gain differences are not calibrated.
   Result: small residual imbalance in `g_i_uvw_ma`.

2. **PWM-coupling offset**: Even with FETs OFF, capacitive coupling from
   switching TMR4 pins to adjacent ADC traces may introduce sub-count bias.
   The 500 ms averaging mitigates this.

3. **VREF tolerance**: ADC reference is VDD (nominally 3.3 V), not a precision
   reference. VDD variation directly scales ADC readings.

4. **No overcurrent protection yet**: The ISR only observes. No fault action
   is triggered based on current thresholds.
