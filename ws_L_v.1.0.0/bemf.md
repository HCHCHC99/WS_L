# BEMF Detection Module — Design & Known Issues

## Overview

BEMF (Back Electromotive Force) detection for BLDC sensorless control on HC32F460.

- **File**: `Adp/Bemf.c`, `Adp/Bemf.h`
- **Role**: Observer-only — collects 4-channel ADC data, no commutation logic
- **Architecture**: Independent instance framework (does NOT depend on `Adc.c/.h`)
- **Depends on**: `Dma.c/.h`, `Aos.c/.h`, `hc32_ll_tmr4.h`, `tmr4_pwm.c/.h`

## Pin Mapping

| Signal   | Pin | ADC Channel | DMA Channel | DMA Buffer |
|----------|-----|-------------|-------------|-------------|
| M_BEMF (Neutral) | PA0 | ADC1_CH0 | DMA1 CH0 | 8 × uint16_t |
| U_BEMF (Phase U) | PA1 | ADC1_CH1 | DMA1 CH1 | 8 × uint16_t |
| V_BEMF (Phase V) | PA2 | ADC1_CH2 | DMA1 CH2 | 8 × uint16_t |
| W_BEMF (Phase W) | PA3 | ADC1_CH3 | DMA1 CH3 | 8 × uint16_t |

## Data Flow

```
TMR4_3 PWM (50kHz, center-aligned triangle, CM_TMR4_3)
  └─→ EVT CH_UH (SCMP0) fires at counter peak (every 20µs)
       └─→ AOS (EVT_SRC_TMR4_3_SCMP0) → ADC1_SEQ_A hardware trigger (EVT0)
            └─→ ADC1 single-shot scan: CH0→CH1→CH2→CH3
                 └─→ ADC1 EOCA event
                      └─→ AOS → DMA1 CH0/1/2/3 (parallel transfer, repeat mode)
                           └─→ DMA BTC interrupt @ DMA1 CH0 → Bemf_DataCallback()
                                └─→ Updates global JScope variables
```

## AOS Routing

Managed by `AOS_InitForBemf()` in `Aos.c`:

| Source | Target | Purpose |
|--------|--------|---------|
| `EVT_SRC_TMR4_3_SCMP0` | `AOS_ADC1_0` | PWM peak → ADC trigger |
| `EVT_SRC_ADC1_EOCA` | `AOS_DMA1_0` | ADC done → DMA CH0 |
| `EVT_SRC_ADC1_EOCA` | `AOS_DMA1_1` | ADC done → DMA CH1 |
| `EVT_SRC_ADC1_EOCA` | `AOS_DMA1_2` | ADC done → DMA CH2 |
| `EVT_SRC_ADC1_EOCA` | `AOS_DMA1_3` | ADC done → DMA CH3 |

**Important**: The legacy `AOS_Init()` (called from `Hardware_Init()`) still connects `Timer0_CMP_B → ADC1`. Since both use `AOS_ADC1_0`, `AOS_InitForBemf()` **overwrites** this connection. If you later re-enable PA6 ADC via `Adc.c`, this will break BEMF timing.

## JScope Global Variables (defined in Bemf.c, extern in Bemf.h)

| Variable | Type | Description |
|----------|------|-------------|
| `g_bemf_m_raw` | `volatile uint16_t` | Neutral point raw ADC (0–4095) |
| `g_bemf_u_raw` | `volatile uint16_t` | Phase U raw ADC |
| `g_bemf_v_raw` | `volatile uint16_t` | Phase V raw ADC |
| `g_bemf_w_raw` | `volatile uint16_t` | Phase W raw ADC |
| `g_bemf_u_mv` | `volatile int16_t` | Phase U BEMF voltage (mV, relative to neutral) |
| `g_bemf_v_mv` | `volatile int16_t` | Phase V BEMF voltage |
| `g_bemf_w_mv` | `volatile int16_t` | Phase W BEMF voltage |
| `g_bemf_m_mv` | `volatile uint16_t` | Neutral point absolute voltage (mV) |
| `g_bemf_sample_cnt` | `volatile uint32_t` | Cumulative sample count |
| `g_bemf_running` | `volatile uint8_t` | Module status (0=stopped, 1=running) |

## Initialization Order (in main.c)

```c
Hardware_Init();         // Clocks, GPIO, AOS_Init (Timer0→ADC1), tick timers
App_Comm_Init(&comm_cfg); // RS485 comms
CommRunner_Init(&runner_cfg); // TMR4_PWM_Config + TMR4_PWM_StartOutput (PWM running!)
Bemf_Init();             // AOS routing + ADC1 config + DMA config + TMR4 EVT config
EventBus_Enable();
// Main loop: periodic BEMF print every 500ms
```

## Dma.c Modification

`Dma_Init()` was modified to support **incremental initialization**:
- Original: early-returned if `s_bDmaInitialized`, skipping new DMA instances
- Fixed: iterates all instances, skips only those already initialized (`u8Initialized`)
- IRQ mapping table initialized only on first call

## Known Issues

### 1. BEMF reads driven phase voltage when motor is stopped
- **Cause**: PWM is still switching MOSFETs even at 0 RPM. The ADC reads the driven PWM voltage on each phase, not the actual back-EMF.
- **Symptom**: `g_bemf_*_mv` values fluctuate with large amplitude (0–3300mV) even when motor is stationary.
- **Solution**: Only read the **floating phase** during six-step commutation:
  - Step 0 (UH+WL): V is floating → read `g_bemf_v_mv`
  - Step 1 (UH+VL): W is floating → read `g_bemf_w_mv`
  - Step 2 (WH+VL): U is floating → read `g_bemf_u_mv`
  - Step 3 (WH+UL): V is floating → read `g_bemf_v_mv`
  - Step 4 (VH+UL): W is floating → read `g_bemf_w_mv`
  - Step 5 (VH+WL): U is floating → read `g_bemf_u_mv`
- **Future API needed**: `Bemf_GetFloatingPhaseBemf(uint8_t u8Step)` that auto-selects the correct channel.

### 2. No zero-crossing detection yet
- The module is **observer-only**. BEMF zero-crossing detection and commutation timing logic is not implemented.
- This is intentional pending validation of BEMF signal quality.

### 3. TMR4 EVT channel shares UH PWM channel
- `Bemf_Tmr4EvtConfig()` configures EVT on `TMR4_EVT_CH_UH` (channel 0).
- This channel is also used for U-phase high-side PWM by `tmr4_pwm.c`.
- EVT and PWM use separate register sets (SCSR/SCMR/SCCR vs OCMR/OCCR), so no known conflict.
- But if PWM is later reconfigured on the fly, EVT settings may need re-applying.

### 4. Sampling at PWM peak may not be ideal for BEMF
- Current trigger: TMR4 counter peak (= center of PWM cycle).
- For BEMF zero-crossing detection, the ideal sampling point may be at the **PWM off-time** (when the floating phase has fully settled).
- May need to adjust SCMP compare value or add delay/sample-window logic.

### 5. PA0–PA3 potential pin conflicts
- PA0–PA3 are also ADC1_IN0–IN3 default pins.
- PA2 (V_BEMF) may conflict if USART2 or other alternate functions are enabled on PA2.
- Verify pin multiplexing if using other peripherals.

### 6. Voltage calculation ignores resistor divider ratio (PENDING)
- **Problem**: The current voltage calculation in `Bemf_DataCallback()` and `Bemf_GetBemfVoltage()`:
  ```c
  g_bemf_u_mv = (int16_t)(((int32_t)s_stcBemfData.i16BemfU * 3300) / 4096);
  ```
  This converts raw ADC difference to mV at the **ADC pin**, NOT the actual phase voltage.
  The real circuit has a resistor divider between each motor phase and the ADC input pin
  (e.g., R1 upper, R2 lower to GND), plus a similar divider for the neutral point.
  Actual phase voltage = ADC_pin_voltage × (R1 + R2) / R2.
- **Information needed** (from schematic):
  1. Resistor values for U/V/W phase dividers (R1 = ?, R2 = ?)
  2. Neutral point (M_BEMF) divider circuit — star-connected from 3 phases, or independent Vbus/2?
  3. Vbus voltage (motor supply voltage)
  4. Any filter capacitors across R2 (affects settling time)
- **Impact**: Without the correct divider ratio, `g_bemf_*_mv` readings are off by the divider
  factor (e.g., if divider is 10:1, real BEMF is ~10× larger than displayed).
  High-frequency PWM noise also passes through unfiltered, causing large value swings.
- **Fix plan**: Once component values are known, add divider ratio macros to `Bemf.h` and
  correct the mV conversion formula. May also need to adjust ADC sampling timing to account
  for filter capacitor settling.

### 7. Fixed 8-sample DMA buffer
- `BEMF_DMA_BUFFER_SIZE = 8` means BTC interrupt fires every 8 × 20µs = 160µs (6.25kHz).
- This gives ~2.6 BTC interrupts per electrical step at 400Hz (6000 RPM, 4 pole pairs).
- May be insufficient for accurate zero-crossing detection at high speed. Consider reducing buffer size to 4 or 2.

## Files Modified/Created

| File | Action | Purpose |
|------|--------|---------|
| `Adp/Bemf.c` | Created | BEMF module implementation (~370 lines) |
| `Adp/Bemf.h` | Created | BEMF module header |
| `Adp/Aos.c` | Modified | Added `AOS_InitForBemf()` |
| `Adp/Aos.h` | Modified | Added TMR4 SCMP event macros + function declaration |
| `Adp/Dma.c` | Modified | `Dma_Init()` now supports incremental initialization |
| `template/source/main.c` | Modified | Added `#include "Bemf.h"`, `Bemf_Init()` call, periodic print |
