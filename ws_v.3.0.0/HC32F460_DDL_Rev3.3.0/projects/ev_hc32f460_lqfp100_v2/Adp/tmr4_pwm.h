#ifndef __TMR4_PWM_H__
#define __TMR4_PWM_H__

#include "main.h"
#include <stdbool.h>

/*=============================================================================
 * TMR4 unit 3 3-channel PWM on:
 *   PB9=TIM4_3_OUH(func2)  PB8=TIM4_3_OUL(func2)
 *   PB7=TIM4_3_OVH(func2)  PB6=TIM4_3_OVL(func2)
 *   PB5=TIM4_3_OWH(func2)  PB4=TIM4_3_OWL(func2)
 *=============================================================================*/

/* Duty cycle range: 0-10000 (representing 0.00% - 100.00%) */
#define TMR4_PWM_DUTY_MAX  10000U

/* Output type */
typedef enum {
    TMR4_OUTPUT_COMPLEMENTARY = 0,  /* Complementary PWM + hardware dead-time */
    TMR4_OUTPUT_SYNC          = 1,  /* Same signal on both outputs (external driver handles dead-time) */
} tmr4_output_type_t;

/* PWM channel pair (U / V / W) — named to avoid collision with DDL TMR4_PWM_CH_* macros */
typedef enum {
    TMR4_CHANNEL_U = 0,
    TMR4_CHANNEL_V = 1,
    TMR4_CHANNEL_W = 2,
} tmr4_pwm_channel_t;

/* Configuration structure for TMR4 PWM (per-channel output type) */
typedef struct {
    tmr4_output_type_t output_type_u;  /* U channel: Complementary or sync */
    tmr4_output_type_t output_type_v;  /* V channel: Complementary or sync */
    tmr4_output_type_t output_type_w;  /* W channel: Complementary or sync */
    uint16_t           freq_hz;        /* PWM frequency in Hz */
    uint16_t           dead_time_ns;   /* Dead-time in ns (only used by COMPLEMENTARY channels) */
    bool               active_high;    /* true = active high, false = active low */
} tmr4_pwm_config_t;

/* Initialize TMR4, GPIO(all 6 pins), counter, OC channels, PWM mode for U/V/W */
void TMR4_PWM_Config(const tmr4_pwm_config_t *pConfig);

/* Enable PWM outputs (start counter) */
void TMR4_PWM_StartOutput(void);

/* Disable PWM outputs (stop counter + disable all OC channels) */
void TMR4_PWM_StopOutput(void);

/* Immediate all-off (emergency stop) */
void TMR4_PWM_EmergencyStop(void);

/* Set PWM frequency (Hz) — updates counter period. All duties must be re-applied after. */
void TMR4_PWM_SetFrequency(uint16_t freq_hz);

/* Set duty cycle for a specific channel: 0 = 0.00%, 10000 = 100.00% (legacy) */
void TMR4_PWM_SetDuty(tmr4_pwm_channel_t channel, uint16_t u16Duty);

/* Set duty cycle as float percentage: 0.0f = 0%, 100.0f = 100% */
void TMR4_PWM_SetDutyFloat(tmr4_pwm_channel_t channel, float duty_pct);

/* Enable or disable a specific PWM channel pair (both H and L sides) */
void TMR4_PWM_ChannelCmd(tmr4_pwm_channel_t channel, bool enable);

/* Enable or disable a single PWM pin (high or low side). */
void TMR4_PWM_PinCmd(tmr4_pwm_channel_t channel, bool high_side, bool enable);

/* Set duty cycle for a single pin (high or low side of a channel).
 * Writes only that pin's OCCR — the other pin is unchanged. */
void TMR4_PWM_PinSetDuty(tmr4_pwm_channel_t channel, bool high_side, uint16_t u16Duty);

/* Set a pin's output level when OC is disabled.
 * level: false=LOW, true=HIGH. Default after Config() is LOW.
 * Use for OFF phase: set H=HIGH, L=LOW → disable both → H,L → interlock. */
void TMR4_PWM_PinSetInvalidLevel(tmr4_pwm_channel_t channel, bool high_side, bool level_high);

/* Channel output mode — 6288T-MNS pre-driver
 *
 * 6288T-MNS truth table (per phase):
 *   HIN=L, LIN=L → HO=L, LO=L  (both OFF / shoot-through protection)
 *   HIN=L, LIN=H → HO=L, LO=H  (low-side ON)
 *   HIN=H, LIN=L → HO=H, LO=L  (high-side ON)
 *   HIN=H, LIN=H → HO=L, LO=L  (both OFF / shoot-through protection)
 *
 * OFF:       H=L=LOW  → both OFF
 * HIGH_SIDE: H=PWM, L=LOW → PWM ON: high-side ON, PWM OFF: both OFF
 * LOW_SIDE:  H=LOW, L=HIGH → low-side ON (continuous) */
typedef enum {
    TMR4_MODE_OFF       = 0,  /* H & L both LOW → 6288T: both FETs OFF */
    TMR4_MODE_HIGH_SIDE = 1,  /* H=PWM, L=LOW → 6288T: high-side FET chopping */
    TMR4_MODE_LOW_SIDE  = 2,  /* H=LOW, L=HIGH → 6288T: low-side FET ON */
} tmr4_channel_mode_t;

/* Switch a channel's output mode at runtime (for six-step commutation).
 * Re-initializes OC + PWM for the channel. duty_pct = 0.0f~100.0f. */
void TMR4_PWM_SetChannelMode(tmr4_pwm_channel_t channel, tmr4_channel_mode_t mode, float duty_pct);

#endif /* __TMR4_PWM_H__ */
