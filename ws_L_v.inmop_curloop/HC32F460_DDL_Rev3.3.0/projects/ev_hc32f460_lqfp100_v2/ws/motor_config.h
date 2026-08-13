/**
 *******************************************************************************
 * @file  motor_config.h
 * @brief Motor control frequency configuration (single source of truth)
 *******************************************************************************
 */

#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

/* ============================================================================
 * Motor control frequency master config: change this one macro.
 *
 *   MOTOR_PWM_FREQ_HZ = PWM switching frequency.
 *   INMOP-style double update (branch inmop_cur_loop):
 *     PWM = 10kHz, ADC/current-loop read = 2x = 20kHz
 *     (TMR4 SCMP0 @ PEAK + SCMP2 @ VALLEY both trigger ADC1 SEQ_B EOCB ISR).
 *
 *   Other values 8kHz~100kHz are possible (limited by ADC conversion time /
 *   ISR load). For 1x sampling per PWM period, keep a single peak trigger
 *   (I_ADC_HARDTRIG = EVT0 only).
 * ==========================================================================*/
#define MOTOR_PWM_FREQ_HZ   10000u

/* Hall sensor enable: 0 = hall disabled (PA8/9/10 freed for encoder ABZ) */
#define MOTOR_HALL_ENABLE   0

/* ============================================================================
 * 环拓扑配置（改这里切换控制结构）
 * ==========================================================================*/
/* 位置环（预留，默认关 - 需位置反馈如编码器） */
#define MOTOR_LOOP_POSITION_ENABLE   0
#define POS_REF_EXTERNAL             0
#define MOTOR_POS_REF_SRC            POS_REF_EXTERNAL

/* 速度环 */
#define MOTOR_LOOP_SPEED_ENABLE      1
#define SPD_REF_TARGET_RPM           0
#define SPD_REF_FROM_POS             1
#define MOTOR_SPD_REF_SRC            SPD_REF_TARGET_RPM

/* 电流环 */
#define MOTOR_LOOP_CURRENT_ENABLE    1
#define CUR_REF_FIXED                0
#define CUR_REF_FROM_SPEED           1
#define MOTOR_CUR_REF_SRC            CUR_REF_FROM_SPEED

/* 占空比来源 */
#define DUTY_DIRECT                  0
#define DUTY_FROM_CURRENT            1
#define MOTOR_DUTY_SRC               DUTY_FROM_CURRENT

/* ============================================================================
 * Motor electrical parameters (3505-KV650 gimbal/outrunner motor)
 * ==========================================================================*/
#define FOC_MOTOR_RS_OHM            0.1f     /* phase resistance (ohm) */
#define FOC_MOTOR_LS_UH             42.3f    /* phase inductance (uH) */
#define FOC_MOTOR_FLUX_VS           0.00084f /* PM flux linkage (V*s, peak) */
#define FOC_MOTOR_KV_RPM_V          650u     /* KV (rpm/V): 12V x 650 = 7800 rpm */
#define FOC_MOTOR_RATED_CURRENT_A   17.0f    /* max phase current (A) */
#define FOC_MOTOR_RATED_TORQUE_NM   0.2f     /* rated torque (Nm) */
#define FOC_MOTOR_MAX_SPEED_RPM     7800u    /* max speed @ 12V */
/* Derived (documentation only):
 *   Kt   = 1.5 * P * flux = 0.0126 Nm/A  (0.2Nm needs ~15.9A, matches 17A rating)
 *   BEMF phase peak = flux * 2*pi*f_elec = ~26mV @ 5Hz (30rpm) -> tiny at I-F speed
 *   electrical time constant tau = L/R = 423us (R does not include driver/wires)
 */

/* ============================================================================
 * FOC parameters (comm_mode 21 = FOC open-loop)
 * ==========================================================================*/
#define MOTOR_FOC_ENABLE        1          /* 1 = compile/enable FOC mode 21 */
#define FOC_POLE_PAIRS          10         /* motor pole pairs */
#define FOC_VBUS_V              12.0f      /* DC bus voltage (V): motor rated 12V (3505-KV650) */
#define FOC_OPENLOOP_FREQ_HZ    5.0f       /* default open-loop electrical freq (Hz) */
#define FOC_OPENLOOP_VOLT_V     0.4f       /* open-loop voltage (V): standstill ~2.5-4A (R=0.1 plus driver/wire); BEMF @5Hz is only ~26mV so the rotor is dragged easily */
#define FOC_OPENLOOP_VOLT_MAX  1.5f       /* HARD CAP: open-loop phase voltage never exceeds this (overheat protection) */
#define FOC_ISR_HZ              20000      /* FOC ISR rate (Hz): 10k PWM x double trigger = 20k */
#define FOC_DEADTIME_NS         500u       /* complementary PWM dead-time (ns) */
/* ============================================================================
 * FOC current-loop parameters (comm_mode 22 = FOC current loop)
 * ==========================================================================*/
#define FOC_IQ_REF_MA        2000        /* Iq target (mA) during start: Kt=12.6mNm/A -> 2A = 25mNm pull-in; vq need ~0.35V < 1.0V clamp */
#define FOC_IQ_RAMP_MA_S     500         /* Iq soft-start ramp rate (mA/s): reaches 800mA hold in ~1.6s and 2A in ~4s */

/* I-F start (mode 22): current-controlled startup with synthetic angle */
#define FOC_IF_HOLD_IQ_MA    800        /* hold theta=0 until Iq ref reaches this (mA): ~10mNm preload to lock the rotor first */
#define FOC_IF_HOLD_MAX_MS   5000       /* max hold time (ms) before forcing the ramp regardless */
#define FOC_IF_FREQ_RAMP_HZ_S   1.0f     /* synthetic frequency ramp (Hz/s) - slower = rotor can follow */
#define FOC_IF_SYNC_MIN_HZ      2.0f     /* min frequency before sync detection */
#define FOC_IF_SYNC_WIN_CNT     2000u    /* sync window length (samples @20k, default 2000=100ms; runtime g_foc_if_sync_win_cnt Watch tunable) */
#define FOC_IF_SYNC_BAND_RAD    0.10f    /* allowed angle-diff band per window (rad, relative to hold lock offset; runtime g_foc_if_sync_band_rad Watch tunable) */
#define FOC_IF_SYNC_GOOD_WINS   2u       /* consecutive good windows required (runtime g_foc_if_sync_good_wins Watch tunable) */
#define FOC_IF_TIMEOUT_MS       15000u   /* start timeout -> fault code 2 */
/* Align calibration (comm_mode 23) - INMOP-style two-step (beta -> alpha) FIXED voltage */
#define FOC_ALIGN_VOLT_V     0.4f       /* fixed align voltage (V): 0.4V/(R=0.1 + loop R) ~ 2.5-4A -> ~31mNm lock torque (< OC 5.5A), Watch tunable */
#define FOC_ALIGN_BETA_MS    1000       /* step1: hold on beta axis (ms), INMOP-style */
#define FOC_ALIGN_STABLE_MS  300        /* encoder-stable window to declare lock (ms) */
#define FOC_ALIGN_TIMEOUT_MS 3000       /* step2 (alpha) timeout before fault code 3 */
#define FOC_ALIGN_HOLD_MS    2000       /* hold lock before releasing output (ms) - long enough to read id/iq */
#define FOC_OC_LIMIT_A       5.5f        /* over-current trip default (A, per phase): 5.5A -> ~69mNm; limited by +-10A sensor (motor max 17A); runtime: g_foc_oc_limit_a */
#define FOC_CUR_SIGN         -1          /* current sign correction: -1 confirmed by mode-23 align (id must be positive) */
#define FOC_ENC_DIR         1           /* encoder direction for electrical angle (+1/-1), Watch tunable via g_foc_enc_dir */
#define FOC_PI_KP            0.10f       /* current PI Kp (V/A): kp = 2*pi*fc*L -> fc ~ 375Hz at L=42.3uH */
#define FOC_PI_KI            240.0f      /* current PI Ki (1/s): pole-zero cancel ki = kp*R/L (tau = 423us) */
#define FOC_PI_UMAX_V        1.0f        /* current PI output clamp (V): 1.0V covers ~6A standstill worst case; normal op ~0.3V at 2A */

#define FOC_VMAX_V            1.0f       /* current-loop max voltage magnitude (V), Watch tunable: headroom through 3A */
#define FOC_VRAMP_V_S         0.5f       /* voltage envelope ramp rate (V/s): reaches 1.0V in 2s, 0.5V in 1s */
#define FOC_CUR_FB_ALPHA      0.3f       /* EMA weight on id/iq feedback (1.0 = off) */
#endif /* __MOTOR_CONFIG_H__ */
