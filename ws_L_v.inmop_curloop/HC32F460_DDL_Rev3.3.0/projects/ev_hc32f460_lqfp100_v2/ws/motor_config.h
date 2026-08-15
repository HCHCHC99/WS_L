/**
 *******************************************************************************
 * @file  motor_config.h
 * @brief 电机控制参数主配置（唯一配置来源）
 *******************************************************************************
 */

#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

/* ============================================================================
 * 电机控制频率主配置：改这里一个宏即可。
 *
 *   MOTOR_PWM_FREQ_HZ = PWM 开关频率。
 *   INMOP 风格双更新（分支 inmop_cur_loop）：
 *     PWM = 10kHz，ADC/电流环采样 = 2x = 20kHz
 *     （TMR4 SCMP0 @ PEAK + SCMP2 @ VALLEY 都触发 ADC1 SEQ_B EOCB ISR）。
 *
 *   其它 8kHz~100kHz 也可用（受 ADC 转换时间 / ISR 负载限制）。
 *   若每个 PWM 周期只采一次，则保留单个峰触发（I_ADC_HARDTRIG = EVT0）。
 * ==========================================================================*/
#define MOTOR_PWM_FREQ_HZ   10000u

/* 霍尔使能：0 = 关闭霍尔（PA8/9/10 释放给编码器 ABZ 用） */
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
 * 电机电气参数（3505-KV650 云台/外转子电机）
 * ==========================================================================*/
#define FOC_MOTOR_RS_OHM            0.1f     /* 相电阻（欧姆） */
#define FOC_MOTOR_LS_UH             42.3f    /* 相电感（uH） */
#define FOC_MOTOR_FLUX_VS           0.00084f /* 永磁磁链（V*s，峰值） */
#define FOC_MOTOR_KV_RPM_V          650u     /* KV（rpm/V）：12V x 650 = 7800 rpm */
#define FOC_MOTOR_RATED_CURRENT_A   17.0f    /* 最大相电流（A） */
#define FOC_MOTOR_RATED_TORQUE_NM   0.2f     /* 额定转矩（Nm） */
#define FOC_MOTOR_MAX_SPEED_RPM     7800u    /* 12V 下最大转速 */
/* 派生量（仅文档说明）：
 *   Kt   = 1.5 * P * flux = 0.0126 Nm/A（0.2Nm 需要 ~15.9A，与 17A 额定吻合）
 *   BEMF 相峰值 = flux * 2*pi*f_elec = ~26mV @ 5Hz（30rpm）-> I-F 低速时很小
 *   电气时间常数 tau = L/R = 423us（R 不含驱动/线阻）
 */

/* ============================================================================
 * FOC 参数（comm_mode 21 = FOC 开环）
 * ==========================================================================*/
#define MOTOR_FOC_ENABLE        1          /* 1 = 编译/使能 FOC（模式 21） */
#define FOC_POLE_PAIRS          10         /* 电机极对数 */
#define FOC_VBUS_V              12.0f      /* 母线电压（V）：电机额定 12V（3505-KV650） */
#define FOC_OPENLOOP_FREQ_HZ    5.0f       /* 默认开环电频率（Hz） */
#define FOC_OPENLOOP_VOLT_V     0.9f       /* 开环电压（V）：0.4V 拉力不足（转子 -32~-38rpm 打滑）；0.9V 在 5Hz 稳定同步 -30rpm */
#define FOC_OPENLOOP_VOLT_MAX  1.5f       /* 硬上限：开环相电压不得超过此值（过热保护） */
#define FOC_ISR_HZ              20000      /* FOC ISR 频率（Hz）：10k PWM x 双触发 = 20k */
#define FOC_DEADTIME_NS         500u       /* 互补 PWM 死区（ns） */

/* ============================================================================
 * FOC 电流环参数（comm_mode 22 = FOC 电流环）【mode22】
 * ==========================================================================*/
#define FOC_IQ_REF_MA        3000        /* 启动时 Iq 目标（mA）：Kt=12.6mNm/A -> 3A = 38mNm 拉入转矩 */
#define FOC_IQ_RAMP_MA_S     500         /* Iq 软启动斜坡（mA/s）：到 1200mA 约 1.6s，到 3000mA 约 4s */

/* I-F 启动（mode 22）：合成角 + 电流控制的启动过程 【mode22】 */
#define FOC_IF_HOLD_IQ_MA    1200       /* hold：theta=0 直到 Iq 参考达到此值（mA）：800mA 锁不住（hold 期 cnt 漂）；1200mA≈15mNm 预载 */
#define FOC_IF_HOLD_MAX_MS   5000       /* hold 最长等待（ms），超时强制进入爬频 */
#define FOC_IF_FREQ_RAMP_HZ_S   1.0f     /* 合成频率爬升率（Hz/s）——越小转子越容易跟上 */
#define FOC_IF_SYNC_MIN_HZ      2.0f     /* 开始同步检测的最低频率 */
#define FOC_IF_SYNC_WIN_CNT     2000u    /* 同步窗口长度（@20k 采样数，2000=100ms；运行时可 Watch 改 g_foc_if_sync_win_cnt） */
#define FOC_IF_SYNC_BAND_RAD    0.30f    /* 每个窗口允许的角度差带宽（rad，相对 hold 锁存偏移）：0.3 实测交接后不触发过流（运行时可 Watch 改 g_foc_if_sync_band_rad） */
#define FOC_IF_SYNC_GOOD_WINS   2u       /* 连续合格窗口数（运行时可 Watch 改 g_foc_if_sync_good_wins） */
#define FOC_IF_TIMEOUT_MS       15000u   /* 启动超时 -> 故障码 2（I-F 未同步） */

/* 对齐校准（comm_mode 23）：INMOP 风格两段式（beta -> alpha）固定电压 */
#define FOC_ALIGN_VOLT_V     0.4f       /* 固定对齐电压（V）：0.4V/(R=0.1+回路电阻) ~ 2.5-4A -> ~31mNm 锁轴转矩（< OC 5.5A），Watch 可调 */
#define FOC_ALIGN_BETA_MS    1000       /* 第 1 步：beta 轴保持（ms） */
#define FOC_ALIGN_STABLE_MS  300        /* 编码器稳定窗口，判定锁定的时间（ms） */
#define FOC_ALIGN_TIMEOUT_MS 3000       /* 第 2 步（alpha）超时 -> 故障码 3 */
#define FOC_ALIGN_HOLD_MS    2000       /* 锁定后再保持输出（ms），足够读 id/iq */

#define FOC_OC_LIMIT_A       5.5f        /* 过流跳闸默认值（A，每相）：5.5A -> ~69mNm；受 +-10A 传感器限制（电机额定 17A）；运行时：g_foc_oc_limit_a */
#define FOC_CUR_SIGN         -1          /* 电流符号修正：-1 由 mode23 对齐确认（id 必须为正） */
#define FOC_ENC_DIR         -1          /* 编码器电角度方向：-1 已确认（mode21：+5Hz 场 -> 转子 -30rpm；enc_dir=-1 使锁定时 enc_elec 跟上 theta） */

/* 控制角 +180°（翻转转矩方向）：注意 cur_sign=-1 会抵消 pi_off（ctrl+180 与不加等价），
 * 所以 pi_off 实际不改变方向；RUN 方向由 FOC_RUN_ANCHOR_DEG + FOC_RUN_IQ_SIGN 决定 【mode22】 */
#define FOC_PI_OFF_180       0

/* RUN 帧 / 方向（运行时可 Watch 改：g_foc_anchor_deg、g_foc_run_iq_sign）【mode22】 */
#define FOC_RUN_ANCHOR_DEG   180         /* 在 hold 锚点上额外叠加的角度（度）：180 = 本台架上唯一能转起来的组合（方向仍待最终确定） */
#define FOC_RUN_IQ_SIGN      1           /* RUN 的 Iq 符号：anchor=180 配 1 能转（但方向为正，未定论）；可试 -1 得负向 */

#define FOC_PI_KP            0.10f       /* 电流环 PI Kp（V/A）：kp = 2*pi*fc*L -> fc ~ 375Hz @ L=42.3uH */
#define FOC_PI_KI            240.0f      /* 电流环 PI Ki（1/s）：极点零点相消 ki = kp*R/L（tau = 423us） */
#define FOC_PI_UMAX_V        1.0f        /* 电流环 PI 输出钳位（V）：1.0V 覆盖静止最坏 ~6A；2A 正常 ~0.3V */

#define FOC_VMAX_V            1.0f       /* 电流环最大电压幅值（V），Watch 可调：留足 3A 裕量 */
#define FOC_VRAMP_V_S         0.5f       /* 电压包络爬升率（V/s）：到 1.0V 需 2s */
#define FOC_CUR_FB_ALPHA      0.3f       /* id/iq 反馈 EMA 权重（1.0 = 关闭） */
#endif /* __MOTOR_CONFIG_H__ */
