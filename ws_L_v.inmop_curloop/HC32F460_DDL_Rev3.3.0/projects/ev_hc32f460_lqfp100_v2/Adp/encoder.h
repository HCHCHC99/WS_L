/**
 *******************************************************************************
 * @file  encoder.h
 * @brief ABZ quadrature encoder driver (TIMERA_1 hardware quadrature count).
 *
 *        PA8  = TIMA1_CLKA (A phase, FUNC4)
 *        PA9  = TIMA1_CLKB (B phase, FUNC4)
 *        PA10 = Z index    (EXTINT_CH10 rising edge -> counter cleared)
 *
 *        Direction convention:
 *          CW  = A leads B (rising-edge order A then B)
 *          CCW = B leads A (rising-edge order B then A)
 *
 *        Encoder: 1024 lines x 4 = 4096 counts / rev.
 *******************************************************************************
 */

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Encoder geometry - 4096 counts/rev (speed data shows real sync speed ~30rpm
 * => actual CPR ~4096; the earlier hand-measured "360" was wrong) */
#define ENCODER_LINES   1024u
#define ENCODER_CPR     (ENCODER_LINES * 4u)   /* 4096 counts/rev */

void    Encoder_Init(void);
void    Encoder_Update(void);          /* periodic (main loop): position + speed */

int32_t Encoder_GetCount(void);        /* 连续累计计数（Z 不复位，signed, 4x counts） */
float   Encoder_GetAngleDeg(void);     /* 0..360 within current revolution */
float   Encoder_GetSpeedRpm(void);     /* filtered speed */
int8_t  Encoder_GetDirection(void);    /* +1 CW, -1 CCW, 0 stopped */
uint32_t Encoder_GetRevCount(void);    /* Z pulses seen (revolutions) */

/* J-Scope observability */
extern volatile int32_t  g_enc_count;      /* 连续累计计数（Z 不复位） */
extern volatile uint8_t  g_enc_dbg_print;   /* 1 = main loop prints cnt/rev every 100ms (manual CPR test) */
extern volatile float    g_enc_count_f;    /* float mirror of g_enc_count */
extern volatile float    g_enc_angle_deg;  /* 0..360 */
extern volatile float    g_enc_speed_rpm;  /* rpm */
extern volatile int8_t   g_enc_dir;        /* +1 CW, -1 CCW, 0=未建立/初始（迟滞锁存，停止后保持最后方向） */
extern volatile uint32_t g_enc_rev;        /* Z pulses (revolutions) */

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
