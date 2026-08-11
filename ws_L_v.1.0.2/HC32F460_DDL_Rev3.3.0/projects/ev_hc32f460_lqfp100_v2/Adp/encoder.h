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
 *        Encoder: 1024 lines, 4x quadrature -> 4096 counts / rev (4096cpr).
 *******************************************************************************
 */

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Encoder geometry */
#define ENCODER_LINES   1024u
#define ENCODER_CPR     (ENCODER_LINES * 4u)   /* 4096 counts/rev (4x quadrature) */

void    Encoder_Init(void);
void    Encoder_Update(void);          /* periodic (main loop): position + speed */

int32_t Encoder_GetCount(void);        /* position since last Z (signed, 4x counts) */
float   Encoder_GetAngleDeg(void);     /* 0..360 within current revolution */
float   Encoder_GetSpeedRpm(void);     /* filtered speed */
int8_t  Encoder_GetDirection(void);    /* +1 CW, -1 CCW, 0 stopped */
uint32_t Encoder_GetRevCount(void);    /* Z pulses seen (revolutions) */

/* J-Scope observability */
extern volatile int32_t  g_enc_count;      /* position since last Z (signed) */
extern volatile float    g_enc_angle_deg;  /* 0..360 */
extern volatile float    g_enc_speed_rpm;  /* rpm */
extern volatile int8_t   g_enc_dir;        /* +1 CW, -1 CCW, 0 stopped */
extern volatile uint32_t g_enc_rev;        /* Z pulses (revolutions) */

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
