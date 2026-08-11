/**
 *******************************************************************************
 * @file  encoder.c
 * @brief ABZ quadrature encoder via TIMERA_1 hardware quadrature count.
 *
 *        PA8  = TIMA1_CLKA (A phase), PA9 = TIMA1_CLKB (B phase)
 *        PA10 = Z index (EXTINT_CH10 rising edge clears the counter)
 *
 *        4x quadrature: both edges of both phases are counted; the CLKA/CLKB
 *        level/edge combos select count up/down (CW = A leads B).
 *        Speed: Encoder_Update() integrates count over a ~10ms window using the
 *        Timer6 us timestamp, then converts to rpm via ENCODER_CPR.
 *******************************************************************************
 */

#include "encoder.h"
#include "hc32_ll.h"
#include "timer6_timebase.h"
#include "rtt_log.h"

/* ---- Pins / peripherals ---- */
#define ENC_CLKA_PORT   (GPIO_PORT_A)
#define ENC_CLKA_PIN    (GPIO_PIN_08)
#define ENC_CLKB_PORT   (GPIO_PORT_A)
#define ENC_CLKB_PIN    (GPIO_PIN_09)
#define ENC_Z_PORT      (GPIO_PORT_A)
#define ENC_Z_PIN       (GPIO_PIN_10)
#define ENC_Z_EIRQ      (EXTINT_CH10)
#define ENC_Z_IRQn      (INT010_IRQn)
#define ENC_Z_IRQ_SRC   (INT_SRC_PORT_EIRQ10)

/* Speed EMA coefficient (10ms window -> tau ~50ms) */
#define ENC_SPEED_ALPHA  (0.2f)
/* Speed integration window (us) */
#define ENC_SPEED_WIN_US (10000u)

/* J-Scope observability */
volatile int32_t  g_enc_count     = 0;
volatile float    g_enc_angle_deg = 0.0f;
volatile float    g_enc_speed_rpm = 0.0f;
volatile int8_t   g_enc_dir       = 0;
volatile uint32_t g_enc_rev       = 0;

static volatile int32_t  s_count    = 0;   /* position since last Z (signed) */
static volatile uint16_t s_last_cnt = 0;   /* last raw TMRA counter */
static volatile int32_t  s_cnt_sum  = 0;   /* speed window: count accumulator */
static volatile uint32_t s_us_sum   = 0;   /* speed window: time accumulator */
static volatile uint64_t s_last_us  = 0;
static float   s_speed_rpm = 0.0f;
static int8_t  s_dir       = 0;
static uint8_t s_inited    = 0;

/* ============================================================================
 * Z index ISR: one pulse per revolution -> reset position origin.
 * ==========================================================================*/
static void encoder_z_isr(void)
{
    EXTINT_ClearExtIntStatus(ENC_Z_EIRQ);
    TMRA_SetCountValue(CM_TMRA_1, 0u);   /* restart quadrature count from 0 */
    s_last_cnt = 0;
    s_count    = 0;
    s_cnt_sum  = 0;
    s_us_sum   = 0;
    g_enc_rev++;
}

/* ============================================================================
 * Encoder_Init - configure PA8/9 (TIMA1 CLKA/CLKB), PA10 (Z EXTINT) and
 *                start TMRA_1 hardware 4x quadrature counting.
 * ==========================================================================*/
void Encoder_Init(void)
{
    stc_gpio_init_t       stcGpio;
    stc_extint_init_t     stcExti;
    stc_irq_signin_config_t stcIrq;
    stc_tmra_init_t       stcTmra;

    if (s_inited) {
        return;
    }

    /* ---- GPIO: PA8/PA9 -> TIMA1_CLKA/CLKB (FUNC4) ---- */
    GPIO_SetFunc(ENC_CLKA_PORT, ENC_CLKA_PIN, GPIO_FUNC_4);  /* TIMA_1_CLKA */
    GPIO_SetFunc(ENC_CLKB_PORT, ENC_CLKB_PIN, GPIO_FUNC_4);  /* TIMA_1_CLKB */

    /* ---- GPIO: PA10 = Z index input ---- */
    GPIO_StructInit(&stcGpio);
    stcGpio.u16PinDir  = PIN_DIR_IN;
    stcGpio.u16PinAttr = PIN_ATTR_DIGITAL;
    GPIO_Init(ENC_Z_PORT, ENC_Z_PIN, &stcGpio);

    /* ---- Z: EXTINT rising edge ---- */
    stcExti.u32Edge        = EXTINT_TRIG_RISING;
    stcExti.u32Filter      = EXTINT_FILTER_ON;
    stcExti.u32FilterClock = EXTINT_FCLK_DIV64;
    EXTINT_Init(ENC_Z_EIRQ, &stcExti);
    GPIO_ExtIntCmd(ENC_Z_PORT, ENC_Z_PIN, ENABLE);

    stcIrq.enIntSrc    = ENC_Z_IRQ_SRC;
    stcIrq.enIRQn      = ENC_Z_IRQn;
    stcIrq.pfnCallback = &encoder_z_isr;
    if (LL_OK != INTC_IrqSignIn(&stcIrq)) {
        MAIN_D("[ENC] ERROR: INTC_IrqSignIn failed for Z index\r\n");
        return;
    }
    NVIC_ClearPendingIRQ(ENC_Z_IRQn);
    NVIC_SetPriority(ENC_Z_IRQn, DDL_IRQ_PRIO_03);
    NVIC_EnableIRQ(ENC_Z_IRQn);

    /* ---- TMRA_1: hardware quadrature count (4x) ---- */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_1, ENABLE);
    TMRA_StructInit(&stcTmra);
    stcTmra.u8CountSrc                = TMRA_CNT_SRC_HW;   /* external CLKA/CLKB */
    stcTmra.sw_count.u8CountMode      = TMRA_MD_SAWTOOTH;
    stcTmra.sw_count.u8CountDir       = TMRA_DIR_UP;
    stcTmra.hw_count.u16CountUpCond   = TMRA_CNT_UP_COND_INVD;
    stcTmra.hw_count.u16CountDownCond = TMRA_CNT_DOWN_COND_INVD;
    stcTmra.u32PeriodValue            = 0xFFFFu;   /* free-run; Z index clears */
    (void)TMRA_Init(CM_TMRA_1, &stcTmra);

    /* 4x quadrature (CW = A leads B):
     *   up   : CLKB low  & A rising ; CLKB high & A falling ;
     *          CLKA high & B rising ; CLKA low  & B falling
     *   down : complementary conditions (CCW = B leads A) */
    TMRA_HWCountUpCondCmd(CM_TMRA_1, TMRA_CNT_UP_COND_CLKB_LOW_CLKA_RISING,   ENABLE);
    TMRA_HWCountUpCondCmd(CM_TMRA_1, TMRA_CNT_UP_COND_CLKB_HIGH_CLKA_FALLING,  ENABLE);
    TMRA_HWCountUpCondCmd(CM_TMRA_1, TMRA_CNT_UP_COND_CLKA_HIGH_CLKB_RISING,   ENABLE);
    TMRA_HWCountUpCondCmd(CM_TMRA_1, TMRA_CNT_UP_COND_CLKA_LOW_CLKB_FALLING,   ENABLE);
    TMRA_HWCountDownCondCmd(CM_TMRA_1, TMRA_CNT_DOWN_COND_CLKB_HIGH_CLKA_RISING, ENABLE);
    TMRA_HWCountDownCondCmd(CM_TMRA_1, TMRA_CNT_DOWN_COND_CLKB_LOW_CLKA_FALLING,  ENABLE);
    TMRA_HWCountDownCondCmd(CM_TMRA_1, TMRA_CNT_DOWN_COND_CLKA_LOW_CLKB_RISING,   ENABLE);
    TMRA_HWCountDownCondCmd(CM_TMRA_1, TMRA_CNT_DOWN_COND_CLKA_HIGH_CLKB_FALLING,  ENABLE);

    TMRA_SetCountValue(CM_TMRA_1, 0u);
    TMRA_Start(CM_TMRA_1);

    s_last_cnt = 0u;
    s_last_us  = 0u;
    s_inited   = 1;

    MAIN_D("[ENC] ABZ init: PA8/PA9 TIMA1 quad x4 (CPR=%lu), PA10 Z rising clear",
           (unsigned long)ENCODER_CPR);
}

/* ============================================================================
 * Encoder_Update - accumulate position, compute speed over a ~10ms window.
 * Call from the main loop.
 * ==========================================================================*/
void Encoder_Update(void)
{
    if (!s_inited) {
        return;
    }

    Timer6_Timebase_UpdateTimestamp();
    uint64_t now_us = Timer6_Timebase_GetTimestamp();
    uint32_t cnt    = TMRA_GetCountValue(CM_TMRA_1);

    int16_t  d_cnt = (int16_t)(cnt - (uint32_t)s_last_cnt);  /* 16-bit wrap safe */
    uint32_t d_us  = (s_last_us == 0u) ? 1000u : (uint32_t)(now_us - s_last_us);
    if (d_us > 1000000u) {
        d_us = 1000000u;   /* clamp after debugger halt / long stall */
    }

    s_last_cnt = (uint16_t)cnt;
    s_last_us  = now_us;
    s_count   += (int32_t)d_cnt;

    s_cnt_sum += (int32_t)d_cnt;
    s_us_sum  += d_us;

    if (s_us_sum >= ENC_SPEED_WIN_US) {
        float rpm = (float)s_cnt_sum * 60000000.0f
                  / ((float)s_us_sum * (float)ENCODER_CPR);
        s_speed_rpm += ENC_SPEED_ALPHA * (rpm - s_speed_rpm);
        s_dir = (s_cnt_sum > 0) ? 1 : (s_cnt_sum < 0) ? -1 : 0;
        s_cnt_sum = 0;
        s_us_sum  = 0;
    }

    /* J-Scope observability */
    g_enc_count = s_count;
    {
        int32_t m = (s_count % (int32_t)ENCODER_CPR) + (int32_t)ENCODER_CPR;
        g_enc_angle_deg = (float)(m % (int32_t)ENCODER_CPR)
                        * 360.0f / (float)ENCODER_CPR;
    }
    g_enc_speed_rpm = s_speed_rpm;
    g_enc_dir       = s_dir;
}

int32_t  Encoder_GetCount(void)     { return g_enc_count; }
float    Encoder_GetAngleDeg(void)  { return g_enc_angle_deg; }
float    Encoder_GetSpeedRpm(void)  { return g_enc_speed_rpm; }
int8_t   Encoder_GetDirection(void) { return g_enc_dir; }
uint32_t Encoder_GetRevCount(void)  { return g_enc_rev; }