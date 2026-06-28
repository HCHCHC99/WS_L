#ifndef __HALL_SENSOR_3CH_H__
#define __HALL_SENSOR_3CH_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ���� */
typedef enum {
    HALL3_DIR_NONE = 0,
    HALL3_DIR_FORWARD,
    HALL3_DIR_REVERSE,
} hall3_direction_t;

/* �ص�: Hall ��������ʱ����, ISR ��ִ�� */
typedef void (*hall3_step_callback_t)(uint8_t step, hall3_direction_t dir);
/* �ص�: ��⵽ 000/111 ʱ���� */
typedef void (*hall3_fault_callback_t)(uint8_t hall_state);

/* ���� */
typedef struct {
    uint8_t   port[3];           /* 0=U(PA10) 1=V(PA9) 2=W(PA8) */
    uint16_t  pin[3];
    uint32_t  eirq_ch[3];
    IRQn_Type irqn[3];
    uint32_t  irq_src[3];
    uint8_t   irq_priority;

    uint8_t   pole_pairs;
    uint8_t   hall_to_step[8];   /* 3bit ״̬�� �� ���ಽ 0~5, 0xFF=���� */

    hall3_step_callback_t  on_step;    /* ISR �ڵ��� */
    hall3_fault_callback_t on_fault;   /* ISR �ڵ��� */

    /* �������� */
    uint8_t   align_step;         /* �����õĻ��ಽ */
    float     align_duty_pct;     /* ����ռ�ձ� */
    uint16_t  align_duration_ms;  /* �������ʱ�� */

    uint16_t  stall_timeout_ms;
} hall_3ch_config_t;

/* ��͸����� */
typedef struct hall_3ch_instance_t* hall_3ch_handle_t;

/* API */
void              hall_3ch_system_init(void);
hall_3ch_handle_t hall_3ch_create(const hall_3ch_config_t *cfg);
void              hall_3ch_destroy(hall_3ch_handle_t h);

void              hall_3ch_start(hall_3ch_handle_t h, hall3_direction_t dir);
void              hall_3ch_start_flying(hall_3ch_handle_t h, hall3_direction_t dir);  /* ��������, �������� */
void              hall_3ch_stop(hall_3ch_handle_t h);    /* ͣ, �� IDLE */
void              hall_3ch_set_table(hall_3ch_handle_t h, const uint8_t table[8]);

void              hall_3ch_update(hall_3ch_handle_t h);  /* ��ѭ����ʱ�� */

uint8_t           hall_3ch_read_raw(hall_3ch_handle_t h); /* Read raw 3-bit Hall GPIO state directly.
                                                             Works in any FSM state (IDLE/ALIGNING/RUNNING/FAULT).
                                                             Returns 0b001-0b110 for valid states,
                                                             0b000 or 0b111 for invalid. */

float             hall_3ch_get_rpm(hall_3ch_handle_t h);
hall3_direction_t hall_3ch_get_direction(hall_3ch_handle_t h);
uint8_t           hall_3ch_is_running(hall_3ch_handle_t h);
uint8_t           hall_3ch_is_stalled(hall_3ch_handle_t h);

/* J-Scope HSS ���μ�� */
extern volatile uint8_t g_scope_ha;     /* Hall A ��ƽ (0/1) */
extern volatile uint8_t g_scope_hb;     /* Hall B ��ƽ (0/1) */
extern volatile uint8_t g_scope_hc;     /* Hall C ��ƽ (0/1) */
extern volatile uint8_t g_scope_step;   /* ��ǰ���ಽ (0-5) */
extern volatile int16_t g_scope_rpm;    /* �˲���ת�� */

#endif
