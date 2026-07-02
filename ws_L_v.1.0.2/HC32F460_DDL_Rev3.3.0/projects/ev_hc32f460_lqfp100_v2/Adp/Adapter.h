#ifndef __ADAPTER_H__
#define __ADAPTER_H__

/* C++���� */
#ifdef __cplusplus
extern "C" {
#endif

#define PHU_PORT        GPIO_PORT_B
#define PHU_PIN         GPIO_PIN_08

#define PHV_PORT        GPIO_PORT_B
#define PHV_PIN         GPIO_PIN_09

#define GPIO_LED_PORT   GPIO_PORT_H
#define GPIO_LED_PIN    GPIO_PIN_02
/*==============================================================================
 * ��������Ӳ������ģ��
 *============================================================================*/

/* ����Ӳ������ - оƬ�������� */
#include "Gpio_io.h"     /* Ӳ����ʼ����� */
#include "Sysclk.h"      /* SysTickʱ������ */
#include "Timer0_Unit2.h"       /* Timer0�����ж����� */
#include "Timer0_Unit1.h" 
#include "Template_Pwm.h"
#include "Pwm.h"
#include "hc32_ll.h"
#include "Aos.h"
#include "Adc.h"
#include "Motor_hall.h"
#include "rtt_log.h"
#ifdef __cplusplus
}
#endif

#endif /* __ADAPTER_H__ */
