#ifndef __GPIO_IO_H__
#define __GPIO_IO_H__

#include "hc32_ll.h"

/* 便捷的引脚操作宏 */
#define GPIO_SET(port, pin)     GPIO_SetPins(port, pin)
#define GPIO_RESET(port, pin)   GPIO_ResetPins(port, pin)
#define GPIO_TOGGLE(port, pin)  GPIO_TogglePins(port, pin)
#define GPIO_READ(port, pin)    GPIO_ReadInputPins(port, pin)

/* 引脚定义 - 方便应用层使用 */
#define PH2_PIN     GPIO_PIN_02
#define PH2_PORT    GPIO_PORT_H

#define PA3_PIN     GPIO_PIN_03
#define PA3_PORT    GPIO_PORT_A

/* 电平状态枚举 - 可以直接使用驱动中的 en_pin_state_t */
/* PIN_RESET = 0 (低电平), PIN_SET = 1 (高电平) */

/* 定义GPIO初始状态枚举 */
typedef enum
{
    GPIO_INIT_LOW  = 0,   /* 初始化为低电平 */
    GPIO_INIT_HIGH = 1    /* 初始化为高电平 */
} en_gpio_init_state_t;


/* 通用GPIO输出初始化函数 */
void Output_GPIO_Init(uint8_t u8Port, uint16_t u16Pin, en_gpio_init_state_t u8InitState);

void Input_GPIO_Init(uint8_t u8Port, uint16_t u16Pin, en_functional_state_t enablePullUp);


#endif /* __GPIO_H__ */
