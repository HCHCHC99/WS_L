#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 包含所有ADP层的模块头文件 */
#include "Adapter.h"



/* 硬件初始化函数声明 */
void Hardware_Init(void);

/* PA6 ADC 数据处理函数 - 在 main loop 中周期性调用 */
void Pa6_Adc_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __HARDWARE_H__ */
