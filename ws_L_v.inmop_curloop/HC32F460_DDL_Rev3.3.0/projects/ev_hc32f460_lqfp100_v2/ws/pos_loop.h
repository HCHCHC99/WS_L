#ifndef __POS_LOOP_H__
#define __POS_LOOP_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void  PosLoop_Init(void);
/* target: deg or counts (reserved; not used until encoder feedback exists) */
void  PosLoop_Update(float target_pos);
float PosLoop_GetPos(void);               /* current position (stub returns 0) */
float PosLoop_GetOutput(void);            /* output = target speed (rpm) */

#ifdef __cplusplus
}
#endif

#endif /* __POS_LOOP_H__ */
