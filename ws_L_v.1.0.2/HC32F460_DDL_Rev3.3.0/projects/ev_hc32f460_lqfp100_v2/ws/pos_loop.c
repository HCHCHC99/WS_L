#include "pos_loop.h"

static float s_out = 0.0f;

void PosLoop_Init(void) {}

void PosLoop_Update(float target_pos)
{
    (void)target_pos;
    s_out = 0.0f;   /* stub: no position feedback yet */
}

float PosLoop_GetPos(void) { return 0.0f; }   /* stub: implement encoder feedback */
float PosLoop_GetOutput(void) { return s_out; }