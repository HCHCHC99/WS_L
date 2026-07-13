/**
 *******************************************************************************
 * @file  test_Vofa.h
 * @brief VOFA+ test signal generator (CH1 sawtooth + CH2 counter).
 *
 * Controlled by VOFA_TEST_ENABLE macro — set to 0 to exclude from build.
 *******************************************************************************
 */

#ifndef __TEST_VOFA_H__
#define __TEST_VOFA_H__

#include <stdint.h>

/* Set to 1 to enable test signals, 0 to disable */
#define VOFA_TEST_ENABLE  0

#if VOFA_TEST_ENABLE

#ifdef __cplusplus
extern "C" {
#endif

void TestVofa_Init(void);
void TestVofa_Run(void);   /* call in main loop */

#ifdef __cplusplus
}
#endif

#endif /* VOFA_TEST_ENABLE */
#endif /* __TEST_VOFA_H__ */
