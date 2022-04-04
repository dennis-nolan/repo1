/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RANGETEST_HEADER
#define RANGETEST_HEADER

#include "CommonTypes.h"

void BeginRangeTest(uint32_t ID, uint16_t numPackets, uint8_t spacingMin, uint8_t spacingMax);
void AbortRangeTest(void);
bool RangeTestInProgress(void);
uint16_t GetRangeTestPacketsLost(void);
void RangeTestAckHandler(uint16_t MAC, bool success);

//close recursive include ifdef
#endif