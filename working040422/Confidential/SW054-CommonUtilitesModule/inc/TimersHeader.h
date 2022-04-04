/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIMERS_HEADER
#define TIMERS_HEADER

#include "ThreadingHeader.h"

#ifndef MAX_TIMERS
#pragma message("MAX_TIMERS not defined. Defaulting to 16.")
#define MAX_TIMERS 16
#endif

#ifndef MAX_TIMEOUTS
#pragma message("MAX_TIMEOUTS not defined. Defaulting to 16.")
#define MAX_TIMEOUTS 16
#endif

void TimerInit(uint8_t tickms);
bool RegisterTimer(void (*callback)(uint8_t), uint8_t ID, uint32_t msDelay, bool persistent, bool immediate);
void CancelTimer(void (*callback)(uint8_t), uint8_t ID);
bool RegisterTimeout(bool* flag, uint32_t msDelay);
void CancelTimeout(bool* flag);
void Tick(void);
void RunTimerBackgroundTasks(void);
uint32_t GetMsSinceStart(void);
uint32_t GetMsElapsed(uint32_t startTime);

//close recursive include ifdef
#endif