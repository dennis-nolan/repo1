/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef THREADING_HEADER
#define THREADING_HEADER

#include "CommonTypes.h"

#ifndef MAX_THREADS
#pragma message("MAX_THREADS not defined. Defaulting to 8.")
#define MAX_THREADS 8
#endif

void ThreadingInit(void);
void RegisterThread(void (*threadHandler)(void), uint8_t priority);
void RunNextThread(void);
void RunAllThreads(void);

//close recursive include ifdef
#endif