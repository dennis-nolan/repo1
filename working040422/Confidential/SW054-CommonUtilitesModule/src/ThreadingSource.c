
#include "ThreadingHeader.h"
#include "TimersHeader.h"

#ifdef THREAD_DEBUG_LOGGING
#include "FlashDriverHeader.h" //////////////////////////////////
#include "HardwareDefines.h"/////////////////////////////////
#endif

typedef struct Thread {
	void (*callback) (void);
	uint8_t priority; // lower values have higher priority
	uint32_t maxTimeSpent;
	uint64_t totalTimeSpent;
} Thread;

Thread threadList[MAX_THREADS];
static uint8_t currentThread = 0;
static uint32_t tickAtThreadStart = 0;
static uint32_t maxTicksSeen = 0;

void ThreadingInit(void)
{
	for(uint8_t i=0; i<MAX_THREADS; i++)
	{
		threadList[i].callback = 0;
		threadList[i].priority = 0xFF;
		threadList[i].maxTimeSpent = 0;
		threadList[i].totalTimeSpent = 0;
	}
}

void RegisterThread(void (*threadHandler)(void), uint8_t priority)
{
	Thread toMove = {0, 0xFF}, temp;
	bool threadPlaced = FALSE;
	
	// Check if thread list is already full
	if (threadList[MAX_THREADS-1].callback != 0)
		return;
	
	// Hunt for the right place to put the thread based on priority and push lower priority threads back
	for(uint8_t i=0; i<MAX_THREADS; i++)
	{
		if (!threadPlaced)
		{
			// Find where the thread should sit
			if ((threadList[i].callback == 0) || (threadList[i].priority > priority))
			{
				// Anything currently in this slot has to be shifted back
				toMove = threadList[i];
				
				// Place the thread here
				threadList[i].callback = threadHandler;
				threadList[i].priority = priority;
				threadPlaced = TRUE;
			}
		}
		else if (toMove.callback != 0)
		{
			// Shift lower priority threads back
			temp = threadList[i];
			threadList[i] = toMove;
			toMove = temp;
		}
		else
		{
			// Nothing left to do
			break;
		}
	}
}

static bool RunThread(uint8_t index)
{
	if (index < MAX_THREADS && threadList[index].callback != 0)
	{
		
#ifdef THREAD_DEBUG_LOGGING
		FlashWriteSettingWord(16, index);
		
		ALL_LEDS_OFF;
		if (index == 0)
			LED1_ON;
		else if (index == 1)
			LED2_ON;
		else if (index == 2)
		{	LED2_ON; LED1_ON;}
		else if (index == 3)
			LED3_ON;
		else if (index == 4)
		{	LED3_ON; LED1_ON;}
#endif
		
		//start timing
		tickAtThreadStart = GetMsSinceStart();
	
		// Call the callback. Handle the case where we have no threads so even the first thread is null.
		if (threadList[index].callback != 0)
			threadList[index].callback();
		
		//record runtime for this thread
		uint32_t diff = GetMsElapsed(tickAtThreadStart);
		threadList[index].totalTimeSpent += diff;
		if (diff > threadList[index].maxTimeSpent)
		{
			threadList[index].maxTimeSpent = diff;
		}

#ifdef THREAD_DEBUG_LOGGING
		if (threadList[index].maxTimeSpent > maxTicksSeen)
		{
		  	maxTicksSeen = threadList[index].maxTimeSpent;
		  	FlashWriteSettingWord(20, index);
		  	FlashWriteSettingWord(24, threadList[index].maxTimeSpent);
		}
#endif
		
		return TRUE;
	}
	else
		return FALSE;
}

void RunNextThread(void)
{
	if (!RunThread(currentThread++))
		currentThread = 0;
}

void RunAllThreads(void)
{
	for (currentThread=0; currentThread<MAX_THREADS; currentThread++)
	{
		if (!RunThread(currentThread))
			break;
	}
	
	currentThread = 0;
}