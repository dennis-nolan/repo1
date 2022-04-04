#include "TimersHeader.h"

typedef struct Timer {
	void (*callback) (uint8_t);
	uint32_t delay;
	uint32_t count;
	uint8_t  ID;
	bool persistent;
	bool hasExpired;
	bool immediate;
} Timer;

typedef struct TimeoutTimer {
	bool* flagPointer;
	uint32_t delay;
	uint32_t count;
} TimeoutTimer;

void EraseTimer(int index);
void EraseTimeout(int index);
int FindTimer(void (*callback)(uint8_t), uint8_t ID);
int FindTimeout(bool* flag);

static Timer timerList[MAX_TIMERS] = {0};
static TimeoutTimer timeoutList[MAX_TIMEOUTS] = {0};
static bool backgroundTimerExpired = FALSE;
static bool locked = TRUE;
static bool handleTickOnUnlock = FALSE;
static uint8_t msPerTick = 1;
static bool ranOutOfTimers = FALSE;
static bool ranOutOfTimeouts = FALSE;
static uint32_t msSinceStart = 0;

static void HandleTick(void);

static void Lock(void)
{
	//wait for unlock
	while(locked);
	
	//lock
	locked = TRUE;
}

static void Unlock(void)
{
	//if we had to delay a tick update do it now, before unlocking
	if (handleTickOnUnlock)
	{
		handleTickOnUnlock = FALSE;
		HandleTick();
	}
	
	locked = FALSE;
}

static void EraseTimer(int index)
{
  	timerList[index].callback = 0;
	timerList[index].delay = 0;
	timerList[index].count = 0;
	timerList[index].ID = 0;
	timerList[index].persistent = FALSE;
	timerList[index].hasExpired = FALSE;
	timerList[index].immediate = FALSE;
}

static void EraseTimeout(int index)
{
  	timeoutList[index].flagPointer = 0;
	timeoutList[index].delay = 0;
	timeoutList[index].count = 0;
}

static int FindTimer(void (*callback)(uint8_t), uint8_t ID)
{
	for(uint8_t i=0; i<MAX_TIMERS; i++)
	{
		if ((timerList[i].callback == callback) && (timerList[i].ID == ID))
			return i;
	}
	
	return -1;
}

static int FindTimeout(bool* flag)
{
	for(uint8_t i=0; i<MAX_TIMEOUTS; i++)
	{
		if (timeoutList[i].flagPointer == flag)
			return i;
	}
	
	return -1;
}

void TimerInit(uint8_t tickms)
{
	locked = TRUE;
	
	msPerTick = tickms;
	
	for(uint8_t i=0; i<MAX_TIMERS; i++)
	{
		EraseTimer(i);
	}
	
	for(uint8_t i=0; i<MAX_TIMEOUTS; i++)
	{
		EraseTimeout(i);
	}
	
	handleTickOnUnlock = FALSE;
	locked = FALSE;
	
	RegisterThread(RunTimerBackgroundTasks, 0x0A);
}

void RunTimerBackgroundTasks(void)
{
	if (backgroundTimerExpired)
	{
		backgroundTimerExpired = FALSE;
		void (*callback)(uint8_t);
		uint8_t ID;
		
		for(uint8_t i=0; i<MAX_TIMERS; i++)
		{
			callback = 0;
			
			Lock();
			
			if ((timerList[i].callback != 0) && (timerList[i].hasExpired == TRUE))
			{
				// Save callback info
				callback = timerList[i].callback;
				ID = timerList[i].ID;
				
				// If not persistent erase the timer
				if (!timerList[i].persistent)
					EraseTimer(i);
				// else the count has already been reloaded
				
				timerList[i].hasExpired = FALSE;
			}
			
			Unlock();
		
			// Call the callback outside of the lock
			if (callback != 0)
				callback(ID);
		}
	}
}

uint32_t GetMsSinceStart(void)
{ return msSinceStart; }

uint32_t GetMsElapsed(uint32_t startTime)
{
	if (msSinceStart >= startTime)
		return msSinceStart - startTime;
	else
		return (uint32_t)((uint64_t)msSinceStart + 0xFFFFFFFF - startTime);
}

bool RegisterTimer(void (*callback)(uint8_t), uint8_t ID, uint32_t msDelay, bool persistent, bool immediate)
{
	if (callback == 0)
		return FALSE;
	
	Lock();
	
	// Check for an existing instance of this timer
	int index = FindTimer(callback, ID);
	bool registered = FALSE;
	
	if (index >= 0)
	{
		// Found an instance of this timer - replace it
		timerList[index].delay = msDelay;
		timerList[index].count = msDelay;
		timerList[index].persistent = persistent;
		timerList[index].hasExpired = FALSE;
		timerList[index].immediate = immediate;
		registered = TRUE;
	}
	else
	{
		for(uint8_t i=0; i<MAX_TIMERS; i++)
		{
			if (timerList[i].callback == 0)
			{
				timerList[i].callback = callback;
				timerList[i].delay = msDelay;
				timerList[i].count = msDelay;
				timerList[i].ID = ID;
				timerList[i].persistent = persistent;
				timerList[i].hasExpired = FALSE;
				timerList[i].immediate = immediate;
				registered = TRUE;
				break;
			}
		}
	}
	
	if (FALSE == registered)
	{
		//failed to find an empty timer slot
		ranOutOfTimers = TRUE;
	}
	
	Unlock();
	
	return registered;
}

bool RegisterTimeout(bool* flag, uint32_t msDelay)
{
	if (flag == 0)
		return FALSE;
	
	Lock();
	
	// Check for an existing instance of this timer
	int index = FindTimeout(flag);
	bool registered = FALSE;
	
	if (index >= 0)
	{
		// Found an instance of this timeout - replace it
		timeoutList[index].delay = msDelay;
		timeoutList[index].count = msDelay;
		registered = TRUE;
	}
	else
	{
		for(uint8_t i=0; i<MAX_TIMEOUTS; i++)
		{
			if (timeoutList[i].flagPointer == 0)
			{
				timeoutList[i].flagPointer = flag;
				timeoutList[i].delay = msDelay;
				timeoutList[i].count = msDelay;
				*flag = FALSE;
				registered = TRUE;
				break;
			}
		}
	}
	
	if (FALSE == registered)
	{
		//failed to find an empty timer slot
		//log an error and timeout immediately
		ranOutOfTimeouts = TRUE;
		*flag = TRUE;
	}
	
	Unlock();
	
	return registered;
}

void CancelTimer(void (*callback)(uint8_t), uint8_t ID)
{
	if (callback == 0)
		return;
	
	Lock();
	
	int index = FindTimer(callback, ID);
	
	if (index >= 0)
		EraseTimer(index);
	
	Unlock();
}

void CancelTimeout(bool* flag)
{
	if (flag == 0)
		return;
	
	Lock();
	
	int index = FindTimeout(flag);
	
	if (index >= 0)
		EraseTimeout(index);
	
	Unlock();
}

void Tick(void)
{
	if (!locked)
	{
		Lock();
		HandleTick();
		Unlock();
	}
	else
		handleTickOnUnlock = TRUE;
}

static void HandleTick(void)
{
	void (*callback)(uint8_t);
	uint8_t ID;
	
	msSinceStart += msPerTick;
	
	//loop through all timers
	for(uint8_t i=0; i<MAX_TIMERS; i++)
	{
		if ((timerList[i].callback != 0) && (timerList[i].hasExpired == FALSE))
		{
			// Handle the case that the delay was set to 0 - don't wrap around
			if (timerList[i].count > 0)
			{
				if (timerList[i].count < msPerTick)
					timerList[i].count = 0;
				else
					timerList[i].count-=msPerTick;
			}
			
			// Time up?
			if (timerList[i].count == 0)
			{
			  	// Handle persistence
				if (timerList[i].persistent)
				{
					// Reset the timer
					timerList[i].count = timerList[i].delay;
				}
			  
				// Handle execution
			  	if (timerList[i].immediate)
				{
					// Save callback info
					callback = timerList[i].callback;
					ID = timerList[i].ID;
					
					// Erase if not persistent
					if (!timerList[i].persistent)
						EraseTimer(i);
					
					// Call the callback immediately
					callback(ID);
				}
				else
				{
					// Mark as expired to handle during background tasks
					timerList[i].hasExpired = TRUE;
					backgroundTimerExpired = TRUE;
				}
			}
		}
	}
	
	//loop through all timeouts
	for(uint8_t i=0; i<MAX_TIMEOUTS; i++)
	{
		//check if this is an active timeout
		if (timeoutList[i].flagPointer != 0)
		{
			if (timeoutList[i].count > 0)
			{
				if (timeoutList[i].count < msPerTick)
					timeoutList[i].count = 0;
				else
					timeoutList[i].count-=msPerTick;
			}
			
			if (timeoutList[i].count == 0)
			{
				//timeout expired. mark it and erase it
				*(timeoutList[i].flagPointer) = TRUE;
				EraseTimeout(i);
			}
		}
	}
}