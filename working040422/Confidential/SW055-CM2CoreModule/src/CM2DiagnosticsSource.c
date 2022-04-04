/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
******************************* END VERSION INFO ******************************/

//#include "MemoryMgmtHeader.h"
#include "CM2DiagnosticsHeader.h"
//#include "CircularBufferHeader.h"

//static CircularBuffer* cachedLogs = 0;
static uint8_t cacheSize = 0;
static bool diagnosticsLocked = FALSE;

static bool AreLogsIdentical(uint8_t* log1, uint8_t* log2)
{
	if (log1 == 0 || log2 == 0)
		return FALSE;
	
	// Compare headers
	if (log1[0] != log2[0] || 
		log1[1] != log2[1] || 
		(log1[2] & 0xE0) != (log2[2] & 0xE0)) // note: don't compare # occurrences
		return FALSE;
	
	uint8_t logOffset = 3;
	if (log1[2] & (1<<7))
		logOffset = 8;
	
	// Compare logs
	for (uint8_t i=logOffset; i < (logOffset + (log1[1] & 0x3F)); i++)
	{
		if (log1[i] != log2[i])
			return FALSE;
	}
	
	return TRUE;
}

static bool AddToExisting(uint8_t* packedLog)
{
	// Wait for the cache to be unlocked, then lock it
	while (diagnosticsLocked); /////////////////////TODO: timeout and fail
	diagnosticsLocked = TRUE;
	
	uint8_t* log = 0;
	
	// Search for log in RAM
	for (uint8_t i=0; i<GetNumItems(cachedLogs); i++)
	{
		if (BufferReadOffset(cachedLogs, (void**)&log, i))
		{
			if (AreLogsIdentical(packedLog, log))
			{
				// If found add to occurrences (if not maxed out) and return TRUE
				if ((log[2] & 0x1F) < 0x1F)
				{
					log[2]++;
					////////////////////TODO: update V,I,T if needed
					diagnosticsLocked = FALSE;
					return TRUE;
				}
			}
		}
	}
	
	// We don't have a similar log in RAM, return failure and we'll store it
	diagnosticsLocked = FALSE;
	return FALSE;
}

static void TryOverwriteLog(uint8_t* packedLog)
{
	// Wait for the cache to be unlocked, then lock it
	while (diagnosticsLocked); /////////////////////TODO: timeout and fail
	diagnosticsLocked = TRUE;
	
	uint8_t* log = 0;
	
	// Search for lower or equal severity log in RAM
	for (uint8_t i=0; i<GetNumItems(cachedLogs); i++)
	{
		if (BufferReadOffset(cachedLogs, (void**)&log, i))
		{
			if ((log[2] & 0x60) <= (packedLog[2] & 0x60))
			{
				// found a match, overwrite it.
				BufferWriteOffset(cachedLogs, packedLog, i, TRUE);
				diagnosticsLocked = FALSE;
				return;
			}
		}
	}
	
	// unable to overwrite, so free it
	FreeMem((void**)&packedLog);
	diagnosticsLocked = FALSE;
}

void DiagnosticsInit(uint8_t ramCacheSize)
{
	cacheSize = ramCacheSize;
	// Create RAM cache
	cachedLogs = BufferInit(cacheSize);
	diagnosticsLocked = FALSE;
}

void DiagnosticsDeinit(void)
{
	BufferDeinit(cachedLogs, TRUE);
}

//source(10b),len(6b),S/L(1b),sev(2b),occ(5b),V(2B),I(2B),T(1B),log(0-63B)
void WriteDiagnosticLog(uint16_t source, uint8_t severity, uint8_t length, bool includeVIT, uint8_t* log, bool immediate)
{
	// Check for invalid params
	if (length > 0x3F | severity > 3 | source > 0x3FF)
		return;
	
	// Create the packed log
	uint8_t logOffset = 3;
	if (includeVIT)
		logOffset = 8;
	
	uint8_t* packedLog = (uint8_t*)GetMem(length + logOffset);
	
	// Pack header bytes
	packedLog[0] = source >> 2;				// top of source
	packedLog[1] = (source << 6) | length;	// bottom of source and length
	packedLog[2] = severity << 5;		// severity and # occurrences
	
	if (includeVIT)
	{
		// Mark that this is a long-form log
		packedLog[2] |= (1<<7);				// short/long
		
		// Pack voltage, current, and temperature
		/*packedLog[3] = voltage >> 8;
		packedLog[4] = voltage;
		packedLog[5] = current >> 8;
		packedLog[6] = current;
		packedLog[7] = temperature;*////////////////////////////////////////////////////////////////
	}
	
	// Copy the log itself
	for (uint8_t i=0; i<length; i++)
		packedLog[i+logOffset] = log[i];
	
	// Store the log somewhere
	if (immediate)
	{
		//write to persistent storage
		////////////TODO
	}
	else
	{
		//cache in RAM
		if (AddToExisting(packedLog))
		{
			FreeMem((void**)&packedLog); // We no longer need this log, we already added it to another
		}
		else
		{
			// See if there is room in the RAM cache
			if (!IS_BUFFER_FULL(cachedLogs))
			{
				BufferWrite(cachedLogs, packedLog);
			}
			else
			{
				// See if we can overwrite a lower severity log
				TryOverwriteLog(packedLog);
			}
		}
	}
}

void ReadDiagnosticLogs(uint8_t storageLocation, uint8_t*** logArray, uint8_t* numLogs)
{
	////////////////TODO: handle other locations///////////
	if (storageLocation != DIAG_STORAGE_RAM)///////////////
		return;////////////////////////////////////////////
	
	// If there is nothing to read, return
	if (cachedLogs == 0 || IS_BUFFER_EMPTY(cachedLogs))
	{
		*logArray = 0;
		*numLogs = 0;
		return;
	}
	
	// Wait for unlock, then lock it
	while (diagnosticsLocked); /////////////////////TODO: timeout and fail
	diagnosticsLocked = TRUE;
	
	*numLogs = GetNumItems(cachedLogs);
	*logArray = (uint8_t**)GetMem(*numLogs * sizeof(uint8_t*));
	
	for(uint8_t i=0; i<*numLogs; i++)
	{
		BufferReadOffset(cachedLogs, (void**)&((*logArray)[i]), i);
	}
	
	// Unlock and return
	diagnosticsLocked = FALSE;
	return;
}
