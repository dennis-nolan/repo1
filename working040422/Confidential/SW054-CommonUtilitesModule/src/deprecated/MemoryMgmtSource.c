
#include <stdlib.h>
#include "MemoryMgmtHeader.h"

#ifdef TRACK_MEM_USED
static uint32_t usedUserMem = 0;
static uint32_t usedMemTotal = 0; //includes memory tracking overhead
#endif

static void OutOfMemory(void)
{
	//for now just loop forever.
	while(1);
	
	/////////////TODO - log the fault and reset the processor
}

void MemInit(void)
{
#ifdef TRACK_MEM_USED
	usedUserMem = 0;
	usedMemTotal = 0;
#endif
}

void* GetMem(size_t size)
{
	//try to get the memory
#ifdef TRACK_MEM_USED
	//allocate space to store the size of this pointer adjacent to it
	void* allocated = malloc(size + sizeof(size_t));
#else
	void* allocated = malloc(size);
#endif
	
	//if it fails then handle the out of memory situation
	if (allocated == 0)
		OutOfMemory();
	
#ifdef TRACK_MEM_USED
	//update counts
	usedUserMem += size;
	usedMemTotal += size + sizeof(size_t);
	
	//the front of the space is to store the size
	size_t* sizePtr = (size_t*)allocated;
	*sizePtr = size;
	
	//the rest of the space is for the user
	allocated = (void*)((uint32_t)allocated + sizeof(size_t));
#endif
	
	return allocated;
}

void FreeMem(void** ptr)
{
	//if pointer to the pointer is 0 there is nothing we can do
	if (ptr == 0)
		return;
	
	//if the pointer is not 0, free it
	if (*ptr != 0)
	{
#ifdef TRACK_MEM_USED
		//get the real allocated pointer
		void* originalMem = (void*)((uint32_t)(*ptr) - sizeof(size_t));
		
		//use the stored size to decrement counts
		usedUserMem -= *(size_t*)originalMem;
		usedMemTotal -= *(size_t*)originalMem + sizeof(size_t);
		
		//free the real pointer
		free(originalMem);
#else
		free(*ptr);
#endif
	}
	
	//set the pointer to 0 so we know it is freed
	*ptr = 0;
}

#ifdef TRACK_MEM_USED
uint32_t GetMemUsed(void)
{
	return usedUserMem;
}
#endif