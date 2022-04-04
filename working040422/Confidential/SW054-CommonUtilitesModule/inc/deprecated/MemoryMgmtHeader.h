/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MEMORY_HEADER
#define MEMORY_HEADER

//NOTE: to actually implement GetMemUsed it requires storing
//		the size of each item adjacent its the pointer. This
//		can be useful for debugging, but takes up too much
//		extra space for use in production.
//		Define "TRACK_MEM_USED" to use this functionality.

#ifdef TRACK_MEM_USED
#include <stdint.h>
#endif

#include <ysizet.h>

void MemInit(void);
void* GetMem(size_t size);
void FreeMem(void** ptr);

#ifdef TRACK_MEM_USED
uint32_t GetMemUsed(void);
#endif

//close recursive include ifdef
#endif