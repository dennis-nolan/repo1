/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CIRCULAR_BUFFER_HEADER
#define CIRCULAR_BUFFER_HEADER

#include "CommonTypes.h"

#define IS_BUFFER_FULL(x) (x->count >= x->size) //(x->bufferFull)
#define IS_BUFFER_EMPTY(x) (x->count == 0) //(x->tailIndex == x->headIndex && !x->bufferFull)

typedef struct
{
	uint8_t size;
	uint8_t headIndex;
	uint8_t tailIndex;
	//bool bufferFull;	//if head==tail and this isn't set the buffer is empty.
	uint8_t count;
	bool locked;		//used to make buffer operations atomic
	void** buffer;		//the actual buffer contents
} CircularBuffer;

// Creates and allocates a new circular buffer
CircularBuffer* BufferInit(uint8_t size);

// Deallocates a circular buffer.
//   Set freeItems=TRUE to free any items remaining in the buffer before freeing the buffer
void BufferDeinit(CircularBuffer* buffer, bool freeItems);

// Writes an item to the buffer. Returns false if full or locked. Does not make a copy of item.
bool BufferWrite(CircularBuffer* buffer, void* item);

// Overwrites an item at an offset, with the option to free the old item
bool BufferWriteOffset(CircularBuffer* buffer, void* item, uint8_t offset, bool freeItems);

// Reads an item from the buffer. Returns false if empty or locked. Does not make a copy of item.
bool BufferRead(CircularBuffer* buffer, void** item);

// Same as BufferRead, but reads at an offset from the buffer and does not modify the buffer.
bool BufferReadOffset(CircularBuffer* buffer, void** item, uint8_t offset);////////////////////////make offset signed so you can do -1 for last item?

// Returns the current number of items in the buffer
uint8_t GetNumItems(CircularBuffer* buffer);

//close recursive include ifdef
#endif