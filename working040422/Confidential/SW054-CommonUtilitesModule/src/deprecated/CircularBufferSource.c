
#include "MemoryMgmtHeader.h"
#include "CircularBufferHeader.h"

CircularBuffer* BufferInit(uint8_t size)
{
	// Don't allow 0-size buffers
	if (size == 0)
		return 0;
	
	// Allocate the struct
	CircularBuffer* toReturn = (CircularBuffer*)GetMem(sizeof(CircularBuffer));
	
	// Allocate the buffer itself
	void** buffer = (void**)GetMem(size * sizeof(void*));
	
	// Initialize contents
	toReturn->size = size;
	toReturn->headIndex = 0;
	toReturn->tailIndex = 0;
	//toReturn->bufferFull = FALSE;
	toReturn->count = 0;
	toReturn->locked = FALSE;
	toReturn->buffer = buffer;
	
	return toReturn;
}

void BufferDeinit(CircularBuffer* buffer, bool freeItems)
{
	// Skip deinit if null
	if (buffer == 0)
		return;
	
	// Skip buffer free if null
	if (buffer->buffer != 0)
	{
		// If the items stored were pointers we may have been asked to free them
		if (freeItems)
		{
			void* temp;
			while (BufferRead(buffer, &temp))
			{ FreeMem(&temp); }
		}
		
		// Free the buffer itself
		FreeMem((void**)(&(buffer->buffer)));
		//buffer->buffer = 0;
	}
	
	// Free the whole struct
	FreeMem((void**)(&buffer));
}

bool BufferWrite(CircularBuffer* buffer, void* item)
{
	// If buffer full or locked return failure
	if (IS_BUFFER_FULL(buffer) || buffer->locked)
		return FALSE;
	
	// Lock buffer for editing
	buffer->locked = TRUE;
	
	// Add item to buffer
	buffer->buffer[buffer->tailIndex] = item;
	
	// Increment tail index
	buffer->tailIndex = (buffer->tailIndex + 1) % buffer->size;
	
	// Full?
	/*if (buffer->tailIndex == buffer->headIndex)
		buffer->bufferFull = TRUE;*/
	buffer->count++;
	
	// Unlock buffer
	buffer->locked = FALSE;
	
	return TRUE;
}

bool BufferWriteOffset(CircularBuffer* buffer, void* item, uint8_t offset, bool freeItems)
{
	// If buffer empty or locked return failure
	if(IS_BUFFER_EMPTY(buffer) || buffer->locked || offset >= GetNumItems(buffer))
		return FALSE;
	
	// Lock buffer for editing
	buffer->locked = TRUE;
	
	// Free previous item
	uint8_t index = (buffer->headIndex + offset) % buffer->size;
	if (freeItems && buffer->buffer[index] != 0)
		FreeMem(&(buffer->buffer[index]));
	
	// Add item to buffer
	buffer->buffer[index] = item;
	
	// Unlock buffer
	buffer->locked = FALSE;
	
	return TRUE;
}

bool BufferRead(CircularBuffer* buffer, void** item)
{
	// If buffer empty or locked return failure
	if(IS_BUFFER_EMPTY(buffer) || buffer->locked)
	{
		*item = 0;
		return FALSE;
	}
	
	// Lock buffer for editing
	buffer->locked = TRUE;
	
	// read item from buffer
	*item = buffer->buffer[buffer->headIndex];
	
	// Increment head index
	buffer->headIndex = (buffer->headIndex + 1) % buffer->size;
	
	// Can't be full anymore
	//buffer->bufferFull = FALSE;
	buffer->count--;
	
	// Unlock buffer
	buffer->locked = FALSE;
	
	return TRUE;
}

bool BufferReadOffset(CircularBuffer* buffer, void** item, uint8_t offset)
{
	// If buffer empty or locked return failure
	if(IS_BUFFER_EMPTY(buffer) || buffer->locked || offset >= GetNumItems(buffer))
	{
		*item = 0;
		return FALSE;
	}
	
	// Lock buffer for editing
	buffer->locked = TRUE;
	
	// read item from buffer
	*item = buffer->buffer[(buffer->headIndex + offset) % buffer->size];
	
	// Unlock buffer
	buffer->locked = FALSE;
	
	return TRUE;
}

uint8_t GetNumItems(CircularBuffer* buffer)
{
	/*if (IS_BUFFER_EMPTY(buffer))
		return 0;
	
	if (IS_BUFFER_FULL(buffer))
		return buffer->size;
	
	if (buffer->tailIndex > buffer->headIndex)
		return (buffer->tailIndex - buffer->headIndex);
	
	return ((buffer->size - buffer->headIndex) + buffer->tailIndex);*/
  
	return buffer->count;
}
