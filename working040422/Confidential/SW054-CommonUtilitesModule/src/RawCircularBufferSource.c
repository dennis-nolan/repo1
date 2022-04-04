/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/03/2016: Incorporating into common modules
	05/19/2016: Initial creation.
******************************* END VERSION INFO ******************************/

#include <stdlib.h>
#include "RawCircularBufferHeader.h"

//assumes that the buffer itself is already initialized
//void RawCircularBufferInit(RawCircularBuffer* buf, uint16_t maxLength)
void RawCircularBufferInit(RawCircularBuffer* buf, void* buffer, uint8_t itemSize, uint16_t maxLength)
{
	buf->maxLength = maxLength;
	buf->nextWriteIndex = 0;
	buf->nextReadIndex = 0;
	buf->count = 0;
	buf->locked = FALSE;
	buf->buffer = buffer;
	buf->itemSize = itemSize;
}

//returns false if full
static bool RawCircularBufferWriteLocked(RawCircularBuffer* buf, uint16_t length, void* data)
{
	//loop through item indexes
	for (uint16_t i=0; i<length; i++)
	{
		if (IsRawBufferFull(buf))
		{
			//unable to write all items
			buf->locked = FALSE;
			return FALSE;
		}
		
		//drop in the next item. loop through bytes.
		for (uint8_t b = 0; b < buf->itemSize; b++)
		{
			//add next byte
			((uint8_t*)buf->buffer)[buf->nextWriteIndex * buf->itemSize + b] = ((uint8_t*)data)[i * buf->itemSize + b];
		}
		//buf->buffer[buf->nextWriteIndex * buf->itemSize] = data[i * buf->itemSize];
		
		//update nextWriteIndex and count
		buf->nextWriteIndex = (buf->nextWriteIndex+1)%(buf->maxLength);
		buf->count++;
	}
	
	//we were able to write all of it
	return TRUE;
}

bool RawCircularBufferWrite(RawCircularBuffer* buf, uint16_t length, void* data)
{
	if (buf->locked)
		return FALSE;
	buf->locked = TRUE;
	
	bool result = RawCircularBufferWriteLocked(buf, length, data);
	
	buf->locked = FALSE;
	return result;
}

//reads bytes from the buffer, and removes them if removeFromBuffer is set
//returns number of bytes actually provided. min(bytesRequested, available, maxLength)
static uint16_t RawCircularBufferReadLocked(RawCircularBuffer* buf, uint16_t numItems, void* returnBuffer, bool removeFromBuffer)
{
	uint16_t localNextReadIndex = buf->nextReadIndex;
	
	if (numItems > buf->count)
		numItems = buf->count;
	
	//loop until we hit the read limit or the buffer is empty
	uint16_t returnBufIndex;
	for (returnBufIndex = 0; returnBufIndex < numItems && !IsRawBufferEmpty(buf); returnBufIndex++)
	{
		//copy item. loop through bytes.
		for (uint8_t b = 0; b < buf->itemSize; b++)
		{
			//add next byte
			((uint8_t*)returnBuffer)[returnBufIndex * buf->itemSize + b] = ((uint8_t*)buf->buffer)[localNextReadIndex * buf->itemSize + b];
		}
		//returnBuffer[returnBufIndex] = buf->buffer[buf->nextReadIndex];
		
		//update location in buffer
		localNextReadIndex = (localNextReadIndex+1)%(buf->maxLength);
		
		//save location in buffer, if removing bytes
		if (removeFromBuffer)
		{
			buf->nextReadIndex = localNextReadIndex;
			//buf->full = FALSE;
			buf->count--;
		}
	}
	
	//all done
	return returnBufIndex;
}

uint16_t RawCircularBufferRead(RawCircularBuffer* buf, uint16_t numItems, void* returnBuffer, bool removeFromBuffer)
{
	if (buf->locked)
		return 0;
	buf->locked = TRUE;

	uint16_t returnBufIndex = RawCircularBufferReadLocked(buf,numItems,returnBuffer,removeFromBuffer);
	
	buf->locked = FALSE;
	return returnBufIndex;
}

bool RawCircularBufferPeek(RawCircularBuffer* buf, uint16_t offset, void* returnBuffer)
{
	if (offset > buf->count)
		return FALSE;
	
	if (buf->locked)
		return FALSE;
	buf->locked = TRUE;
	
	uint16_t startIndex = ((buf->nextReadIndex + offset)%(buf->maxLength)) * buf->itemSize;
	
	//copy item. loop through bytes.
	for (uint8_t b = 0; b < buf->itemSize; b++)
	{
		//add next byte
		
		((uint8_t*)returnBuffer)[b] = ((uint8_t*)buf->buffer)[startIndex + b];
	}
	
	//all done
	buf->locked = FALSE;
	return TRUE;
}

uint16_t RawCircularBufferGetLength(RawCircularBuffer* buf)
{
	/*if (IsRawBufferFull(buf))
		return buf->maxLength;
	else if (IsRawBufferEmpty(buf))
		return 0;
	else if (buf->nextWriteIndex > buf->nextReadIndex)
		return (buf->nextWriteIndex - buf->nextReadIndex);
	else
		return (buf->maxLength + buf->nextWriteIndex - buf->nextReadIndex);*/
	return buf->count;
}

bool RawCircularBufferClear(RawCircularBuffer* buf)
{
	if (buf->locked)
		return FALSE;
	buf->locked = TRUE;
	
	//reset back to empty
	buf->nextReadIndex = buf->nextWriteIndex;
	buf->count = 0;
	
	//all done
	buf->locked = FALSE;
	return TRUE;
}

bool RawCircularBufferDelete(RawCircularBuffer* buf, uint16_t numItems)
{
	if (buf->locked)
		return FALSE;
	buf->locked = TRUE;
	
	if (numItems >= buf->count)
	{
		//reset back to empty
		buf->nextReadIndex = buf->nextWriteIndex;
		buf->count = 0;
	}
	else
	{
		buf->nextReadIndex = (buf->nextReadIndex + numItems)%(buf->maxLength);
		buf->count -= numItems;
	}
	
	//all done
	buf->locked = FALSE;
	return TRUE;
}

bool RawCircularBufferTransfer(RawCircularBuffer* inputBuf, RawCircularBuffer* outputBuf, uint16_t maxToTransfer)
{
	if (inputBuf->itemSize != outputBuf->itemSize)
		return FALSE;
	  
	if (inputBuf->locked || outputBuf->locked)
		return FALSE;
	inputBuf->locked = TRUE;
	outputBuf->locked = TRUE;
	
	uint8_t transferNum = outputBuf->maxLength - outputBuf->count;
	
	if (transferNum > inputBuf->count)
		transferNum = inputBuf->count;
	
	if (transferNum > maxToTransfer)
		transferNum = maxToTransfer;
	
	bool success = TRUE;
	void* temp = (void*)malloc(inputBuf->itemSize);
	for (uint16_t i=0; i<transferNum; i++)
	{
		if (!RawCircularBufferReadLocked(inputBuf,1,temp,TRUE) || !RawCircularBufferWriteLocked(outputBuf,1,temp))
		{
			success = FALSE;
			break;
		}
	}
	free(temp);
	
	//all done
	inputBuf->locked = FALSE;
	outputBuf->locked = FALSE;
	return success;
}
