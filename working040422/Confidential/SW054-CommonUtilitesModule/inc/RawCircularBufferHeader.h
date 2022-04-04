/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/03/2016: Incorporating into common modules
	05/19/2016: Initial creation.
******************************* END VERSION INFO ******************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RAW_CIRC_BUFFER_HEADER
#define RAW_CIRC_BUFFER_HEADER

#include "CommonTypes.h"

#define RawCircularBufferReadAll(circ, ret, rem) RawCircularBufferRead(circ, 0xFFFF, ret, rem)
#define IsRawBufferFull(circ)		((circ)->count >= (circ)->maxLength)
#define IsRawBufferEmpty(circ)		((circ)->count == 0)

typedef struct {
	uint16_t maxLength;
	uint16_t nextWriteIndex;
	uint16_t nextReadIndex;
	uint8_t count;
	bool locked;
	void* buffer;
	uint8_t itemSize;
} RawCircularBuffer;

void RawCircularBufferInit(RawCircularBuffer* buf, void* buffer, uint8_t itemSize, uint16_t maxLength);
bool RawCircularBufferWrite(RawCircularBuffer* buf, uint16_t length, void* data);
uint16_t RawCircularBufferRead(RawCircularBuffer* buf, uint16_t numItems, void* returnBuffer, bool removeFromBuffer);
bool RawCircularBufferPeek(RawCircularBuffer* buf, uint16_t offset, void* returnBuffer);
uint16_t RawCircularBufferGetLength(RawCircularBuffer* buf);
bool RawCircularBufferClear(RawCircularBuffer* buf);
bool RawCircularBufferDelete(RawCircularBuffer* buf, uint16_t numItems);
bool RawCircularBufferTransfer(RawCircularBuffer* inputBuf, RawCircularBuffer* outputBuf, uint16_t maxToTransfer);

//close recursive include ifdef
#endif