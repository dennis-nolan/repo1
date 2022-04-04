
#include "FlashMocks.h"

#define APP_SIZE 128
static uint8_t mockScratchSpace[APP_SIZE];
static bool scratchByteDirty[APP_SIZE];
bool invalidWriteOcurred = FALSE;
bool invalidAddress = FALSE;

void FlashMockReset(void)
{
	FlashEraseScratch();
	FlashMockResetFaults();
}

void FlashMockResetFaults(void)
{
	invalidWriteOcurred = FALSE;
	invalidAddress = FALSE;
}

uint32_t FlashGetMaxAppSize(void)
{
	return APP_SIZE;
}

bool FlashEraseScratch(void)
{
	for (uint8_t i=0; i<APP_SIZE; i++)
	{
		mockScratchSpace[i] = 0xCD;
		scratchByteDirty[i] = FALSE;
	}
	return TRUE;
}

bool FlashWriteScratch(uint32_t offset, uint32_t data)
{
	if ((offset >= APP_SIZE-4) || (offset % 4 != 0))
	{
		invalidAddress = TRUE;
		return FALSE;
	}
	
	if (scratchByteDirty[offset] || scratchByteDirty[offset+1] || scratchByteDirty[offset+2] || scratchByteDirty[offset+3])
	{
		//trying to write to a byte that was not erased.
		invalidWriteOcurred = TRUE;
		return FALSE; ////////////should this return false or just corrupt the data?
	}
	
	mockScratchSpace[offset] = data>>24;
	mockScratchSpace[offset+1] = data>>16;
	mockScratchSpace[offset+2] = data>>8;
	mockScratchSpace[offset+3] = data;
	
	scratchByteDirty[offset] = TRUE;
	scratchByteDirty[offset+1] = TRUE;
	scratchByteDirty[offset+2] = TRUE;
	scratchByteDirty[offset+3] = TRUE;
	
	return TRUE;
}

bool FlashMockVerify(uint8_t* app, uint8_t length)
{
	for (uint8_t i=0; i<length; i++)
	{
		if (app[i] != mockScratchSpace[i])
			return FALSE;
	}
	return TRUE;
}