
#include "CM2CoreTypesHeader.h"

uint32_t FlashGetMaxAppSize(void);
bool FlashEraseScratch(void);
bool FlashWriteScratch(uint32_t offset, uint32_t data);

void FlashMockReset(void);
void FlashMockResetFaults(void);
bool FlashMockVerify(uint8_t* app, uint8_t length);