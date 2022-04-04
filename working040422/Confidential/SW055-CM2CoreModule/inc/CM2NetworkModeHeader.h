/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
******************************* END VERSION INFO ******************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CM2NETWORKMODE_HEADER
#define CM2NETWORKMODE_HEADER

//#include "CircularBufferHeader.h"
#include "CM2CoreHeader.h"

void CM2NetworkModeInit(uint8_t deviceType, uint32_t ID, uint32_t NID);
void CM2NetworkModeDeinit(void);
//void ProcessNextRxPacketNetworkMode(CircularBuffer* rxBuffer);

// close recursive include ifdef
#endif