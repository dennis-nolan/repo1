/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
	02/06/2019: Removed unusued GetPairedDevices prototype.
******************************* END VERSION INFO ******************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CM2PAIRINGMODE_HEADER
#define CM2PAIRINGMODE_HEADER

//#include "CircularBufferHeader.h"
#include "CM2CoreHeader.h"

void CM2PairingTableModeInit(uint8_t deviceType, uint32_t ID);
void CM2PairingTableModeDeinit(void);
void ProcessNextRxPacketTableMode(CM2Packet* packet);//CircularBuffer* rxBuffer);
void SetPairingTable(uint32_t* storedPairingTable, uint8_t numStored, uint8_t tableLength);
void SetPromiscuousMode(bool enable);
bool IsPaired(uint32_t ID);
bool Pair(uint32_t ID);
bool Unpair(uint32_t ID);
void SetPairingMode(bool enable);

// close recursive include ifdef
#endif