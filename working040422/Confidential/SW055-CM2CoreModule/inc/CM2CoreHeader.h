/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
******************************* END VERSION INFO ******************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CM2CORE_HEADER
#define CM2CORE_HEADER

#include "CM2CoreTypesHeader.h"

#define PROTOCOL_VERSION 0x01 //4 bits major version, 4 bits minor version
#define IS_VERSION_COMPATIBLE(x) ((x&0xF0) == (PROTOCOL_VERSION&0xF0))
#define CHECK_MAC_BEFORE_ACK

void RunCM2CoreBackgroundTasks(void);
bool CM2Busy(void);
RxPacketResult ProcessAdminPacket(CM2Packet* packet);
void Ack(CM2Packet* packet, bool highLevel, bool blocking);
void Nack(CM2Packet* packet, uint8_t reasonCode);
void CM2CoreInit(void);
void CM2CoreDeinit(void);
void CM2CoreConfigure(uint32_t ID, uint8_t deviceType, uint32_t networkID, uint32_t* pairingTable, uint8_t numStored, uint8_t tableLength);
void RegisterPacketRxCallback(RxPacketResult (*CommandPacketRX)(CM2Packet*));
void RegisterFlashInterface(uint32_t maxAppSize, bool (*EraseScratch)(void), bool (*WriteScratch)(uint32_t offset, uint32_t data));
void RegisterFlashInterface64(uint32_t maxAppSize, bool (*EraseScratch)(void), bool (*WriteScratch)(uint32_t offset, uint64_t data), bool use64BitWrites);
RxPacketResult ExecutePacketRxCallback(CM2Packet* packet);
bool CM2TxBlocking(CM2Packet* packet);
bool CheckPacket(CM2Packet* packet, uint32_t NID);
bool IsPacketForMe(CM2Packet* packet);
uint8_t GetType(void);
void StartRangeTest(uint8_t destType, uint32_t destID, uint16_t length, uint8_t spacing);
void StopRangeTest(void);
bool IsRangeTestActive(void);
void SetUnpairedPacketCallback(void (*UnpairedPacketRX)(CM2Packet*));
void UnSetUnpairedPacketCallback(void);
/*void RegisterFlatPacketCallback(void (*FlatPacketCallback)(uint8_t*));
bool IsFlatPacketCallbackAvailable(void);
void CallFlatPacketCallback(RawCircularBuffer* buf);*/


/***** Required interface from the main project *****/
void ResetDevice(void);
void DeviceReset(void);
uint8_t CheckDevice(void);
uint32_t GetID(void);

// close recursive include ifdef
#endif