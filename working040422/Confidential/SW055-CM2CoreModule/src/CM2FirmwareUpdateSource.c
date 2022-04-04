
#include "CM2FirmwareUpdateHeader.h"
#include "CM2CoreTypesHeader.h"
#include "CM2RoutingHeader.h"
#include "CM2SecurityHeader.h"
#include "CM2CoreHeader.h"
#include "TimersHeader.h"

#ifndef FW_UPDATE_TIMEOUT
#pragma message("FW_UPDATE_TIMEOUT is not defined. Defaulting to 10000.")
#define FW_UPDATE_TIMEOUT 10000 // 10 seconds
#endif

FW_SESSION_INFO fwSessionInfo; 

enum {STATUS_NONE=0,STATUS_WAIT_LL,STATUS_WAIT_HL,STATUS_FAIL,STATUS_SUCCESS};


static CM2Packet fwUpdateStart = 
{
	PORT_UNKNOWN,                         //packet source
	PORT_UNKNOWN,                           //destination SPIRIT
	0,
	1,//PROTOCOL_VERSION,                      //protocol ver
	0x80,//DEVICE_TYPE,                           //src type
	0,                                     //dest type
	0,                                     //src ID
	0x001BFB8E,                            //dest ID
	0x00,                                  //subaddress
	9,                                     //length
	0x0000,                                //MAC to be filled out before acking
	{OP_EX_PACKET_START,0xA,0xB,0xC,0xD,0,0x0F,0x84,1}   //command/payload
};
static CM2Packet fwUpdateData = 
{
	PORT_UNKNOWN,                         //packet source
	PORT_UNKNOWN,                           //destination SPIRIT
	0,//0xFF,
	1,//PROTOCOL_VERSION,                      //protocol ver
	0x80,//DEVICE_TYPE,                           //src type
	0,                                     //dest type
	0,                                     //src ID
	0x001BFB8E,                            //dest ID
	0x00,                                  //subaddress
	0x00,                                  //length
	0x0000,                                //MAC to be filled out before acking
	{OP_EX_PACKET_DATA,0xA,0xB,0xC,0xD,0}                  //command/payload
};
static uint8_t updateStatus = UPDATE_STATUS_NONE;
static uint8_t lastUpdatePacketStatus = STATUS_NONE;
static uint8_t* nextByte;
static CM2Packet* lastUpdatePacket = 0;
static uint32_t updatePacketNum = 0;
static uint32_t lastByteAddr = 0;
static bool updateTimeout = FALSE;
static uint32_t updateID = 0xAABBCCDD;
static uint32_t lastStateChange = 0;

static uint16_t timeoutFails = 0;
static uint16_t lowlevelFails = 0;
static uint16_t highlevelFails = 0;


uint32_t RunFirmwareGetBytesRemaining(void)
{
	uint32_t bytesRemaining = lastByteAddr - (uint32_t)nextByte;
		return bytesRemaining;
}

void SetFirmwareUpdateStatusNone(void)
{ updateStatus = UPDATE_STATUS_NONE; }

uint8_t GetFirmwareUpdateStatus(void)
{ return updateStatus; }

void AbortFirmwareUpdate(void)
{ updateStatus = UPDATE_STATUS_FAIL; }

static void SetPacketStatus(uint8_t newStatus)
{
	if (lastUpdatePacketStatus == newStatus)
		return;
	
	lastUpdatePacketStatus = newStatus;
	lastStateChange = GetMsSinceStart();
}

void FWUpdateAckResult(uint16_t mac, bool highLevel, bool success, uint8_t reason)
{
	if (mac != lastUpdatePacket->mac)
		return;
	
	if (success)
	{
		if (highLevel)
                {  
			SetPacketStatus(STATUS_SUCCESS);
                }        
		else
                {  
                  if (fwSessionInfo.timeoutHLACK == 0)
                  {
                    SetPacketStatus(STATUS_SUCCESS);
                  }
                  else
                  {
                    SetPacketStatus(STATUS_WAIT_HL);
                  }      
                }        
	}
	else
	{
          if (reason == NACK_STOPDOWNLOAD)
          {
            updateStatus = UPDATE_STATUS_FAIL;
          }
          else
          {
            SetPacketStatus(STATUS_FAIL);
          }  
	}
}

void BeginFirmwareUpdate(FW_SESSION_INFO *sessionInfo)
{
	if (updateStatus == UPDATE_STATUS_IN_PROGRESS)
		return;
	
        fwSessionInfo.destID = sessionInfo->destID;
        fwSessionInfo.binaryStartAddr = sessionInfo->binaryStartAddr;
        fwSessionInfo.binaryLength = sessionInfo->binaryLength;
        fwSessionInfo.deviceDest = sessionInfo->deviceDest;
        fwSessionInfo.destType = sessionInfo->destType;
        fwSessionInfo.sourceType = sessionInfo->sourceType;
        fwSessionInfo.timeoutLLACK = sessionInfo->timeoutLLACK;
        fwSessionInfo.timeoutHLACK = sessionInfo->timeoutHLACK;             
          
	//reset all update variables
	updateStatus = UPDATE_STATUS_NONE;
	nextByte = (uint8_t*)sessionInfo->binaryStartAddr;
	updatePacketNum = 0;
	lastByteAddr = sessionInfo->binaryStartAddr + sessionInfo->binaryLength;
	lowlevelFails = 0;
	highlevelFails = 0;
	
	//generate update ID
	uint32_t myID = GetID();
	updateID = (myID << 16) + Rand16();
	
	//fill out data packet header
        fwUpdateData.retries = 0;  //BETH
	fwUpdateData.deviceSource = PORT_INTERNAL;
	fwUpdateData.deviceDest = (CM2PortType)sessionInfo->deviceDest;
	fwUpdateData.srcID = myID;
	fwUpdateData.srcType = sessionInfo->sourceType;
	fwUpdateData.destID = sessionInfo->destID;
	fwUpdateData.destType = sessionInfo->destType;
	fwUpdateData.command[1] = updateID >> 24;
	fwUpdateData.command[2] = updateID >> 16;
	fwUpdateData.command[3] = updateID >> 8;
	fwUpdateData.command[4] = updateID;
	
	//fill out start packet
	fwUpdateStart.deviceSource = PORT_INTERNAL;
	fwUpdateStart.deviceDest = (CM2PortType)sessionInfo->deviceDest;
	fwUpdateStart.srcID = myID;
	fwUpdateStart.srcType = sessionInfo->sourceType;
	fwUpdateStart.destID = sessionInfo->destID;
	fwUpdateStart.destType = sessionInfo->destType;
	fwUpdateStart.command[1] = updateID >> 24;
	fwUpdateStart.command[2] = updateID >> 16;
	fwUpdateStart.command[3] = updateID >> 8;
	fwUpdateStart.command[4] = updateID;
	fwUpdateStart.command[5] = (sessionInfo->binaryLength >> 16);
	fwUpdateStart.command[6] = (sessionInfo->binaryLength >> 8);
	fwUpdateStart.command[7] = sessionInfo->binaryLength;
	fwUpdateStart.mac = CalcMAC(&fwUpdateStart, fwUpdateData.srcID);
	
	//send packet
	lastUpdatePacket = &fwUpdateStart;
	if (RoutePacket(&fwUpdateStart))
		SetPacketStatus(STATUS_WAIT_LL);
	else
		SetPacketStatus(STATUS_FAIL);
	
	//update timeout
	updateTimeout = FALSE;
	RegisterTimeout(&updateTimeout, FW_UPDATE_TIMEOUT);
	
	//allow RunFirmwareUpdateTask to run by setting updateStatus
	updateStatus = UPDATE_STATUS_IN_PROGRESS;
}

void RunFirmwareUpdateTask(void)
{
 
	//do nothing if an update isn't started
	if (updateStatus != UPDATE_STATUS_IN_PROGRESS)
		return;
	
	//Handle update timeout
	if (updateTimeout)
	{
		updateStatus = UPDATE_STATUS_FAIL;
		timeoutFails++;
		return;
	}
	
	//handle waiting on acks
	if (STATUS_WAIT_LL == lastUpdatePacketStatus)
	{
		if (GetMsElapsed(lastStateChange) < fwSessionInfo.timeoutLLACK)
			return;
		else
		{
			SetPacketStatus(STATUS_FAIL);
			lowlevelFails++;
		}
	}
	else if (STATUS_WAIT_HL == lastUpdatePacketStatus)
	{
		if (GetMsElapsed(lastStateChange) < fwSessionInfo.timeoutHLACK)
			return;
		else
		{
			SetPacketStatus(STATUS_FAIL);
			highlevelFails++;
		}
	}
	
	//if the last packet failed try it again
	if (STATUS_FAIL == lastUpdatePacketStatus)
	{
		if (RoutePacket(lastUpdatePacket))
			SetPacketStatus(STATUS_WAIT_LL);
		else
			SetPacketStatus(STATUS_FAIL);
		return;
	}
	
	//continuing forward - reset the timeout
	RegisterTimeout(&updateTimeout, FW_UPDATE_TIMEOUT);
	
	//check to see if we're done
	uint32_t bytesRemaining = lastByteAddr - (uint32_t)nextByte;
	if (bytesRemaining == 0)
	{
		updateStatus = UPDATE_STATUS_SUCCESS;
		return;
	}
	
	//send next data packet
	uint8_t chunkSize = 56;
	if (bytesRemaining < chunkSize)
	{
		chunkSize = bytesRemaining;
		while (chunkSize%8 != 0)
		{ chunkSize++; }
	}
	
	fwUpdateData.commandLen = 7+chunkSize;
	fwUpdateData.command[5] = (updatePacketNum >> 8);
	fwUpdateData.command[6] = updatePacketNum & 0xFF;
	updatePacketNum++;
	
	for (int i = 0; i < chunkSize; i++)
	{
		if ((uint32_t)nextByte <= lastByteAddr)
			fwUpdateData.command[7+i] = *nextByte;
		else
			fwUpdateData.command[7+i] = 0;
		nextByte++;
	}
	fwUpdateData.mac = CalcMAC(&fwUpdateData, GetID());
	
	lastUpdatePacket = &fwUpdateData;
	if (RoutePacket(&fwUpdateData))
		SetPacketStatus(STATUS_WAIT_LL);
	else
		SetPacketStatus(STATUS_FAIL);
}