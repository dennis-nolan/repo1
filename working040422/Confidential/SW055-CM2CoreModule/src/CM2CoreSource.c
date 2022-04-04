/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
******************************* END VERSION INFO ******************************/

#include "CM2CoreHeader.h"
#include "ThreadingHeader.h"
#include "TimersHeader.h"
#include "CM2SecurityHeader.h"
#include "CM2PairingTableModeHeader.h"
#include "CM2NetworkModeHeader.h"
#include "CM2RoutingHeader.h"
#include "CM2PortsHeader.h"
#include "CM2RangeTestHeader.h"
#include "RandomHeader.h"

#define ACK_TIMEOUT 20 //ms
#define IDLE_TIMEOUT 100 //ms
#define ACK_TIMEOUT_ID 0x01
#define IDLE_TIMEOUT_ID 0x02

#define PASSIVE_PAIRING	0
#define ACTIVE_PAIRING	1
#define PAIRED			2

#define MAX_EXTENDED_PACKET_RAM_LENGTH 128


static uint8_t pairingState = PASSIVE_PAIRING;
//static uint32_t myID = 0, myNetworkID = 0;///////////////////////
static uint8_t myDeviceType = 0;
static RxPacketResult (*RxCallback) (CM2Packet*) = 0;
static bool flashInterfaceRegistered = FALSE;
static uint32_t flashMaxAppSize = 0;
static bool flashWord64Bit = FALSE;
static bool (*FlashEraseScratch)(void) = 0;
static bool (*FlashWriteScratch)(uint32_t offset, uint32_t data) = 0;
static bool (*FlashWriteScratch64)(uint32_t offset, uint64_t data) = 0;
static bool waitingForAck = FALSE;
static uint16_t macForPendingAck = 0x0000;
static bool idleTimeout = FALSE;
static uint32_t currentExtendedPacket = 0;
static uint32_t extendedPacketNumBytes = 0;
static bool extendedPacketIsFWUpdate = FALSE;
static uint32_t extendedPacketCurrentIndex;
static uint8_t* flattenedExtendedPacket;
static bool isInitialized = FALSE;
static Port internalPort;
static CM2Packet pktResp =
{
	PORT_UNKNOWN,
	PORT_UNKNOWN,	//destination SPIRIT
	0xFF,
	PROTOCOL_VERSION,
	0xFF,				//src type
	DIRECT_ADDRESSING,
	0xFFFFFFFF,				//src ID
	0xFFFFFFFF,		//dest
	0x00,				//no subaddress
	0x03,				//3 byte payload
	0x0000,				//MAC to be filled out
	{OP_CHALLENGE_RESP, 0xFF, 0xFF}			//
};
static CM2Packet ack =
{
	PORT_INTERNAL,
	PORT_UNKNOWN,//destination SPIRIT
	0,
	PROTOCOL_VERSION,
	0x00,		//src type to be filled out in init
  	DIRECT_ADDRESSING,
	0x00000000,	//src ID to be filled out in init
	0x00000000,	//dest ID to be filled out before acking
	0x00,		//no subaddress
	0x03,		//3 byte payload
	0x0000,		//MAC to be filled out before acking
	{OP_LOW_LEVEL_ACK,0,0} //will contain opcode + msg MAC
};
static CM2Packet nack =
{
	PORT_INTERNAL,
	PORT_UNKNOWN,//destination SPIRIT
	0,
	PROTOCOL_VERSION,
	0x00,		//src type to be filled out in init
  	DIRECT_ADDRESSING,
	0x00000000,	//src ID to be filled out in init
	0x00000000,	//dest ID to be filled out before acking
	0x00,		//no subaddress
	0x04,		//4 byte payload
	0x0000,		//MAC to be filled out before acking
	{OP_NACK,0,0,0xFF} //will contain opcode + msg MAC
};
static CM2Packet delayedAck =
{
	PORT_INTERNAL,
	PORT_UNKNOWN,//destination SPIRIT
	0,
	PROTOCOL_VERSION,
	0x00,		//src type to be filled out in init
  	DIRECT_ADDRESSING,
	0x00000000,	//src ID to be filled out in init
	0x00000000,	//dest ID to be filled out before acking
	0x00,		//no subaddress
	0x03,		//3 byte payload
	0x0000,		//MAC to be filled out before acking
	{OP_LOW_LEVEL_ACK,0,0} //will contain opcode + msg MAC
};

static void ResetIdleTimeout(void)
{
	idleTimeout = FALSE;
	RegisterTimeout(&idleTimeout,IDLE_TIMEOUT);
}

bool CM2Busy(void)
{
  /////////////////////////TODO: will need to handle any other non-blocking operations///////////////
	return (bool)(idleTimeout==FALSE);// || IS_BUFFER_EMPTY(rxBuffer)==FALSE);
}

static void DelayedAck(uint8_t dummy)
{
	RoutePacket(&delayedAck);
}

static CM2Packet reconstructed;
static CM2Packet copy;
static int extendedPacketLastPacketNum = -1;
/*static*/ RxPacketResult ProcessAdminPacket(CM2Packet* packet)
{
	uint16_t response, length;
	uint32_t ID;
	uint8_t numBytes;
    uint64_t dtemp2,dtemp3;
    bool status = FALSE;
    Port* tempPort;

	switch(packet->command[0])
	{
		case OP_ACK:
		  	// Check if we were waiting for this
			if (waitingForAck && macForPendingAck == ((packet->command[1]<<8) | packet->command[2]))
			{
				// Mark that we heard back
				waitingForAck = FALSE;
				macForPendingAck = 0x0000;
			}
			break;
		case OP_PING:
			tempPort = GetPort(packet->deviceSource);
			if (tempPort != 0 && tempPort->automaticLLAcks && DIRECT_ADDRESSING != packet->destType)
			{
				delayedAck.deviceDest = packet->deviceSource;
				delayedAck.destID = packet->srcID;
				delayedAck.command[1] = (packet->mac)>>8;
				delayedAck.command[2] = packet->mac;
				delayedAck.mac = CalcMAC(&delayedAck, GetID());
				RegisterTimer(DelayedAck, 0, (GetRandom16() & 63), FALSE, FALSE);
			}
			//else this has already been acked because it was direct
			break;
		case OP_JOIN_REQ:
			// We only care if we are in pairing mode
		  	if (pairingState==PASSIVE_PAIRING || pairingState==ACTIVE_PAIRING)
			{

			}
			break;
               /*
		case OP_JOIN_RESP:
			// If we are trying to pair then update pairing info
			if (pairingState==PASSIVE_PAIRING)
			{
				////////////////////////////// TODO: verify packet structure
			  	ID = (packet->command[1] << 24) | (packet->command[2] << 16) | (packet->command[3] << 8) | packet->command[4];
				//SetNID(ID);
				pairingState = PAIRED;
				////////////////////////////// TODO: save current channel
			}
			break;
                */
		/*case OP_VERSION_REQ:

			break;*/
		case OP_VERSION:

			break;
		case OP_EX_PACKET_START:
			/////////////////////TODO: handle case of extended packet already in progress
			currentExtendedPacket = (packet->command[1]<<24) | (packet->command[2]<<16) | (packet->command[3]<<8) | packet->command[4];
			extendedPacketNumBytes = (packet->command[5]<<16) | (packet->command[6]<<8) | packet->command[7];
			extendedPacketIsFWUpdate = (bool)(packet->command[8] > 0); ////////////////////////////////////TODO - check for bad values
			if (extendedPacketIsFWUpdate == FALSE)
			{
				if (extendedPacketNumBytes < MAX_EXTENDED_PACKET_RAM_LENGTH)
				{
					//flattenedExtendedPacket = (uint8_t*)GetMem(extendedPacketNumBytes);
				}
				//else error //////////////////////////TODO
			}
			else
			{
				//This is a firmware update

				if (flashInterfaceRegistered && extendedPacketNumBytes <= flashMaxAppSize)
				{
					//Force ack
					Ack(packet, FALSE, TRUE);
					//erase the scratch area in preparation for new FW.
					if ((*FlashEraseScratch)()== TRUE)
                                        {
                                          Ack(packet,TRUE,FALSE);
                                        }
                                        else
                                        {
                                          Nack(packet,NACK_FLASHERASEFAIL);
                                        }
				}
				else
				{
					/////////////////////TODO-nowhere to write the data or not enough room - send a nak
				}
			}
			extendedPacketCurrentIndex = 0;
			extendedPacketLastPacketNum = -1;

			break;
		case OP_EX_PACKET_DATA:
			////////////////////TODO: handle packets that are too long for RAM
//version 0000001B                        Ack(packet, FALSE, TRUE);  //V0x0000000D
			if (currentExtendedPacket == (packet->command[1]<<24) | (packet->command[2]<<16) | (packet->command[3]<<8) | packet->command[4])
			{
				//skip packet if we've already handled it
				if (extendedPacketLastPacketNum == ((packet->command[5]<<8) | packet->command[6]))
                                {
                                  Ack(packet,TRUE,FALSE);  //V0x0000000D
                                  break;
                                }
				extendedPacketLastPacketNum = (packet->command[5]<<8) | packet->command[6];

				status = TRUE;
				if (extendedPacketIsFWUpdate == FALSE)
				{
					for(uint8_t i=0; i<(packet->commandLen - 7) && extendedPacketCurrentIndex<extendedPacketNumBytes ; i++)
					{
						flattenedExtendedPacket[extendedPacketCurrentIndex++] = packet->command[i+7];
						////////////////////////////TODO: handle timeouts
					}
				}
				else
				{
					//this is a FW update, so write it to flash
					if (TRUE == flashWord64Bit)
					{
						numBytes = 8;
						for(uint8_t i=0; i<(packet->commandLen - 7) && extendedPacketCurrentIndex<extendedPacketNumBytes ; i+=numBytes)
						{
							dtemp2 = packet->command[i+14];
							dtemp2 = dtemp2<<56;
							dtemp3 = packet->command[i+13];
							dtemp3 = dtemp3<<48;
							dtemp2 |=dtemp3;
							dtemp3 = packet->command[i+12];
							dtemp3 = dtemp3<<40;
							dtemp2 |= dtemp3;
							dtemp3 = packet->command[i+11];
							dtemp3 = dtemp3<<32;
							dtemp2 |= dtemp3;
							dtemp3 = packet->command[i+10];
							dtemp3 = dtemp3<<24;
							dtemp2 |= dtemp3;
							dtemp3 = packet->command[i+9];
							dtemp3 = dtemp3<<16;
							dtemp2 |= dtemp3;
							dtemp3 = packet->command[i+8];
							dtemp3 = dtemp3<<8;
							dtemp2 |= dtemp3;
							dtemp3 = packet->command[i+7];
							dtemp2 |= dtemp3;
							//(*FlashWriteScratch)(extendedPacketCurrentIndex, (packet->command[i+7]<<24) | (packet->command[i+8]<<16) | (packet->command[i+9]<<8) | packet->command[i+10]);
							if ((*FlashWriteScratch64)(extendedPacketCurrentIndex, (dtemp2))!= TRUE)
                                                        {
                                                          status = FALSE;
                                                        }
							extendedPacketCurrentIndex += numBytes;
							////////////////////////////TODO: handle timeouts
						}
					}
					else
					{
						numBytes = 4;
						for(uint8_t i=0; i<(packet->commandLen - 7) && extendedPacketCurrentIndex<extendedPacketNumBytes ; i+=numBytes)
						{
							//(*FlashWriteScratch)(extendedPacketCurrentIndex, (packet->command[i+7]<<24) | (packet->command[i+8]<<16) | (packet->command[i+9]<<8) | packet->command[i+10]);
							if ((*FlashWriteScratch)(extendedPacketCurrentIndex, (packet->command[i+10]<<24) | (packet->command[i+9]<<16) |
                                                                                 (packet->command[i+8]<<8) | packet->command[i+7]) != TRUE)
                                                        {
                                                            status = FALSE;
                                                        }
							extendedPacketCurrentIndex += numBytes;
							////////////////////////////TODO: handle timeouts
						}
					}
                                        if (status == TRUE)
                                        {
                                          if(extendedPacketCurrentIndex < extendedPacketNumBytes-1)
                                          {
                                            Ack(packet,TRUE,FALSE);
                                          }
                                        }
                                        else
                                        {
                                         Nack(packet,NACK_FLASHWRITEFAIL);
                                        }
				}

				if (TRUE == status && extendedPacketCurrentIndex >= extendedPacketNumBytes-1)
				{
					//packet complete

					//////////////////////////TODO-verify and send final ack

					if (extendedPacketIsFWUpdate == FALSE)
					{
						UnflattenCM2PacketStatic(&reconstructed, extendedPacketNumBytes, flattenedExtendedPacket, PORT_SPIRIT, FALSE);//////////////TODO: source
						RoutePacket(&reconstructed);
					}
					else
					{
						/////////////////TODO-check CRC first


						//send a high-level ack
						if (CheckDevice() != 0)
                                                {
                                                  Ack(packet,TRUE,TRUE);
                                                  //reset to check/copy/run the new app
                                                  ResetDevice();
                                                }
                                                else
                                                {
                                                  Nack(packet,NACK_DONWLOADFAIL);
                                                }
					}

					//reset variables
					//extendedPacketCurrentIndex = 0;
					//FreeMem((void**)&flattenedExtendedPacket);
					//flattenedExtendedPacket = 0;
					//extendedPacketNumBytes = 0;
					//extendedPacketIsFWUpdate = FALSE;
				}
			}
			break;
		case OP_CHALLENGE:
		  	//abort any retries we're doing
			AbortAllRetries();

			//create a RESPONSE packet
			response = CalcChallengeResponse((packet->command[1]<<8)|packet->command[2], packet->srcID, GetID());
			/*uint8_t cmdResp[3] = {OP_CHALLENGE_RESP,response>>8,response};
			CM2Packet pktResp =
			{
				PORT_INTERNAL,
				PORT_SPIRIT,	//destination SPIRIT
				PROTOCOL_VERSION,
				0xFF,				//src type
				myID,				//src ID
				DIRECT_ADDRESSING,
				packet->srcID,		//dest
				0x00,				//no subaddress
				0x03,				//3 byte payload
				cmdResp,			//
				0x0000				//MAC to be filled out before acking
			};*/
			pktResp.deviceSource = PORT_INTERNAL;
			pktResp.deviceDest = PORT_SPIRIT;
			pktResp.srcType = myDeviceType;
			pktResp.srcID = GetID();
			pktResp.destID = packet->srcID;
			pktResp.command[1] = response>>8;
			pktResp.command[2] = response;
			pktResp.mac = CalcMAC(&pktResp, GetID());

			CopyPacketStatic(&pktResp, &copy);

			if (!RoutePacket(&copy))////////////////////TODO-copy before this time-critical area
			{
				//FreeMem((void**)&(copy->command));
				//FreeMem((void**)&copy);
			}
			break;
		/*case OP_CHALLENGE_RESP:
			if (pairingMode)
			{
				//check response & add to table
				if (((packet->command[1]<<8) | packet->command[2]) == CalcChallengeResponse(latestChallenge, myID, packet->srcID))
					Pair(packet->srcID);
				//else they got it wrong. ignore.
			}
			break;*/
		case OP_RANGE_TEST:
			ID = (packet->command[2]<<24)|(packet->command[3]<<16)|(packet->command[4]<<8)|(packet->command[5]);
			length = (packet->command[6]<<8)|(packet->command[7]);
			if (length == 0 || packet->command[1] != 0) //not currently supporting group call range tests
				AbortRangeTest();
			else
				BeginRangeTest(ID,length,packet->command[8],packet->command[9]);
			break;
		default:
			//unknown admin opcode
			return RX_PACKET_ERR_OPCODE;
			break;
	}

	return RX_PACKET_HANDLED;
}

static uint32_t firstTryFail = 0;
static uint32_t acksSent = 0;
void Ack(CM2Packet* packet, bool highLevel, bool blocking)
{
	Port* tempPort = GetPort(packet->deviceSource);
	if (tempPort->handleAckingCallback != 0)
	{
		tempPort->handleAckingCallback(packet, highLevel?OP_ACK:OP_LOW_LEVEL_ACK, 0, blocking);
		return;
	}

	//populate the packet details
	//ack.deviceSource = PORT_INTERNAL;
	ack.deviceDest = packet->deviceSource;
	ack.destID = packet->srcID;

	if (highLevel)
        {
		ack.command[0] = OP_ACK;
//                ack.retries = 2; //v0x000000E
        }
	else
		ack.command[0] = OP_LOW_LEVEL_ACK;

	ack.srcID = GetID();
	ack.command[1] = (packet->mac)>>8;
	ack.command[2] = packet->mac;
	ack.mac = CalcMAC(&ack, GetID());

	//send the ack
	if (blocking)
	{
		TxOnSpecifiedPort(&ack, TRUE);
	}
	else if (highLevel)
	{
		RoutePacket(&ack);
	}
	else
	{
		if (TX_FAIL == TxOnSpecifiedPort(&ack, FALSE))
		{
			firstTryFail++;
			RoutePacket(&ack);
		}
	}
	acksSent++;
}

void Nack(CM2Packet* packet, uint8_t reasonCode)
{
	Port* tempPort = GetPort(packet->deviceSource);
	if (tempPort->handleAckingCallback != 0)
	{
		tempPort->handleAckingCallback(packet, OP_NACK, reasonCode, FALSE);
		return;
	}

	//populate the packet details
	//nack.deviceSource = PORT_INTERNAL;
	nack.deviceDest = packet->deviceSource;
	nack.destID = packet->srcID;
	nack.srcID = GetID();

//        nack.retries = 2; //V0x0000000E
	nack.command[1] = (packet->mac)>>8;
	nack.command[2] = packet->mac;
	nack.command[3] = reasonCode;
	nack.mac = CalcMAC(&nack, nack.srcID);

	//send the nack
	//TxOnSpecifiedPort(&nack);
	RoutePacket(&nack);
}

/*static*/ void RunCM2CoreBackgroundTasks(void)
{
	// Process next packet. Handle protocol packets here, forward command packets up
	//ProcessNextRxPacketNetworkMode();/////////////////////////TODO-handle switching
	//ProcessNextRxPacketTableMode(rxBuffer);

//UpdateChallenge();

	// Have the packet router process the next buffered packet
	RouteNextPacket();
}

void CM2CoreInit(void)//uint32_t ID, uint8_t deviceType, uint32_t networkID)
{
	if (isInitialized)
		return;

	// Create RX buffer
	////////rxBuffer = BufferInit(4);

	// Register background task
	RegisterThread(RunCM2CoreBackgroundTasks, 0x0F);

	// Start idle timer
	ResetIdleTimeout();

	isInitialized = TRUE;
}

void CM2CoreDeinit(void)
{
	if (isInitialized)
	{
		//BufferDeinit(rxBuffer, TRUE);
		RouterDeinit();

		////////////////////////TODO-unregister thread, cancel idle timer

		isInitialized = FALSE;
	}

	//CM2SecurityDeinit();
	CM2PairingTableModeDeinit();
}

static TxStatus RxDataAvailable(CM2PortType portType, CM2Packet* packet, bool dummy)
{
	// This call is made during the RX interrupt, so keep processing to a minimum
	/*if (IS_PACKET_VALID(packet))
	{
		ResetIdleTimeout();
		BufferWrite(rxBuffer, packet);
	}

	return TRUE;*/

	// Process next packet. Handle protocol packets here, forward command packets up
	//ProcessNextRxPacketNetworkMode();/////////////////////////TODO-handle switching
	ProcessNextRxPacketTableMode(packet);
	return TX_SUCCESS;
}

void CM2CoreConfigure(uint32_t ID, uint8_t deviceType, uint32_t networkID, uint32_t* storedPairingTable, uint8_t numStored, uint8_t tableLength)
{
	//myID = ID;
	myDeviceType = deviceType;
	//myNetworkID = networkID;
	ack.srcID = ID;
	ack.srcType = deviceType;
	nack.srcID = ID;
	nack.srcType = deviceType;
	delayedAck.srcID = ID;
	delayedAck.srcType = deviceType;

	RouterInit();

	internalPort.portType = PORT_INTERNAL;
	internalPort.Transmit = RxDataAvailable;
	internalPort.Ready = 0;
	RegisterPort(&internalPort);

	CM2PairingTableModeInit(myDeviceType, ID);
	SetPairingTable(storedPairingTable, numStored, tableLength);

	//CM2NetworkModeInit(deviceType, ID, networkID);
}

/*uint32_t GetID(void)
{
	return myID;
}

uint32_t GetNID(void)
{
	return myNetworkID;
}*/

uint8_t GetType(void)
{
	return myDeviceType;
}

void RegisterPacketRxCallback(RxPacketResult (*callback)(CM2Packet*))
{
	RxCallback = callback;
}

void RegisterFlashInterface(uint32_t maxAppSize, bool (*EraseScratch)(void), bool (*WriteScratch)(uint32_t offset, uint32_t data))
{
	flashMaxAppSize = maxAppSize;
	FlashEraseScratch = EraseScratch;
	FlashWriteScratch = WriteScratch;

	if (maxAppSize > 0 && FlashEraseScratch != 0 && FlashWriteScratch != 0)
		flashInterfaceRegistered = TRUE;
}

void RegisterFlashInterface64(uint32_t maxAppSize, bool (*EraseScratch)(void), bool (*WriteScratch)(uint32_t offset, uint64_t data), bool use64BitWrites)
{
	flashMaxAppSize = maxAppSize;
	FlashEraseScratch = EraseScratch;
	FlashWriteScratch64 = WriteScratch;
	flashWord64Bit = use64BitWrites;

	if (maxAppSize > 0 && FlashEraseScratch != 0 && FlashWriteScratch64 != 0)
		flashInterfaceRegistered = TRUE;
}

RxPacketResult ExecutePacketRxCallback(CM2Packet* packet)
{
	if (RxCallback != 0)
		return RxCallback(packet);
	return RX_PACKET_UNHANDLED;
}

bool CheckPacket(CM2Packet* packet, uint32_t NID)
{
	if (!IS_PACKET_VALID(packet))
		return FALSE;

	if (!IsPacketForMe(packet))
		return FALSE;

	if (!IS_VERSION_COMPATIBLE(packet->protocolVer))
		return FALSE;

	if (PORT_INTERNAL == packet->deviceSource)
		return TRUE; //we generated the packet, don't worry about MAC
	else
		return IsMacCorrect(packet, NID);
}

bool IsPacketForMe(CM2Packet* packet)
{
	if (!IS_PACKET_VALID(packet))
		return FALSE;

	Port* tempPort = GetPort(packet->deviceSource);
	if (tempPort == 0)
		return FALSE;

	if (TRUE == tempPort->filtering.flags.destIDFiltering &&
	    packet->destType == DIRECT_ADDRESSING && packet->destID != tempPort->filtering.ID)
		return FALSE;

	if (TRUE == tempPort->filtering.flags.destTypeFiltering &&
	    IS_TYPE_ADDRESSING(packet->destType) && packet->destType != tempPort->filtering.deviceType && packet->destType != (tempPort->filtering.deviceType & 0xF0))
		return FALSE;

	if (TRUE == tempPort->filtering.flags.subaddrFiltering &&
	    packet->subaddress != tempPort->filtering.subaddress)
		return FALSE;

	return TRUE;
}

