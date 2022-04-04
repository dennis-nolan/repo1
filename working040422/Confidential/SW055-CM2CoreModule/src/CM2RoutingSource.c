/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/10/2016: Fixed bug in RouteNextPacket related to acking that duplicated received packets
	08/08/2016: Updated ack/retry system for static memory
	08/02/2016: Fixed bug preventing retries when broadcasting.
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	10/07/2015: Initial creation.
******************************* END VERSION INFO ******************************/

#include "CM2RoutingHeader.h"
#include "RawCircularBufferHeader.h"
#include "CM2CoreHeader.h"
#include "CM2SecurityHeader.h"
#include "TimersHeader.h"
//#include "CM2CoreTypesHeader.h"
#include "CM2PortsHeader.h"
#include "CM2FirmwareUpdateheader.h"
#include "CM2RangeTestHeader.h"
#include "CM2PortsHeader.h"

#define INPUT_BUFFER_SIZE		16
#define RETRY_BUFFER_SIZE		8
#define ACK_TIMEOUT			60  //BETH 30//20 //ms

#ifndef DEFAULT_NUM_RETRIES
#pragma message("DEFAULT_NUM_RETRIES is not defined. Defaulting to 5.")
#define DEFAULT_NUM_RETRIES		5
#endif

#define DUPLICATE_TIMEOUT		(ACK_TIMEOUT*(DEFAULT_NUM_RETRIES+1)) //ms
//#define MAX_TX_ATTEMPTS			3
#define MIN_TIME_BETWEEN_TX		10 //ms

static CM2Packet actualInputBuffer[INPUT_BUFFER_SIZE];
static RawCircularBuffer inputBuffer;
static CM2Packet retryDataBuffer[RETRY_BUFFER_SIZE];
static CM2Packet packetTemp;
static uint32_t lastTxTick = 0;

void RouterInit(void)
{
	// Create circular buffer
	//inputBuffer = BufferInit(INPUT_BUFFER_SIZE);
	RawCircularBufferInit(&inputBuffer, (void*)&actualInputBuffer, sizeof(CM2Packet), INPUT_BUFFER_SIZE);
	
	// Clear ack buffer
	/*for (uint8_t i=0; i<RETRY_BUFFER_SIZE; i++)
	{
		retryDataBuffer[i].mac = 0;
		retryDataBuffer[i].ports = 0xFFFF;
	}*/
	
	PortHandlingInit();
}

void RouterDeinit(void)
{
	/*while(BufferRead(inputBuffer, (void**)&packetTemp))
	{
		FreeMem((void**)&packetTemp->command);
		FreeMem((void**)&packetTemp);
	}
	BufferDeinit(inputBuffer, FALSE);
	inputBuffer = 0;*/
	
	for (uint8_t i=0; i<RETRY_BUFFER_SIZE; i++)
	{
		retryDataBuffer[i].mac = 0;
		retryDataBuffer[i].retries = 0;
		
		/*if (retryDataBuffer[i].originalPacket != 0)
		{
			FreeMem((void**)&retryDataBuffer[i].originalPacket->command);
			FreeMem((void**)&retryDataBuffer[i].originalPacket);
		}*/
	}
	
	/*for (uint8_t i=0; i<MAX_NUM_PORTS; i++)
	{
		if (ports[i] != 0)
		{
			//FreeMem((void**)&ports[i]);
			ports[i] = 0;
		}
	}*/
	//////////////////// TODO - port deinit
}

/*void RegisterPort(Port* portInfo)
{
	// verify validity
	//////////////////////TODO
	
	// find an empty slot to put it in
	for (uint8_t i=0; i<MAX_NUM_PORTS; i++)
	{
		if (ports[i] == 0)
		{
			ports[i] = portInfo;
			return;
		}
	}
	
	//if we got here we ran out of ports
	//////////////////////TODO - log error
}*/

bool IsRouterQueueFull(void)
{
	return (bool)IsRawBufferFull(&inputBuffer);
}

static void (*AckResultCallback)(uint16_t,bool) = 0;
void RegisterAckResultCallback(void (*callback)(uint16_t mac, bool success))
{
	AckResultCallback = callback;
}

void AbortRetries(uint16_t mac)
{
	for(uint8_t i=0; i<RETRY_BUFFER_SIZE; i++)
	{
		if (retryDataBuffer[i].mac == mac)
		{
			//clear ack data
			//FreeMem((void**)&retryDataBuffer[i].originalPacket->command);
			//FreeMem((void**)&retryDataBuffer[i].originalPacket);
			retryDataBuffer[i].mac = 0;
			
			//done
//EMH			return;// TRUE;
		}
	}
	
	//couldn't find it
	//return FALSE;
}

void AbortAllRetries(void)
{
	for(uint8_t i=0; i<RETRY_BUFFER_SIZE; i++)
	{
		retryDataBuffer[i].mac = 0;
	}
}

void AbortSimilar(CM2Packet* packet)
{
	for(uint8_t i=0; i<RETRY_BUFFER_SIZE; i++)
	{
		if (retryDataBuffer[i].mac == 0)
			continue;
		else if (retryDataBuffer[i].destType == packet->destType &&
				 retryDataBuffer[i].destID == packet->destID &&
				 retryDataBuffer[i].command[0] == packet->command[0])
		{
			//if this is an OP_COMM then check for a subopcode match
			if (packet->command[0] == OP_COMM && retryDataBuffer[i].command[1] != packet->command[1])
				continue; //subopcode mismatch
			
			//clear ack data
			retryDataBuffer[i].mac = 0;
		}
	}
}

static void RetryExpiration(uint8_t retryIndex)
{
	//see if this has already been cleared
	if (retryDataBuffer[retryIndex].mac == 0)
		return;
	
	//ack has timed out. check to see if we should retry.
	if (retryDataBuffer[retryIndex].retries > 0)
	{
		//retry
		TxOnSpecifiedPort(&retryDataBuffer[retryIndex], FALSE);
		retryDataBuffer[retryIndex].retries--;
		RegisterTimer(RetryExpiration, retryIndex, ACK_TIMEOUT, FALSE, FALSE);
	}
	else
	{
		//report the failure
		/*if (NoAckCallback != 0 && IS_ACK_REQUIRED_TX(&retryDataBuffer[retryIndex]))
			NoAckCallback(retryDataBuffer[retryIndex].mac);*/
		if (IS_ACK_REQUIRED_TX(&retryDataBuffer[retryIndex]))
		{
			FWUpdateAckResult(retryDataBuffer[retryIndex].mac, FALSE, FALSE,NACK_UNDEFINED);
			RangeTestAckHandler(retryDataBuffer[retryIndex].mac, FALSE);
			
			if (AckResultCallback != 0)
				AckResultCallback(retryDataBuffer[retryIndex].mac, FALSE);
		}
		
		//clear ack data
		retryDataBuffer[retryIndex].mac = 0;
	}
}

void TxFailed(uint16_t mac)
{
	/*for(uint8_t i=0; i<RETRY_BUFFER_SIZE; i++)
	{
		if (retryDataBuffer[i].mac == mac)
		{
			AckExpiration(i);
			return;
		}
	}*/
}

static uint32_t acksReceived = 0;
static void AckReceived(uint16_t mac, bool highLevel, bool success,uint8_t reason)
{
	AbortRetries(mac);
	
	FWUpdateAckResult(mac, highLevel, success,reason);
	
	if (success)
	{
		acksReceived++;
		
		//report the success
		RangeTestAckHandler(mac, TRUE);
		if (AckResultCallback != 0)
			AckResultCallback(mac, TRUE);
	}
}

static Port* tempPort;
bool RoutePacket(CM2Packet* packet)
{
	bool sendAck = FALSE;
	
	// Check basic validity
	if (!IS_PACKET_VALID(packet))
		return TRUE;
	
	if (packet->deviceSource != PORT_INTERNAL)
	{
		// Handle low-level acking ASAP
		// v00_0B added processing of high level ACK as if LLACK 
		if (packet->command[0] == OP_LOW_LEVEL_ACK)
		{
			// Check if we were waiting on this ack
			AckReceived((packet->command[1]<<8)|packet->command[2], FALSE, TRUE,0);
			return TRUE;
		}
		else if (packet->command[0] == OP_ACK || packet->command[0] == OP_NACK)
		{
			// Check if we were waiting on this ack
			AckReceived((packet->command[1]<<8)|packet->command[2], TRUE, (packet->command[0] == OP_ACK),packet->command[3]);
		}
		
		tempPort = GetPort(packet->deviceSource);
		
		if (tempPort != 0)
		{
			//do we need to SEND an ack?
			if (tempPort->handleAckingCallback != 0)
			{
				tempPort->handleAckingCallback(packet, OP_LOW_LEVEL_ACK, 0, FALSE);
			}
			else if (tempPort->automaticLLAcks && IsPacketForMe(packet) && IS_ACK_REQUIRED_RX(packet))
			{
				// If this packet requires acking then send it
				sendAck = TRUE;
			}
			
			if (tempPort->filtering.flags.duplicateFiltering)
			{
				//is this a duplicate?
				PacketLog logTemp;
				for (uint8_t i=0; i<tempPort->packetHistory.count; i++)
				{
					RawCircularBufferPeek(&tempPort->packetHistory, i, (void*)&logTemp);
					
					if (logTemp.MAC == packet->mac && GetMsElapsed(logTemp.arrivalTime) < DUPLICATE_TIMEOUT)
						return TRUE;
				}
				logTemp.arrivalTime = GetMsSinceStart();
				logTemp.MAC = packet->mac;
				if (IsRawBufferFull(&tempPort->packetHistory))
					RawCircularBufferDelete(&tempPort->packetHistory, 1);
				RawCircularBufferWrite(&tempPort->packetHistory, 1, &logTemp);
			}
		}
	}
	
	////////////////////ResetIdleTimeout(); ///////////////TODO
	
	// Add packet to buffer
	if (RawCircularBufferWrite(&inputBuffer, 1, (void*)packet))
	{
		if (sendAck)
			Ack(packet, FALSE, FALSE);
		return TRUE;
	}
	else
		return FALSE;
}

static bool lastPacketSent = FALSE;
static void PrepareNextPacket(void)
{
	// Get the next packet from the buffer and route it
	if (IsRawBufferEmpty(&inputBuffer) || !RawCircularBufferRead(&inputBuffer, 1, (void*)&packetTemp, TRUE))
		return;
	
	// If the destination isn't filled out try to figure out the best way to route it.
	if (PORT_UNKNOWN == packetTemp.deviceDest)
	{
		//Is it addressed directly to us?

		packetTemp.deviceDest = PORT_INTERNAL; ///////////TODO - temporary solution to retransmission on spirit alt
	}
	//else use the existing port selections
	
	// Set up retries and success/failure reporting
	if (PORT_INTERNAL == packetTemp.deviceSource)
	{
		if (packetTemp.command[0] != OP_LOW_LEVEL_ACK)
		{
			//find an empty slot to store it in
			for(uint8_t i=0; i<RETRY_BUFFER_SIZE; i++)
			{
				if (retryDataBuffer[i].mac == 0)
				{
					CopyPacketStatic(&packetTemp, &(retryDataBuffer[i]));
					if (retryDataBuffer[i].retries == 0xFF)
						retryDataBuffer[i].retries = DEFAULT_NUM_RETRIES;
					RegisterTimer(RetryExpiration, i, ACK_TIMEOUT, FALSE, FALSE);
					//ackCopySaved = TRUE;
					break;
				}
			}
			
			///////////////TODO-handle buffer overflow
			//if (!ackCopySaved)
			//	while(1);
		}
	}
	
	lastPacketSent = FALSE;
}

static uint16_t txFailedToStart=0;
void RouteNextPacket(void)
{
    if (lastPacketSent)
        PrepareNextPacket();

    if (FALSE == lastPacketSent)
    {
	    if (MIN_TIME_BETWEEN_TX > GetMsElapsed(lastTxTick))
		   return;
	    lastTxTick = GetMsSinceStart();

	    // send packet to the specified port, as long as it provides a transmit function
	    TxStatus status = TxOnSpecifiedPort(&packetTemp, FALSE);

	    if (TX_FAIL != status && TX_IDLE != status)
	    {
		   //TX is proceeding or permanently failed, we can remove it from the buffer
		   //RawCircularBufferDelete(&inputBuffer, 1);
		   lastPacketSent = TRUE;
	    }
	    else
	    {
		   txFailedToStart++;
	    }
    }
}
