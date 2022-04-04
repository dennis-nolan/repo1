/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
******************************* END VERSION INFO ******************************/

//#include <stdlib.h>
//#include "MemoryMgmtHeader.h"
#include "CM2NetworkModeHeader.h"
#include "CM2SecurityHeader.h"

static CM2Packet* packetTemp = 0;
static uint8_t myDeviceType = 0;
static uint32_t myID = 0;
static uint32_t myNID = 0;

void CM2NetworkModeInit(uint8_t deviceType, uint32_t ID, uint32_t NID)
{
	myDeviceType = deviceType;
	myID = ID;
	myNID = NID;
}

void CM2NetworkModeDeinit(void)
{
	
}

void ProcessNextRxPacketNetworkMode(CircularBuffer* rxBuffer)
{
	// get the next packet
  	while(!IS_BUFFER_EMPTY(rxBuffer) && !BufferRead(rxBuffer, (void**)&packetTemp));
	
	if (!IS_PACKET_VALID(packetTemp))
		return; // Nothing valid in buffer
	
	
	if (packetTemp->deviceSource != PORT_SPIRIT && !IsPacketForMe(packetTemp, myDeviceType, myID))
	{ // forward internal packets not addressed to us over SPIRIT
	  
		// fill in src if necessary
		if (packetTemp->srcID == 0xFFFFFFFF)
			packetTemp->srcID = myID;
		
		// fill in mac if necessary
		if (packetTemp->mac == 0xFFFF)
			packetTemp->mac = CalcMAC(packetTemp, IS_ADMIN_COMMAND(packetTemp)?packetTemp->srcID:myNID);
		
		// send it
		TxBlocking(packetTemp);
		//CM2TxBlocking(packetTemp); //leads to a call loop!
		///////////////TODO: handle result
	}
	else
	{
		// Verify packet. Handle network packets here, send all others to application
		if (IS_ADMIN_COMMAND(packetTemp))
		{
			// Note that all network packets use the src addr as the network ID
			if (CheckPacket(packetTemp, myDeviceType, myID, packetTemp->srcID))
			{
				// ack if required
				if (IS_ACK_REQUIRED_RX(packetTemp))
					Ack(packetTemp);
			
				// Handle here
				ProcessAdminPacket(packetTemp);
			}
		}
		else if (CheckPacket(packetTemp, myDeviceType, myID, myNID))
		{
			// ack if required
			if (IS_ACK_REQUIRED_RX(packetTemp))
				Ack(packetTemp);
			
			if (IS_SCRIPTING_COMMAND(packetTemp))
			{
				// Handle in scripting module
				//ProcessScriptingPacket(packetTemp); ///////////////////////////////////////TODO-make this fit with extended CM2
			}
			else //if (cmdRxCallback != 0)
			{
				// Handle in application
				//cmdRxCallback(packetTemp);
				ExecutePacketRxCallback(packetTemp);
			}
		}
	}

	// Clean up
	if (packetTemp != 0)
	{
		//if (packetTemp->command != 0)
		FreeMem((void**)&packetTemp->command);
		FreeMem((void**)&packetTemp);
		packetTemp = 0;
	}
}

