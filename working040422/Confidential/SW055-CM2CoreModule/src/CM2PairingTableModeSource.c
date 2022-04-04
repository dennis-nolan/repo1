/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
	02/06/2019: Cut out automatic challenge/response and let it be handled within unpaired packet callback.
******************************* END VERSION INFO ******************************/

//#include <stdlib.h>
//#include "MemoryMgmtHeader.h"
#include "CM2PairingTableModeHeader.h"
#include "CM2SecurityHeader.h"
#include "CM2RoutingHeader.h"


static uint32_t* pairingTable = 0;
static uint8_t pairingTableLen = 0;
static uint8_t numPaired = 0;
static bool pairingMode = FALSE;
static uint16_t latestChallenge = 0xFFFF;
static uint8_t myDeviceType = 0;
static uint32_t myID = 0;
static bool promiscuousMode = FALSE;

static void (*UnpairedRxCallback) (CM2Packet*) = 0;

void UnSetUnpairedPacketCallback(void)
{
	UnpairedRxCallback = 0;
}
void SetUnpairedPacketCallback(void (*UnpairedPacketRX)(CM2Packet*))
{
	UnpairedRxCallback = UnpairedPacketRX;
}

void CM2PairingTableModeInit(uint8_t deviceType, uint32_t ID)
{
	myDeviceType = deviceType;
	myID = ID;
	pairingTableLen = 0;
	numPaired = 0;
	pairingMode = FALSE;
}

void CM2PairingTableModeDeinit(void)
{
	if (pairingTable != 0)
	{
		pairingTable = 0;
	}
}

static uint32_t notPaired=0,spiritNotForMe=0,invalidPacket=0;
void ProcessNextRxPacketTableMode(CM2Packet* packet)
{
  RxPacketResult result;
	// get the next packet
	if (IS_PACKET_VALID(packet) && packet->mac == CalcMAC(packet, packet->srcID))
	{
		if (IsPacketForMe(packet))
		{
			if (promiscuousMode || IsPaired(packet->srcID) || IS_ADMIN_COMMAND(packet) ||
				(PORT_SPIRIT != packet->deviceSource && PORT_SPIRIT_ALT != packet->deviceSource))
			{
				//in pairing table, handle the packet
				
				// run the packet through the app's handler to see if they want to handle it. If not try to handle it here.
				result = ExecutePacketRxCallback(packet);
				if (RX_PACKET_UNHANDLED == result)
				{
					if (IS_ADMIN_COMMAND(packet))
					{
						// Attempt to handle within this driver
						result = ProcessAdminPacket(packet);
					}
					else if (IS_SCRIPTING_COMMAND(packet))
					{
						// Handle in scripting module
						//result = ProcessScriptingPacket(packetTemp); /////////////////////////////TODO-make this fit with extended CM2
					}
				}
				
				if (RX_PACKET_HANDLED != result)
				{
					//error with packet. Nack if necessary.
					if (IS_ACK_REQUIRED_RX(packet))
						Nack(packet, 0xFF); //////TODO - fill out reason code
				}
			}
			else if (UnpairedRxCallback != 0)
			{
				//application is interested in unpaired packets, so forward it
				UnpairedRxCallback(packet);
			}
			else //else not in table while not in pair mode - ignore
				notPaired++;
		}
		else //else packet is not for me and came across Spirit - ignore
			spiritNotForMe++;
	}
	else //else invalid packet - ignore
		invalidPacket++;
}

void SetPairingTable(uint32_t* storedPairingTable, uint8_t numStored, uint8_t tableLength)
{
	if (storedPairingTable != 0 && tableLength != 0 && numStored <= tableLength)
	{
		//if (pairingTable != 0)
		//FreeMem((void**)&pairingTable);
		
		pairingTable = storedPairingTable;
		numPaired = numStored;
		pairingTableLen = tableLength;
	}
}

void SetPromiscuousMode(bool enable)
{
	promiscuousMode = enable;
}

bool IsPaired(uint32_t ID)
{
	//if no table then fail immediately
	if (pairingTable == 0 || pairingTableLen == 0 || ID == 0xFFFFFFFF)
		return FALSE;
	
	//search
	for(uint8_t i=0; i<pairingTableLen; i++)
	{
		if (pairingTable[i] == ID)
			return TRUE;
	}
	return FALSE;
}

bool Pair(uint32_t ID)
{
	//if no table then fail immediately
	if (pairingTable == 0 || pairingTableLen == 0)
		return FALSE;
	
	uint16_t firstZeroIndex = 0xFFFF;
	
	//see if the ID is already in the table and look for the first empty spot
	for(uint8_t i=0; i<pairingTableLen; i++)
	{
		if (pairingTable[i] == ID)
			return TRUE;
		else if (pairingTable[i] == 0xFFFFFFFF && i < firstZeroIndex)
			firstZeroIndex = i;
	}
	//if we got here it isn't in the table yet.
	
	if (firstZeroIndex == 0xFFFF)
		return FALSE; //no room
	//else we found a place for it
	
	pairingTable[firstZeroIndex] = ID;
	numPaired++;
	return TRUE;
}

bool Unpair(uint32_t ID)
{
	//if no table then fail immediately
	if (pairingTable == 0 || pairingTableLen == 0)
		return FALSE;
	
	//search within table
	for(uint8_t i=0; i<pairingTableLen; i++)
	{
		if (pairingTable[i] == ID)
		{
			//found the ID
			pairingTable[i] = 0;
			numPaired--;
			return TRUE;
		}
	}
	return FALSE;
}

void SetPairingMode(bool enable)
{
	pairingMode = enable;
	
	if (pairingMode)
		latestChallenge = GenerateChallenge();
	
	////////////////////////////TODO-timeout
}