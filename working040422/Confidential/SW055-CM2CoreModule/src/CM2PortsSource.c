/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/11/2015: Initial creation.
******************************* END VERSION INFO ******************************/

#include "CM2PortsHeader.h"
#include "TimersHeader.h"
#include "CM2RoutingHeader.h"

#ifndef MAX_NUM_PORTS
#pragma message("MAX_NUM_PORTS is not defined. Defaulting to 3.")
#define MAX_NUM_PORTS	3
#endif

#define INTERBYTE_TIMEOUT_MS		100
#define MAX_CONSECUTIVE_MAC_ERRORS	20

enum {PARSE_STATE_NO_PACKET=0, PARSE_STATE_23_FOUND, PARSE_STATE_81_FOUND, PARSE_STATE_LENGTH_FOUND, PARSE_STATE_COMPLETE};

static Port* ports[MAX_NUM_PORTS];
static CM2Packet unflattened;


void PortHandlingInit(void)
{
	RegisterThread(RunCM2PortsTasks, 0x05);
}

static int GetPortIndex(CM2PortType portType)
{
	for (uint8_t i=0; i<MAX_NUM_PORTS; i++)
	{
		if (ports[i]->portType == portType || PORT_ALL == portType)
			return i;
	}
	return -1;
}

Port* GetPort(CM2PortType portType)
{
	int i = GetPortIndex(portType);
	
	if (i < 0)
		return (Port*)0;
	else
		return ports[i];
}

static uint32_t fail81=0, lenFail=0, failUnf=0, totalStarted=0, /*bytesTossed1=0,*/ bytesTossed2=0, packetQueued=0, packetQueueFail=0;
static uint8_t temp, length;
static void Parse(Port* port)
{
	if (port->rxByteBuffer.count == 0 || (port->packetParseState != PARSE_STATE_LENGTH_FOUND && port->rxByteBuffer.count < 3))
		return;
	
	if (port->packetParseState == PARSE_STATE_NO_PACKET)
	{
		//remove any leading garbage
		
		while (port->rxByteBuffer.count > 0 && RawCircularBufferPeek(&port->rxByteBuffer,0,&temp) && temp != 0x23)
		{ RawCircularBufferDelete(&port->rxByteBuffer, 1); bytesTossed2++; }
		
		if (temp == 0x23)
		{
			port->packetParseState = PARSE_STATE_23_FOUND;
			totalStarted++;
		}
		else
			return;
	}
	
	if (port->packetParseState == PARSE_STATE_23_FOUND)
	{
		//look for 81
		if (port->rxByteBuffer.count > 1 && RawCircularBufferPeek(&port->rxByteBuffer,1,&temp))
		{
			if (temp == 0x81)
			{
				port->packetParseState = PARSE_STATE_81_FOUND;
			}
			else
			{
				//not a valid start. delete the 0x23 and reset.
				RawCircularBufferDelete(&port->rxByteBuffer, 1);
				port->packetParseState = PARSE_STATE_NO_PACKET;
				fail81++;
				return;
			}
		}
	}
	
	if (port->packetParseState == PARSE_STATE_81_FOUND)
	{
		//look for length
		if (port->rxByteBuffer.count > 2 && RawCircularBufferPeek(&port->rxByteBuffer,2,&temp))
		{
			if (temp > 15 && temp <= port->rxByteBuffer.maxLength)
			{
				port->packetParseState = PARSE_STATE_LENGTH_FOUND;
			}
			else
			{
				//not a valid start. delete the 0x2381 and reset.
				RawCircularBufferDelete(&port->rxByteBuffer, 2);
				port->packetParseState = PARSE_STATE_NO_PACKET;
				lenFail++;
				return;
			}
		}
	}
	
	if (port->packetParseState == PARSE_STATE_LENGTH_FOUND)
	{
		if (port->rxByteBuffer.count >= 3 && RawCircularBufferPeek(&port->rxByteBuffer,2,&length))
		{
			if (port->rxByteBuffer.count >= length)
			{
				//packet complete!
				if (UnflattenCM2PacketStatic3(&unflattened, &port->rxByteBuffer, port->portType))
				{
					if (RoutePacket(&unflattened))
						packetQueued++;
					else
						packetQueueFail++;
					
					//remove bytes from buffer even if it failed to route
					RawCircularBufferDelete(&port->rxByteBuffer, length);
					
					//reset state
					port->packetParseState = PARSE_STATE_NO_PACKET;
				}
				else
				{
					//failed to unflatten. reset state and remove 0x2381
					RawCircularBufferDelete(&port->rxByteBuffer, 2);
					port->packetParseState = PARSE_STATE_NO_PACKET;
					failUnf++;
				}
			}
		}
	}
}

void RunCM2PortsTasks(void)
{
	for (uint8_t i=0; i<MAX_NUM_PORTS; i++)
	{
		if (ports[i]->portType != PORT_UNKNOWN)
			Parse(ports[i]);
	}
}

int RegisterPort(Port* portInfo)
{
	// verify validity
	//////////////////////TODO
	
	// find an empty slot to put it in
	for (uint8_t i=0; i<MAX_NUM_PORTS; i++)
	{
		if (ports[i] == 0)
		{
			ports[i] = portInfo;
			return i;
		}
	}
	
	//if we got here we ran out of ports
	//////////////////////TODO - log error
	
	return -1;
}

TxStatus TxOnSpecifiedPort(CM2Packet* packet, bool blocking)
{
	int index = GetPortIndex(packet->deviceDest);
	
	if (index < 0)
		return TX_FAIL_FATAL;
	else
		return ports[index]->Transmit(ports[index]->portType, packet, blocking);
}

/*static uint32_t interbyteTimeouts=0;
static void InterbyteTimeout(uint8_t portIndex)
{
	//we haven't gotten a new byte in time, so clear the buffer
	RawCircularBufferClear(&(ports[portIndex]->rxByteBuffer));
	interbyteTimeouts++;
}*/

static uint32_t bufferFull = 0;
bool RxBytes(CM2PortType portType, uint8_t num, uint8_t* data)
{
	int index = GetPortIndex(portType);
	if (index < 0)
		return FALSE;
	
	//check if the buffer is full
	if (IsRawBufferFull(&(ports[index]->rxByteBuffer)))
	{
		bufferFull++;
		
		//clear it out to make room
		RawCircularBufferClear(&(ports[index]->rxByteBuffer));
		ports[index]->packetParseState = PARSE_STATE_NO_PACKET;
	}
	
	//copy all of it
	return RawCircularBufferWrite(&(ports[index]->rxByteBuffer),num,data);
}

static uint16_t txFailedToFinish = 0;
void TxCallback(CM2PortType portFlags, uint16_t mac, TxStatus status)
{
	//uint8_t dummy = 5;
	//dummy++;
	
	//success? good! not much to do.
	//failure? what kind?
		//driver/HW issue?
		//holdoff limit?
		//unknown?

	if (TX_FAIL == status)
	{
		txFailedToFinish++;
		//TxFailed(mac);
	}
}

void ReportMacResult(CM2PortType type, bool success)
{
	int index = GetPortIndex(type);
	
	if (index < 0)
		return;
	
	if (success)
		ports[index]->macFailures = 0;
	else
	{
		ports[index]->macFailures++;
		
		/*if (ports[index]->macFailures >= MAX_CONSECUTIVE_MAC_ERRORS && ports[index]->Reinit != 0)
		{
			ports[index]->Reinit(ports[index]->portType);
			ports[index]->macFailures = 0;
		}*/
	}
	
	return;
}
