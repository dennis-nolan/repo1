
#include "MemoryMgmtHeader.h"
#include "CM2RoutingMocks.h"

static CM2Packet* lastTX1Packet = 0;
static CM2Packet* lastTX2Packet = 0;
static CM2Packet* lastTX3Packet = 0;
static CM2Packet* lastTX4Packet = 0;

CM2Packet* GetLastTx(uint8_t num)
{
	switch(num)
	{
		case 1:
			return lastTX1Packet;
		case 2:
			return lastTX2Packet;
		case 3:
			return lastTX3Packet;
		case 4:
			return lastTX4Packet;
		default:
			return 0;
	}
}

bool Transmit1(CM2Packet* packet)
{
	FreeMem((void**)&lastTX1Packet);
	lastTX1Packet = CopyPacket(packet);
	return TRUE;
}

bool Transmit2(CM2Packet* packet)
{
	FreeMem((void**)&lastTX2Packet);
	lastTX2Packet = CopyPacket(packet);
	return TRUE;
}

bool Transmit3(CM2Packet* packet)
{
	FreeMem((void**)&lastTX3Packet);
	lastTX3Packet = CopyPacket(packet);
	return TRUE;
}

bool Transmit4(CM2Packet* packet)
{
	FreeMem((void**)&lastTX4Packet);
	lastTX4Packet = CopyPacket(packet);
	return TRUE;
}

bool Ready(void)
{
	return TRUE;
}

void ResetRoutingMocks(void)
{
	if (lastTX1Packet != 0)
	{
		//if (lastTX1Packet->command != 0)
		FreeMem((void**)&lastTX1Packet->command);
		FreeMem((void**)&lastTX1Packet);
		//lastTX1Packet = 0;
	}
	
	if (lastTX2Packet != 0)
	{
		//if (lastTX2Packet->command != 0)
		FreeMem((void**)&lastTX2Packet->command);
		FreeMem((void**)&lastTX2Packet);
		//lastTX2Packet = 0;
	}
	
	if (lastTX3Packet != 0)
	{
		//if (lastTX3Packet->command != 0)
		FreeMem((void**)&lastTX3Packet->command);
		FreeMem((void**)&lastTX3Packet);
		//lastTX3Packet = 0;
	}
		
	if (lastTX4Packet != 0)
	{
		//if (lastTX4Packet->command != 0)
		FreeMem((void**)&lastTX4Packet->command);
		FreeMem((void**)&lastTX4Packet);
		//lastTX4Packet = 0;
	}
}
