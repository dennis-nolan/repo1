
//#include <stdlib.h>
#include "MemoryMgmtHeader.h"
#include "CM2UTMocks.h"

CM2Packet* lastSpiritTxPacket = 0;
CM2Packet* lastAppRxPacket = 0;

void ResetMocks(void)
{
	if (lastSpiritTxPacket != 0)
	{
		//if (lastSpiritTxPacket->command != 0)
			FreeMem((void**)&lastSpiritTxPacket->command);
		//lastSpiritTxPacket->command = 0;
		
		FreeMem((void**)&lastSpiritTxPacket);
	}
	lastSpiritTxPacket = 0;
	
	if (lastAppRxPacket != 0)
	{
		//if (lastAppRxPacket->command != 0)
			FreeMem((void**)&lastAppRxPacket->command);
		//lastAppRxPacket->command = 0;
		
		FreeMem((void**)&lastAppRxPacket);
	}
	lastAppRxPacket = 0;
}

bool TxBlocking(CM2Packet* packet)
{
	if (lastSpiritTxPacket != 0)
	{
		//if (lastSpiritTxPacket->command != 0)
			FreeMem((void**)&lastSpiritTxPacket->command);
		//lastSpiritTxPacket->command = 0;
		
		FreeMem((void**)&lastSpiritTxPacket);
	}
	
	lastSpiritTxPacket = CopyPacket(packet);
	return TRUE;
}

bool PacketRX(CM2Packet* packet)
{
	if (IS_ADMIN_COMMAND(packet) || IS_SCRIPTING_COMMAND(packet))
		return FALSE;
	
	if (lastAppRxPacket != 0)
	{
		//if (lastAppRxPacket->command != 0)
			FreeMem((void**)&lastAppRxPacket->command);
		//lastAppRxPacket->command = 0;
		
		FreeMem((void**)&lastAppRxPacket);
	}
	
	lastAppRxPacket = CopyPacket(packet);
	
	return TRUE;
}
