
#include "CM2UTUtils.h"
#include "CM2CoreHeader.h"
#include "CM2SecurityHeader.h"

bool CheckAck(CM2Packet* ackToCheck, uint32_t dest, uint32_t src, uint16_t originalMac, uint32_t NID)
{
	if (ackToCheck == 0)
		return FALSE;
	
	uint8_t ackPayload[3] = {OP_LOW_LEVEL_ACK, originalMac>>8, originalMac};
	CM2Packet ack = 
	{
		PORT_INTERNAL,
		PORT_SPIRIT,//destination SPIRIT
		PROTOCOL_VERSION,
		0,
		src,
		DIRECT_ADDRESSING,
		dest,
		0x00,		//no subaddress
		0x03,		//3 byte payload
		ackPayload, //will contain opcode + msg MAC
		0x0000		//MAC to be filled out before acking
	};
	ack.mac = CalcMAC(&ack, NID);
	return PacketsIdentical(ackToCheck, &ack);
}
