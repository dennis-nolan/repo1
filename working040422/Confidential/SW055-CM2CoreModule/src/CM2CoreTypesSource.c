/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
******************************* END VERSION INFO ******************************/

//#include "MemoryMgmtHeader.h"
#include "CM2CoreTypesHeader.h"
#include "CM2SecurityHeader.h"
#include "CM2PortsHeader.h"
#include <string.h>

/*void ClearPortFlags(CM2PortFlags* portflags)
{
	portflags->PORT_INTERNAL = FALSE;
	portflags->PORT_SPIRIT = FALSE;
	portflags->PORT_SPIRIT_ALT = FALSE;
	portflags->PORT_SPIRIT_PLC = FALSE;
	portflags->PORT_BT = FALSE;
	portflags->PORT_BLE = FALSE;
	portflags->PORT_USB = FALSE;
	portflags->PORT_UART = FALSE;
	portflags->PORT_CAN = FALSE;
	portflags->RESERVED = 0;
}

bool AnyFlagsSet(CM2PortFlags flags)
{
	if (flags.PORT_INTERNAL)
		return TRUE;
	if (flags.PORT_SPIRIT)
		return TRUE;
	if (flags.PORT_SPIRIT_ALT)
		return TRUE;
	if (flags.PORT_SPIRIT_PLC)
		return TRUE;
	if (flags.PORT_BT)
		return TRUE;
	if (flags.PORT_BLE)
		return TRUE;
	if (flags.PORT_USB)
		return TRUE;
	if (flags.PORT_UART)
		return TRUE;
	if (flags.PORT_CAN)
		return TRUE;
	return FALSE;
}

bool AnyFlagsCommon(CM2PortFlags flags1, CM2PortFlags flags2)
{
	if (flags1.PORT_INTERNAL && flags2.PORT_INTERNAL)
		return TRUE;
	if (flags1.PORT_SPIRIT && flags2.PORT_SPIRIT)
		return TRUE;
	if (flags1.PORT_SPIRIT_ALT && flags2.PORT_SPIRIT_ALT)
		return TRUE;
	if (flags1.PORT_SPIRIT_PLC && flags2.PORT_SPIRIT_PLC)
		return TRUE;
	if (flags1.PORT_BT && flags2.PORT_BT)
		return TRUE;
	if (flags1.PORT_BLE && flags2.PORT_BLE)
		return TRUE;
	if (flags1.PORT_USB && flags2.PORT_USB)
		return TRUE;
	if (flags1.PORT_UART && flags2.PORT_UART)
		return TRUE;
	if (flags1.PORT_CAN && flags2.PORT_CAN)
		return TRUE;
	return FALSE;
}*/

/*enum {
	SOURCE_INTERNAL = 0x00,
	SOURCE_SPIRIT	= 0x01,
	SOURCE_BT		= 0x02,
	SOURCE_BLE		= 0x03,
	SOURCE_USB		= 0x04,
	SOURCE_UART		= 0x05,
	//Not assigned	= 0x06 to 0xFE
	SOURCE_UNKNOWN	= 0xFF
} PacketSource;*/

/*CM2Packet* CopyPacket(CM2Packet* packet)
{
	CM2Packet* toReturn = (CM2Packet*)GetMem(sizeof(CM2Packet));
	toReturn->command = (uint8_t*)GetMem(packet->commandLen);
	
	CopyPacketStatic(packet, toReturn);
	
	return toReturn;
}*/
	
bool CopyPacketStatic(CM2Packet* inputPacket, CM2Packet* outputPacket)
{
	if (outputPacket == 0 || inputPacket == 0 || outputPacket->command == 0 || inputPacket->command == 0)
		return FALSE;
	
	outputPacket->deviceSource = inputPacket->deviceSource;
	outputPacket->deviceDest = inputPacket->deviceDest;
	outputPacket->retries = inputPacket->retries;
	outputPacket->protocolVer = inputPacket->protocolVer;
	outputPacket->srcType = inputPacket->srcType;
	outputPacket->srcID = inputPacket->srcID;
	outputPacket->destType = inputPacket->destType;
	outputPacket->destID = inputPacket->destID;
	outputPacket->subaddress = inputPacket->subaddress;
	outputPacket->commandLen = inputPacket->commandLen;
	
	memcpy(outputPacket->command, inputPacket->command, inputPacket->commandLen);
	
	outputPacket->mac = inputPacket->mac;
	
	return TRUE;
}

bool PacketsIdentical(CM2Packet* packet1, CM2Packet* packet2)
{
	if (packet1 == 0)
		return (bool)(packet2 == 0);
	else if (packet2 == 0)
		return FALSE;
	
	if (packet1->command == 0)
		return (bool)(packet2->command == 0);
	else if (packet2->command == 0)
		return FALSE;
	
	/*if (packet1->deviceSource != packet2->deviceSource)
		return FALSE;
	
	if (packet1->deviceDest != packet2->deviceDest)
		return FALSE;*/
	
	if (packet1->protocolVer != packet2->protocolVer)
		return FALSE;
	
	if (packet1->srcType != packet2->srcType)
		return FALSE;
	
	if (packet1->srcID != packet2->srcID)
		return FALSE;
	
	if (packet1->destType != packet2->destType)
		return FALSE;
	
	if (packet1->destID != packet2->destID)
		return FALSE;
	
	if (packet1->subaddress != packet2->subaddress)
		return FALSE;
	
	if (packet1->commandLen != packet2->commandLen)
		return FALSE;
	
	if (packet1->mac != packet2->mac)
		return FALSE;
	
	for (uint8_t i=0; i<packet1->commandLen; i++)
	{
		if (packet1->command[i] != packet2->command[i])
			return FALSE;
	}
	
	return TRUE;
}

/*void FlattenCM2Packet(CM2Packet* inputPacket, uint8_t* length, uint8_t** data, bool flattenForSerial)
{
	if (inputPacket == 0)
	{
		*length = 0;
		*data = 0;
		return;
	}
	
	*length = 15 + inputPacket->commandLen;
	if (flattenForSerial)
		*length += 3; //2 start bytes + length
	
	*data = (uint8_t*)GetMem(*length);
	
	FlattenCM2PacketStatic(inputPacket,length,*data,flattenForSerial);
}*/

//bool FlattenCM2PacketStatic(CM2Packet* inputPacket, uint8_t* length, uint8_t* data, bool flattenForSerial)
void FlattenCM2PacketStatic(CM2Packet* inputPacket, uint8_t* length, uint8_t* data, bool flattenForSerial)
{
	if (inputPacket == 0)
	{
		*length = 0;
		return;// FALSE;
	}
	
	*length = 15 + inputPacket->commandLen;
	if (flattenForSerial)
		*length += 3; //2 start bytes + length
	
	uint8_t offset=0;
	if (flattenForSerial)
	{
		data[0]  = 0x23;
		data[1]  = 0x81;
		data[2]  = *length;
		offset = 3;
	}
	
	data[0+offset]  = inputPacket->protocolVer;
	data[1+offset]  = inputPacket->srcType;
	data[2+offset]  = (inputPacket->srcID)>>24;
	data[3+offset]  = (inputPacket->srcID)>>16;
	data[4+offset]  = (inputPacket->srcID)>>8;
	data[5+offset]  = inputPacket->srcID;
	data[6+offset]  = inputPacket->destType;
	data[7+offset]  = (inputPacket->destID)>>24;
	data[8+offset]  = (inputPacket->destID)>>16;
	data[9+offset]  = (inputPacket->destID)>>8;
	data[10+offset]  = inputPacket->destID;
	data[11+offset] = inputPacket->subaddress;
	data[12+offset] = inputPacket->commandLen;
	
	
	// Copy payload
	for (uint8_t i=0; i<inputPacket->commandLen; i++)
	{ data[i+13+offset] = inputPacket->command[i]; }
	
	// add MAC
	data[inputPacket->commandLen + 13 + offset] = (inputPacket->mac)>>8;
	data[inputPacket->commandLen + 14 + offset] = inputPacket->mac;
	
	return;// TRUE;
}

/*CM2Packet* UnflattenCM2Packet(uint8_t length, uint8_t* data, uint16_t source, bool unflattenFromSerial)
{
	// Check validity of input
	if (length < 15 || data == 0)
		return (CM2Packet*)0;
	
	if (unflattenFromSerial && (data[0] != 0x23 || data[1] != 0x81))
		return (CM2Packet*)0;
	
	CM2Packet* toReturn = (CM2Packet*)GetMem(sizeof(CM2Packet));
	
	uint8_t offset=0;
	if (unflattenFromSerial)
		offset = 3;
	
	toReturn->deviceSource = source;
	toReturn->deviceDest = PORT_UNKNOWN;
	toReturn->protocolVer = data[0+offset];
	toReturn->srcType = data[1+offset];
	toReturn->srcID = (data[2+offset]<<24) | (data[3+offset]<<16) | (data[4+offset]<<8) | (data[5+offset]);
	toReturn->destType = data[6+offset];
	toReturn->destID = (data[7+offset]<<24) | (data[8+offset]<<16) | (data[9+offset]<<8) | (data[10+offset]);
	toReturn->subaddress = data[11+offset];
	toReturn->commandLen = data[12+offset];
	
	// Copy payload
	toReturn->command = (uint8_t*)GetMem(toReturn->commandLen);
	for (uint8_t i=0; i<toReturn->commandLen; i++)
		toReturn->command[i] = data[i+13+offset];
	
	toReturn->mac = (data[length-2]<<8) | data[length-1];
	
	return toReturn;
}*/

bool UnflattenCM2PacketStatic(CM2Packet* packetToPopulate, uint8_t length, uint8_t* data, CM2PortType source, bool unflattenFromSerial)
{
	//bool unflattenFromSerial = TRUE;//////////////////////////////////////////////////////
	
	// Check validity of input
	if (length < 15 || data == 0)
		return FALSE;//(CM2Packet*)0;
	
	if (unflattenFromSerial && (data[0] != 0x23 || data[1] != 0x81))
		return FALSE;//(CM2Packet*)0;
	
	//CM2Packet* toReturn = (CM2Packet*)GetMem(sizeof(CM2Packet));
	
	uint8_t offset=0;
	if (unflattenFromSerial)
		offset = 3;
	
	packetToPopulate->deviceSource = source;
	packetToPopulate->deviceDest = PORT_UNKNOWN;
	packetToPopulate->protocolVer = data[0+offset];
	packetToPopulate->srcType = data[1+offset];
	packetToPopulate->srcID = (data[2+offset]<<24) | (data[3+offset]<<16) | (data[4+offset]<<8) | (data[5+offset]);
	packetToPopulate->destType = data[6+offset];
	packetToPopulate->destID = (data[7+offset]<<24) | (data[8+offset]<<16) | (data[9+offset]<<8) | (data[10+offset]);
	packetToPopulate->subaddress = data[11+offset];
	packetToPopulate->commandLen = data[12+offset];
	
	// Copy payload
	//packetToPopulate->command = (uint8_t*)GetMem(toReturn->commandLen);
	for (uint8_t i=0; i<packetToPopulate->commandLen; i++)
		packetToPopulate->command[i] = data[i+13+offset];
	
	packetToPopulate->mac = (data[length-2]<<8) | data[length-1];
	
	return TRUE;//toReturn;
}

bool UnflattenCM2PacketStatic2(CM2Packet* packetToPopulate, RawCircularBuffer* buf, CM2PortType source)
{
	// Check validity of input
	if (buf == 0 || buf->count < 15)
		return FALSE;
	
	uint8_t temp, length;
	
	RawCircularBufferRead(buf,1,&temp,TRUE);
	if (temp != 0x23)
		return FALSE;
	
	RawCircularBufferRead(buf,1,&temp,TRUE);
	if (temp != 0x81)
		return FALSE;
	
	RawCircularBufferRead(buf,1,&length,TRUE);
	if (length < 15)
		return FALSE;
	
	packetToPopulate->deviceSource = source;
	packetToPopulate->deviceDest = PORT_UNKNOWN;
	
	RawCircularBufferRead(buf,1,&packetToPopulate->protocolVer,TRUE);
	RawCircularBufferRead(buf,1,&packetToPopulate->srcType,TRUE);
	
	uint8_t* tempPtr = (uint8_t*)&packetToPopulate->srcID;
	RawCircularBufferRead(buf,1,&tempPtr[3],TRUE);
	RawCircularBufferRead(buf,1,&tempPtr[2],TRUE);
	RawCircularBufferRead(buf,1,&tempPtr[1],TRUE);
	RawCircularBufferRead(buf,1,&tempPtr[0],TRUE);
	
	RawCircularBufferRead(buf,1,&packetToPopulate->destType,TRUE);
	
	tempPtr = (uint8_t*)&packetToPopulate->destID;
	RawCircularBufferRead(buf,1,&tempPtr[3],TRUE);
	RawCircularBufferRead(buf,1,&tempPtr[2],TRUE);
	RawCircularBufferRead(buf,1,&tempPtr[1],TRUE);
	RawCircularBufferRead(buf,1,&tempPtr[0],TRUE);
	
	RawCircularBufferRead(buf,1,&packetToPopulate->subaddress,TRUE);
	RawCircularBufferRead(buf,1,&packetToPopulate->commandLen,TRUE);
	
	for (int i=packetToPopulate->commandLen-1; i>=0; i--)
	{
		RawCircularBufferRead(buf,1,&packetToPopulate->command[i],TRUE);
	}
	
	tempPtr = (uint8_t*)&packetToPopulate->mac;
	RawCircularBufferRead(buf,1,&tempPtr[1],TRUE);
	RawCircularBufferRead(buf,1,&tempPtr[0],TRUE);
	
	return TRUE;
}

static uint32_t paramFail=0, headerFail=0, macFail=0;
bool UnflattenCM2PacketStatic3(CM2Packet* packetToPopulate, RawCircularBuffer* buf, CM2PortType source)//, bool removeFromBuffer)
{
	// Check validity of input
	if (buf == 0 || buf->count < 15)
	{
		paramFail++;
		return FALSE;
	}
	
	uint8_t temp, length, offset = 0;
	
	RawCircularBufferPeek(buf,offset++,&temp);
	if (temp != 0x23)
	{
		headerFail++;
		return FALSE;
	}
	
	RawCircularBufferPeek(buf,offset++,&temp);
	if (temp != 0x81)
	{
		headerFail++;
		return FALSE;
	}
	
	RawCircularBufferPeek(buf,offset++,&length);
	if (length < 15)
	{
		headerFail++;
		return FALSE;
	}
	
	packetToPopulate->deviceSource = source;
	packetToPopulate->deviceDest = PORT_UNKNOWN;
	
	RawCircularBufferPeek(buf,offset++,&packetToPopulate->protocolVer);
	RawCircularBufferPeek(buf,offset++,&packetToPopulate->srcType);
	
	uint8_t* tempPtr = (uint8_t*)&packetToPopulate->srcID;
	RawCircularBufferPeek(buf,offset++,&tempPtr[3]);
	RawCircularBufferPeek(buf,offset++,&tempPtr[2]);
	RawCircularBufferPeek(buf,offset++,&tempPtr[1]);
	RawCircularBufferPeek(buf,offset++,&tempPtr[0]);
	
	RawCircularBufferPeek(buf,offset++,&packetToPopulate->destType);
	
	tempPtr = (uint8_t*)&packetToPopulate->destID;
	RawCircularBufferPeek(buf,offset++,&tempPtr[3]);
	RawCircularBufferPeek(buf,offset++,&tempPtr[2]);
	RawCircularBufferPeek(buf,offset++,&tempPtr[1]);
	RawCircularBufferPeek(buf,offset++,&tempPtr[0]);
	
	RawCircularBufferPeek(buf,offset++,&packetToPopulate->subaddress);
	RawCircularBufferPeek(buf,offset++,&packetToPopulate->commandLen);
	
	for (int i=0; i<packetToPopulate->commandLen; i++)
	{
		RawCircularBufferPeek(buf,offset++,&packetToPopulate->command[i]);
	}
	
	tempPtr = (uint8_t*)&packetToPopulate->mac;
	RawCircularBufferPeek(buf,offset++,&tempPtr[1]);
	RawCircularBufferPeek(buf,offset++,&tempPtr[0]);
	
	if (IsMacCorrect(packetToPopulate, packetToPopulate->srcID))
	{
		ReportMacResult(source, TRUE);
	}
	else
	{
		ReportMacResult(source, FALSE);
		macFail++;
		return FALSE;
	}
	
	return TRUE;
}