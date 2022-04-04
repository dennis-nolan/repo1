extern "C"
{
//#include <stdlib.h>
#include "MemoryMgmtHeader.h"
#include "TimersHeader.h"
#include "ThreadingHeader.h"
#include "CM2CoreHeader.h"
#include "CM2SecurityHeader.h"
//#include "CM2CoreTypesHeader.h"
//#include "CM2UTMocks.h"
#include "CM2UTUtils.h"
#include "CM2RoutingHeader.h"
#include "CM2RoutingMocks.h"
}

#include "CppUTest/TestHarness.h"

#define MY_ID 0xCDEF5467
#define MY_TYPE 0xCD
#define MY_NID 0xCDCDABAB

#define ID_OF_MOCK_DEVICE 0xABCD9876

extern CM2Packet* lastSpiritTxPacket;
extern CM2Packet* lastAppRxPacket;

static bool PacketRxDummy(CM2Packet* packet)
{ return FALSE; }

TEST_GROUP(NwkPacketTestGroup)
{
	void setup()
	{
		//ResetMocks();
		TimerInit();
		ThreadingInit();
		CM2CoreInit();
		uint32_t* testTable = (uint32_t*)GetMem(4);
		*testTable = ID_OF_MOCK_DEVICE; //only this device paired
		CM2CoreConfigure(MY_ID, MY_TYPE, MY_NID, testTable, 1, 1);
		RegisterPacketRxCallback(PacketRxDummy);
		
		ResetRoutingMocks();
		//register mock spirit port
		Port* testPort2 = (Port*)GetMem(sizeof(Port));
		testPort2->sourceType = PORT_SPIRIT;
		testPort2->Transmit = Transmit2;
		testPort2->Ready = Ready;
		RegisterPort(testPort2);
	}
	
	void teardown()
	{
		//ResetMocks();
		RouterDeinit();
		CM2CoreDeinit();
		ResetRoutingMocks();
	}
};

TEST(NwkPacketTestGroup, TestSetup)
{
	CHECK(1);
}

TEST(NwkPacketTestGroup, TestUnflatten)
{
	uint8_t extPacketPayloadLen = 20;
	uint8_t extPacketPayload[20];
	extPacketPayload[0] = 0x23;
	extPacketPayload[1] = 0x5F;
	
	//check null
	CM2Packet* unf = UnflattenCM2Packet(extPacketPayloadLen, (uint8_t*)0, PORT_SPIRIT, TRUE);
	CHECK(unf == 0);
	
	//check wrong start bytes
	unf = UnflattenCM2Packet(extPacketPayloadLen, extPacketPayload, PORT_SPIRIT, TRUE);
	CHECK(unf == 0);
}

TEST(NwkPacketTestGroup, TestFlattenUnflatten)
{
	uint8_t pingPayload[3] = {OP_PING,0,0};
	CM2Packet ping = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,	//destination SPIRIT
		PROTOCOL_VERSION,
		0,
		ID_OF_MOCK_DEVICE,	//src
		DIRECT_ADDRESSING,
		MY_ID,		//dest
		0x00,		//no subaddress
		0x03,		//3 byte payload
		pingPayload,//will contain opcode + count
		0x0000		//MAC to be filled out before acking
	};
	ping.mac = CalcMAC(&ping, ID_OF_MOCK_DEVICE);
	uint8_t extPacketPayloadLen;
	uint8_t* extPacketPayload;
	
	//check flatten/unflatten without serial wrapper
	FlattenCM2Packet(&ping, &extPacketPayloadLen, &extPacketPayload, FALSE);
	CM2Packet* unf = UnflattenCM2Packet(extPacketPayloadLen, extPacketPayload, PORT_SPIRIT, FALSE);
	CHECK(PacketsIdentical(&ping, unf));
	
	//free buffers
	FreeMem((void**)&extPacketPayload);
	FreeMem((void**)&unf->command);
	FreeMem((void**)&unf);
	
	//check flatten/unflatten WITH serial wrapper
	FlattenCM2Packet(&ping, &extPacketPayloadLen, &extPacketPayload, TRUE);
	CHECK(extPacketPayload[0] == 0x23);
	CHECK(extPacketPayload[1] == 0x81);
	CHECK(extPacketPayload[2] == extPacketPayloadLen);
	unf = UnflattenCM2Packet(extPacketPayloadLen, extPacketPayload, PORT_SPIRIT, TRUE);
	
	CHECK(PacketsIdentical(&ping, unf));
	
	FreeMem((void**)&extPacketPayload);
	FreeMem((void**)&unf->command);
	FreeMem((void**)&unf);
}

TEST(NwkPacketTestGroup, TestPingAck)
{
	//prepare packets
	uint8_t ackPayload[3] = {OP_LOW_LEVEL_ACK, 0, 0};
	CM2Packet ack = 
	{
		PORT_INTERNAL,
		PORT_SPIRIT,//destination SPIRIT
		PROTOCOL_VERSION,
		0,			//src type
		MY_ID,		//src ID
		DIRECT_ADDRESSING,
		ID_OF_MOCK_DEVICE,	//dest ID
		0x00,		//no subaddress
		0x03,		//3 byte payload
		ackPayload, //will contain opcode + msg MAC
		0x0000		//MAC to be filled out before acking
	};
	
	uint8_t pingPayload[3] = {OP_PING,0,0};
	CM2Packet ping = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0,				//src type
		ID_OF_MOCK_DEVICE,//src ID
		DIRECT_ADDRESSING,
		0x11112222,	//dest
		0x00,		//no subaddress
		0x03,		//3 byte payload
		pingPayload,//will contain opcode + count
		0x0000		//MAC to be filled out before acking
	};
	
	CM2Packet* ping1 = CopyPacket(&ping);
	ping1->mac = CalcMAC(ping1, ID_OF_MOCK_DEVICE);
	
	CM2Packet* ping2 = CopyPacket(&ping);
	ping2->destID = MY_ID;
	ping2->mac = CalcMAC(ping2, ID_OF_MOCK_DEVICE);
	ack.command[1] = ping2->mac >> 8;
	ack.command[2] = ping2->mac;
	ack.mac = CalcMAC(&ack, MY_ID);

	//inject ping not for me
	RoutePacket(ping1);
	
	//give it a chance to process
	for(int i=0; i<3; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify no ack
	CHECK(GetLastTx(2) == 0);
	//CHECK(lastAppRxPacket == 0);
	
	/////////////////////////////////////////////
	
	//set to my address
	ping.destID = MY_ID;
	ping.mac = CalcMAC(ping2, ID_OF_MOCK_DEVICE);
	ack.command[1] = ping2->mac >> 8;
	ack.command[2] = ping2->mac;
	ack.mac = CalcMAC(&ack, MY_ID);
	
	//inject ping
	RoutePacket(ping2);
	
	//give it a chance to process
	for(int i=0; i<3; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify ack
	CHECK(PacketsIdentical(GetLastTx(2), &ack));
}

