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
#include "FlashMocks.h"
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

extern bool invalidWriteOcurred;
extern bool invalidAddress;

static bool PacketRxDummy(CM2Packet* packet)
{ return FALSE; }

TEST_GROUP(ExtPacketTestGroup)
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

TEST(ExtPacketTestGroup, TestSetup)
{
	CHECK(1);
}

TEST(ExtPacketTestGroup, TestSmallExtendedPacket)
{
	uint8_t pingPayload[3] = {OP_PING,0,0};
	CM2Packet ping = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0,
		ID_OF_MOCK_DEVICE,	//src
		DIRECT_ADDRESSING,
		MY_ID,		//dest
		0xFF,		//no subaddress
		0x03,		//3 byte payload
		pingPayload,//will contain opcode + count
		0x0000		//MAC to be filled out before acking
	};
	ping.mac = CalcMAC(&ping, ID_OF_MOCK_DEVICE);
	uint8_t extPacketPayloadLen;
	uint8_t* extPacketPayload;
	FlattenCM2Packet(&ping, &extPacketPayloadLen, &extPacketPayload, FALSE);
	
	uint32_t packetID = 0x12345678;
	uint8_t extStartPayload[9] = {OP_EX_PACKET_START, packetID>>24, packetID>>16, packetID>>8, packetID, 0, 0, extPacketPayloadLen, 0};
	CM2Packet extStart = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0,
		ID_OF_MOCK_DEVICE,	//src
		DIRECT_ADDRESSING,
		MY_ID,		//dest
		0xFF,		//no subaddress
		9,			//payload len
		extStartPayload,
		0x0000		//MAC to be filled out
	};
	extStart.mac = CalcMAC(&extStart, ID_OF_MOCK_DEVICE);
	CM2Packet* extStartInjectable = CopyPacket(&extStart);
	
	
	//inject start packet
	RoutePacket(extStartInjectable);
	
	//give it a chance to process
	for(int i=0; i<3 && GetLastTx(2)==0; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify ack
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, extStart.mac, MY_ID));
	ResetRoutingMocks();
	
	//first data packet
	uint8_t extdataPayload1[19] = {OP_EX_PACKET_DATA/*OP_EXT_PACKET_DATA*/, packetID>>24, packetID>>16, packetID>>8, packetID, 0, 1};
	for(uint8_t i=0; i<12; i++)
	{ extdataPayload1[7+i] = extPacketPayload[i];}
	CM2Packet extData = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0,
		ID_OF_MOCK_DEVICE,	//src
		DIRECT_ADDRESSING,
		MY_ID,		//dest
		0xFF,		//no subaddress
		19,			//12 byte payload
		extdataPayload1,
		0x0000		//MAC to be filled out
	};
	extData.mac = CalcMAC(&extData, ID_OF_MOCK_DEVICE);
	CM2Packet* extDataInjectable = CopyPacket(&extData);
	
	
	//inject data packet 1
	RoutePacket(extDataInjectable);
	
	//give it a chance to process
	for(int i=0; i<3 && GetLastTx(2)==0; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify ack
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, extData.mac, MY_ID));
	ResetRoutingMocks();
	
	//second data packet
	uint8_t extdataPayload2[9] = {OP_EX_PACKET_DATA, packetID>>24, packetID>>16, packetID>>8, packetID, 0, 2};
	for(uint8_t i=0; i<2; i++)
	{ extdataPayload2[7+i] = extPacketPayload[i+12];}
	extData.commandLen = 9;
	extData.command = extdataPayload2;
	extData.mac = CalcMAC(&extData, ID_OF_MOCK_DEVICE);
	extDataInjectable = CopyPacket(&extData);
	
	
	//inject data packet 2
	RoutePacket(extDataInjectable);
	
	//give it a chance to process
	for(int i=0; i<3 && GetLastTx(2)==0; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify ack
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, extData.mac, MY_ID));
	ResetRoutingMocks();
	
	//final data packet
	uint8_t extdataPayload3[11] = {OP_EX_PACKET_DATA, packetID>>24, packetID>>16, packetID>>8, packetID, 0, 2};
	for(uint8_t i=0; i<4; i++)
	{ extdataPayload3[7+i] = extPacketPayload[i+14];}
	extData.commandLen = 11;
	extData.command = extdataPayload3;
	extData.mac = CalcMAC(&extData, ID_OF_MOCK_DEVICE);
	extDataInjectable = CopyPacket(&extData);
	
	
	//inject data packet 3
	RoutePacket(extDataInjectable);
	
	//give it a chance to process
	for(int i=0; i<3 && GetLastTx(2)==0; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify ack
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, extData.mac, MY_ID));
	ResetRoutingMocks();

	
	//give reconstructed packet a chance to process
	for(int i=0; i<3; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify ack for reconstructed ping
	CHECK(GetLastTx(2) != 0);
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, ping.mac, MY_ID));
	
	FreeMem((void**)&extPacketPayload);
}

TEST(ExtPacketTestGroup, TestSmallFWUpdate)
{
	RegisterFlashInterface(FlashGetMaxAppSize(), FlashEraseScratch, FlashWriteScratch);
	
	uint8_t firmware[] = "This is the test firmware. Don't actually try to run it, it probably won't work very well!!";
	uint8_t length = 92;
	
	//create start packet
	uint32_t packetID = 0x9222AAFE;
	uint8_t extStartPayload[9] = {OP_EX_PACKET_START, packetID>>24, packetID>>16, packetID>>8, packetID, length>>16, length>>8, length, 1};
	CM2Packet extStart = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0,
		ID_OF_MOCK_DEVICE,	//src
		DIRECT_ADDRESSING,
		MY_ID,				//dest
		0xFF,				//no subaddress
		9,					//payload len
		extStartPayload,
		0x0000				//MAC to be filled out
	};
	extStart.mac = CalcMAC(&extStart, ID_OF_MOCK_DEVICE);
	CM2Packet* extStartInjectable = CopyPacket(&extStart);
	
	//inject start packet
	RoutePacket(extStartInjectable);
	
	//give it a chance to process
	for(int i=0; i<3 && GetLastTx(2)==0; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify ack
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, extStart.mac, MY_ID));
	ResetRoutingMocks();
	
	//first data packet
	uint8_t extdataPayload1[15] = {OP_EX_PACKET_DATA, packetID>>24, packetID>>16, packetID>>8, packetID, 0, 0};
	for(uint8_t i=0; i<8; i++)
	{ extdataPayload1[7+i] = firmware[i];}
	CM2Packet extData = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0,
		ID_OF_MOCK_DEVICE,	//src
		DIRECT_ADDRESSING,
		MY_ID,		//dest
		0xFF,		//no subaddress
		15,			//12 byte payload
		extdataPayload1,
		0x0000		//MAC to be filled out
	};
	extData.mac = CalcMAC(&extData, ID_OF_MOCK_DEVICE);
	CM2Packet* extDataInjectable = CopyPacket(&extData);
	
	//inject data packet 1
	RoutePacket(extDataInjectable);
	
	//give it a chance to process
	for(int i=0; i<3 && GetLastTx(2)==0; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify ack
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, extData.mac, MY_ID));
	ResetRoutingMocks();
	
	//second data packet
	uint8_t extdataPayload2[31] = {OP_EX_PACKET_DATA, packetID>>24, packetID>>16, packetID>>8, packetID, 0, 1};
	for(uint8_t i=0; i<24; i++)
	{ extdataPayload2[7+i] = firmware[i+8];}
	CM2Packet extData2 = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0,
		ID_OF_MOCK_DEVICE,	//src
		DIRECT_ADDRESSING,
		MY_ID,		//dest
		0xFF,		//no subaddress
		31,			//12 byte payload
		extdataPayload2,
		0x0000		//MAC to be filled out
	};
	extData2.mac = CalcMAC(&extData2, ID_OF_MOCK_DEVICE);
	extDataInjectable = CopyPacket(&extData2);
	
	//inject data packet 2
	RoutePacket(extDataInjectable);
	
	//give it a chance to process
	for(int i=0; i<3 && GetLastTx(2)==0; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify ack
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, extData2.mac, MY_ID));
	ResetRoutingMocks();
	
	//third data packet
	uint8_t extdataPayload3[19] = {OP_EX_PACKET_DATA, packetID>>24, packetID>>16, packetID>>8, packetID, 0, 2};
	for(uint8_t i=0; i<12; i++)
	{ extdataPayload3[7+i] = firmware[i+32];}
	CM2Packet extData3 = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0,
		ID_OF_MOCK_DEVICE,	//src
		DIRECT_ADDRESSING,
		MY_ID,		//dest
		0xFF,		//no subaddress
		19,			//12 byte payload
		extdataPayload3,
		0x0000		//MAC to be filled out
	};
	extData3.mac = CalcMAC(&extData3, ID_OF_MOCK_DEVICE);
	extDataInjectable = CopyPacket(&extData3);
	
	//inject data packet 3
	RoutePacket(extDataInjectable);
	
	//give it a chance to process
	for(int i=0; i<3 && GetLastTx(2)==0; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify ack
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, extData3.mac, MY_ID));
	ResetRoutingMocks();
	
	//final data packet
	uint8_t extdataPayload4[55] = {OP_EX_PACKET_DATA, packetID>>24, packetID>>16, packetID>>8, packetID, 0, 3};
	for(uint8_t i=0; i<48; i++)
	{ extdataPayload4[7+i] = firmware[i+44];}
	CM2Packet extData4 = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0,
		ID_OF_MOCK_DEVICE,	//src
		DIRECT_ADDRESSING,
		MY_ID,		//dest
		0xFF,		//no subaddress
		55,			//12 byte payload
		extdataPayload4,
		0x0000		//MAC to be filled out
	};
	extData4.mac = CalcMAC(&extData4, ID_OF_MOCK_DEVICE);
	extDataInjectable = CopyPacket(&extData4);
	
	//inject final data packet
	RoutePacket(extDataInjectable);
	
	//give it a chance to process
	for(int i=0; i<3 && GetLastTx(2)==0; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify ack
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, extData4.mac, MY_ID));
	ResetRoutingMocks();
	
	//verify data arrived successfully in scratch
	CHECK(invalidWriteOcurred == FALSE);
	CHECK(invalidAddress == FALSE);
	CHECK(FlashMockVerify(firmware, length));
}

