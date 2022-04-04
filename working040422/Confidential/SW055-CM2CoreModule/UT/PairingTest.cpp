extern "C"
{
//#include <stdlib.h>
#include "MemoryMgmtHeader.h"
#include "TimersHeader.h"
#include "ThreadingHeader.h"
#include "CM2CoreHeader.h"
#include "CM2SecurityHeader.h"
//#include "CM2UTMocks.h"
#include "CM2UTUtils.h"
#include "CM2PairingTableModeHeader.h"
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

TEST_GROUP(PairingTestGroup)
{
	void setup()
	{
		//ResetMocks();
		TimerInit();
		ThreadingInit();
		CM2CoreInit();
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

TEST(PairingTestGroup, TestPairSetupTeardown)
{
	CHECK(1);
}

TEST(PairingTestGroup, TestPairConfigDeinit)
{
	uint32_t* testTable = (uint32_t*)GetMem(4 * 8);
	testTable[0] = 0x12345678;
	testTable[1] = 0xAABBCCDD;
	testTable[2] = ID_OF_MOCK_DEVICE;
	testTable[3] = 0x11223344;
	testTable[4] = 0;
	testTable[5] = 0;
	testTable[6] = 0;
	testTable[7] = 0;
	
	//configure with a table with 4 devices paired and 4 empty slots
	CM2CoreConfigure(MY_ID, MY_TYPE, MY_NID, testTable, 4, 8);
	CHECK(1);
}

TEST(PairingTestGroup, TestPairAddRemove)
{
	//fail - no table
	CHECK(Pair(0xFEDCBA98) == false);
	CHECK(Unpair(0x55667788) == false);
	CHECK(IsPaired(0x22334455) == false);

	uint32_t* testTable = (uint32_t*)GetMem(4 * 5);
	testTable[0] = 0x12345678;
	testTable[1] = 0xAABBCCDD;
	testTable[2] = ID_OF_MOCK_DEVICE;
	testTable[3] = 0x11223344;
	testTable[4] = 0;
	
	//configure with a table with 4 devices paired and 1 empty slot
	CM2CoreConfigure(MY_ID, MY_TYPE, MY_NID, testTable, 4, 5);
	CHECK(IsPaired(0x22334455) == false);
	
	//pair success
	CHECK(Pair(0xABCDABCD));
	CHECK(IsPaired(0xABCDABCD));
	
	//pair fail - full
	CHECK(Pair(0x55554444) == false);
	CHECK(IsPaired(0x55554444) == false);
	
	//unpair success
	CHECK(Unpair(0xABCDABCD));
	CHECK(IsPaired(0xABCDABCD) == false);
	
	//unpair fail - not in table
	CHECK(Unpair(0x55554444) == false);
	CHECK(IsPaired(0x55554444) == false);
}

TEST(PairingTestGroup, TestGetPaired)
{
	uint32_t* readTable = 0;
	uint8_t readLength = 0;
	
	//nothing has been configured yet, so should be empty
	GetPairedDevices(&readTable, &readLength);
	CHECK(readTable == 0);
	CHECK(readLength == 0);
	
	uint32_t* testTable = (uint32_t*)GetMem(4 * 3);
	testTable[0] = 0x12345678;
	testTable[1] = 0xAABBCCDD;
	testTable[2] = 0;
	
	//configure with a table with 2 devices paired and 1 empty slot
	CM2CoreConfigure(MY_ID, MY_TYPE, MY_NID, testTable, 2, 3);
	
	//should return only the filled out part of the table
	GetPairedDevices(&readTable, &readLength);
	CHECK(readTable != 0);
	CHECK(readLength == 2);
	CHECK(readTable[0] == 0x12345678);
	CHECK(readTable[1] == 0xAABBCCDD);
	FreeMem((void**)&readTable);
	readTable = 0;
	
	//pair a new device and check again
	CHECK(Pair(0xABCDABCD));
	GetPairedDevices(&readTable, &readLength);
	CHECK(readTable != 0);
	CHECK(readLength == 3);
	CHECK(readTable[0] == 0x12345678);
	CHECK(readTable[1] == 0xAABBCCDD);
	CHECK(readTable[2] == 0xABCDABCD);
	FreeMem((void**)&readTable);
	readTable = 0;
	
	//unpair and check again
	CHECK(Unpair(0xABCDABCD));
	GetPairedDevices(&readTable, &readLength);
	CHECK(readTable != 0);
	CHECK(readLength == 2);
	CHECK(readTable[0] == 0x12345678);
	CHECK(readTable[1] == 0xAABBCCDD);
	FreeMem((void**)&readTable);
	readTable = 0;
}

TEST(PairingTestGroup, TestRxChallenge)
{
	uint32_t* testTable = (uint32_t*)GetMem(4);
	*testTable = ID_OF_MOCK_DEVICE; //only this device paired
	CM2CoreConfigure(MY_ID, MY_TYPE, MY_NID, testTable, 1, 1);
	
	//setup as a remote
	SetPromiscuousMode(TRUE);
	
	//inject challenge
	uint16_t challengeSent = GenerateChallenge();
	uint8_t chlng[3] = {OP_CHALLENGE, challengeSent>>8, challengeSent};
	CM2Packet challenge = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0xFF,				//src type
		ID_OF_MOCK_DEVICE,	//src ID
		DIRECT_ADDRESSING, 	//dest type
		MY_ID,				//dest ID
		0x00,				//no subaddress
		0x03,				//3 byte length
		chlng,				//command
		0x0000				//MAC to be filled out
	};
	challenge.mac = CalcMAC(&challenge, ID_OF_MOCK_DEVICE);
	RoutePacket(CopyPacket(&challenge));
	
	RouteNextPacket();
	CHECK(GetLastTx(2)->command[0] == OP_LOW_LEVEL_ACK);
	ResetRoutingMocks();
	
	//give it a chance to process
	/*for(int i=0; i<3; i++)
	{
		Tick();
		RunAllThreads();
		
		if (GetLastTx(2) != 0 && GetLastTx(2)->command[0] == OP_LOW_LEVEL_ACK)
			ResetRoutingMocks(); //handle the ack
	}*/
	
	RouteNextPacket();
	//verify response
	CHECK(GetLastTx(2) != 0);
	CHECK(GetLastTx(2)->command[0] == OP_CHALLENGE_RESP);
	uint16_t response = CalcChallengeResponse(challengeSent, ID_OF_MOCK_DEVICE, GetLastTx(2)->srcID);
	CHECK(response == (GetLastTx(2)->command[1]<<8) | GetLastTx(2)->command[2]);
}

/*TEST(PairingTestGroup, TestEnterPairMode)
{
	uint32_t* testTable = (uint32_t*)GetMem(4 * 3);
	testTable[0] = 0x12345678;
	testTable[1] = 0;
	testTable[2] = 0;
	
	//configure with a table with 1 devices paired and 2 empty slots
	CM2CoreConfigure(MY_ID, MY_TYPE, MY_NID, testTable, 1, 3);
	
	//seed PRNG
	uint8_t* seed = (uint8_t*)GetMem(8);
	seed[0]=230;
	seed[1]=55;
	seed[2]=156;
	seed[3]=192;
	seed[4]=12;
	seed[5]=1;
	seed[6]=94;
	seed[7]=99;
	SeedRNG(seed, 8);
	
	//create a NAME packet
	uint8_t cmdName[3] = {OP_NAME,0,0};
	CM2Packet pktName = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0,
		ID_OF_MOCK_DEVICE,	//src
		DIRECT_ADDRESSING,
		MY_ID,		//dest
		0x00,		//no subaddress
		0x03,		//3 byte payload
		cmdName,	//
		0x0000		//MAC to be filled out before acking
	};
	pktName.mac = CalcMAC(&pktName, ID_OF_MOCK_DEVICE);
	
	//create a RESPONSE packet
	uint8_t cmdResp[3] = {OP_CHALLENGE_RESP,0,0};
	CM2Packet pktResp = 
	{
		PORT_SPIRIT,
		PORT_INTERNAL,
		PROTOCOL_VERSION,
		0,
		ID_OF_MOCK_DEVICE,	//src
		DIRECT_ADDRESSING,
		MY_ID,		//dest
		0x00,		//no subaddress
		0x03,		//3 byte payload
		cmdResp,	//
		0x0000		//MAC to be filled out before acking
	};
	pktResp.mac = CalcMAC(&pktResp, ID_OF_MOCK_DEVICE);

	//----------------------------------------------------------------------
	
	//verify unpaired
	CHECK(IsPaired(ID_OF_MOCK_DEVICE) == false);
	//inject packet from unpaired device before entering pair mode
	RoutePacket(CopyPacket(&pktName));
	RouteNextPacket();
	//verify low level ack
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, pktName.mac, MY_ID));
	
	//----------------------------------------------------------------------

	//enter pair mode
	SetPairingMode(TRUE);
	//inject packet from unpaired device
	RoutePacket(CopyPacket(&pktName));
	RouteNextPacket();
	
	//verify ack
	//CHECK(GetLastTx(2)->command[0] == OP_LOW_LEVEL_ACK);
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, pktName.mac, MY_ID));
	
	//give it a chance to process
	for(int i=0; i<3; i++)
	{
		Tick();
		RunAllThreads();
	}
	
	//verify challenge
	CHECK(GetLastTx(2)->command[0] == OP_CHALLENGE);
	uint16_t challenge = (GetLastTx(2)->command[1]<<8) | GetLastTx(2)->command[2];
	//inject response
	uint16_t response = CalcChallengeResponse(challenge, GetLastTx(2)->srcID, ID_OF_MOCK_DEVICE);
	pktResp.command[1] = response>>8;
	pktResp.command[2] = response;
	pktResp.mac = CalcMAC(&pktResp, ID_OF_MOCK_DEVICE);
	RoutePacket(CopyPacket(&pktResp));
	RouteNextPacket();
	//give it a chance to process
	for(int i=0; i<3; i++)
	{
		Tick();
		RunAllThreads();
	}
	//verify added to pairing table
	CHECK(IsPaired(ID_OF_MOCK_DEVICE));
	
	//----------------------------------------------------------------------

	//inject packet from device
	RoutePacket(CopyPacket(&pktName));
	RouteNextPacket();
	//give it a chance to process
	for(int i=0; i<3; i++)
	{
		Tick();
		RunAllThreads();
	}
	//verify ack
	CHECK(CheckAck(GetLastTx(2), ID_OF_MOCK_DEVICE, MY_ID, pktName.mac, MY_ID));
}


*/


