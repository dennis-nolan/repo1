extern "C"
{
//#include <stdlib.h>
#include "MemoryMgmtHeader.h"
#include "CM2RoutingHeader.h"
#include "CM2SecurityHeader.h"
#include "CM2RoutingMocks.h"
#include "TimersHeader.h"
}

#include "CppUTest/TestHarness.h"

#define MY_ID 0xCDEF5467
#define MY_TYPE 0xCD
#define MY_NID 0xCDCDABAB

#define ID_OF_MOCK_DEVICE 0xABCD9876

//static uint8_t cmd[32] = {10,9,8,7,6,5,4,3,2,1};

static CM2Packet testPacketBT = 
{
	PORT_INTERNAL,
	PORT_BT,
	0xFF,
	1,
	0,
	DIRECT_ADDRESSING,
	ID_OF_MOCK_DEVICE,	//src
	MY_ID,		//dest
	0xFF,		//no subaddress
	10,			//
	0x0000,		//MAC to be filled out
	cmd,			//
};

/*static CM2Packet testPacketBLE = 
{
	PORT_INTERNAL,
	PORT_BLE,
	1,
	0,
	ID_OF_MOCK_DEVICE,	//src
	DIRECT_ADDRESSING,
	MY_ID,		//dest
	0xFF,		//no subaddress
	13,			//
	cmd,			//
	0x0000		//MAC to be filled out
};

static CM2Packet testPacketSpirit = 
{
	PORT_INTERNAL,
	PORT_SPIRIT,
	1,
	0,
	ID_OF_MOCK_DEVICE,	//src
	DIRECT_ADDRESSING,
	MY_ID,		//dest
	0xFF,		//no subaddress
	19,			//
	cmd,			//
	0x0000		//MAC to be filled out
};

static CM2Packet testPacketSpiritTX = 
{
	PORT_INTERNAL,
	PORT_SPIRIT,
	1,
	0,
	MY_ID,		//src
	DIRECT_ADDRESSING,
	ID_OF_MOCK_DEVICE,		//dest
	0xFF,		//no subaddress
	19,			//
	cmd,			//
	0x0000		//MAC to be filled out
};

static uint8_t ackCmd[3] = {OP_LOW_LEVEL_ACK,0,0};
static CM2Packet testPacketSpiritTXAck = 
{
	PORT_SPIRIT,
	PORT_INTERNAL,
	1,
	0,
	ID_OF_MOCK_DEVICE,	//src
	DIRECT_ADDRESSING,
	MY_ID,		//dest
	0xFF,		//no subaddress
	3,			//
	ackCmd,		//
	0x0000		//MAC to be filled out
};

static CM2Packet testPacketSpiritTX2 = 
{
	PORT_INTERNAL,
	PORT_SPIRIT,
	1,
	0,
	MY_ID,		//src
	DIRECT_ADDRESSING,
	ID_OF_MOCK_DEVICE,		//dest
	0xAA,		//bogus subaddress
	7,			//
	cmd,		//
	0x0000		//MAC to be filled out
};

static uint8_t ackCmd2[3] = {OP_LOW_LEVEL_ACK,0,0};
static CM2Packet testPacketSpiritTXAck2 = 
{
	PORT_SPIRIT,
	PORT_INTERNAL,
	1,
	0,
	ID_OF_MOCK_DEVICE,	//src
	DIRECT_ADDRESSING,
	MY_ID,		//dest
	0xFF,		//no subaddress
	3,			//
	ackCmd2,		//
	0x0000		//MAC to be filled out
};

static CM2Packet testPacketMe = 
{
	PORT_SPIRIT,
	PORT_UNKNOWN,
	1,
	0,
	ID_OF_MOCK_DEVICE,	//src
	DIRECT_ADDRESSING,
	MY_ID,		//dest
	0xFF,		//no subaddress
	31,			//
	cmd,			//
	0x0000		//MAC to be filled out
};

static CM2Packet testPacketMe2 = 
{
	PORT_SPIRIT,
	PORT_UNKNOWN,
	1,
	0,
	ID_OF_MOCK_DEVICE,	//src
	DIRECT_ADDRESSING,
	MY_ID,		//dest
	0xAA,		//bogus subaddress
	31,			//
	cmd,			//
	0x0000		//MAC to be filled out
};

static CM2Packet testPacketNotMe = 
{
	PORT_SPIRIT,
	PORT_UNKNOWN,
	1,
	0,
	ID_OF_MOCK_DEVICE,	//src
	DIRECT_ADDRESSING,
	MY_ID+1,		//dest
	0xFF,		//no subaddress
	16,			//
	cmd,			//
	0x0000		//MAC to be filled out
};

static CM2Packet testPacketBroadcast = 
{
	PORT_SPIRIT,
	PORT_UNKNOWN,
	1,
	0,
	ID_OF_MOCK_DEVICE,	//src
	BROADCAST_ADDRESSING,
	0xFFFFFFFF,	//dest
	0xFF,		//no subaddress
	20,			//
	cmd,			//
	0x0000		//MAC to be filled out
};*/


TEST_GROUP(RoutingTestGroup)
{
	void setup()
	{
		ResetRoutingMocks();
		testPacketBT.mac = CalcMAC(&testPacketBT, ID_OF_MOCK_DEVICE);
		testPacketBLE.mac = CalcMAC(&testPacketBLE, ID_OF_MOCK_DEVICE);
		testPacketSpirit.mac = CalcMAC(&testPacketSpirit, ID_OF_MOCK_DEVICE);
		testPacketSpiritTX.mac = CalcMAC(&testPacketSpiritTX, MY_ID);
		testPacketSpiritTXAck.command[1] = testPacketSpiritTX.mac >> 8;
		testPacketSpiritTXAck.command[2] = testPacketSpiritTX.mac;
		testPacketSpiritTXAck.mac = CalcMAC(&testPacketSpiritTXAck, ID_OF_MOCK_DEVICE);
		testPacketSpiritTX2.mac = CalcMAC(&testPacketSpiritTX2, MY_ID);
		testPacketSpiritTXAck2.command[1] = testPacketSpiritTX2.mac >> 8;
		testPacketSpiritTXAck2.command[2] = testPacketSpiritTX2.mac;
		testPacketSpiritTXAck2.mac = CalcMAC(&testPacketSpiritTXAck2, ID_OF_MOCK_DEVICE);
		TimerInit();
	}
	
	void teardown()
	{
		ResetRoutingMocks();
	}
};

TEST(RoutingTestGroup, TestSetup)
{
	CHECK(1);
}

TEST(RoutingTestGroup, TestRoutingMacros)
{
	uint16_t set1 = 0xAC48, set2;
	
	for (uint8_t i=0; i<15; i++)
	{
		for (uint8_t j=0; j<15; j++)
		{
			set1 += 17;
			
			//identical cases (true)
			set2 = set1;
			CHECK(IS_PORT_SUBSET(set1,set2));
			
			//true cases
			set2 = set1 & (~(1<<i)) & (~(1<<j)); //set 2 will always have more 0's or be equal
			CHECK(IS_PORT_SUBSET(set2,set1));
			
			//false cases (reordered in call)
			if (set1 != set2)
				CHECK(IS_PORT_SUBSET(set1,set2) == FALSE);
		}
	}
}

/*TEST(RoutingTestGroup, TestInitAddDeinit)
{
	Port* testPort = (Port*)GetMem(sizeof(Port));
	testPort->sourceType = PORT_BT;
	testPort->Transmit = Transmit1;
	testPort->Ready = Ready;
	
	RouterInit(MY_TYPE, MY_ID);
	RegisterPort(testPort);
	RouterDeinit();
	
	CHECK(1);
}

TEST(RoutingTestGroup, TestTrivialRoutes)
{
	Port* testPort1 = (Port*)GetMem(sizeof(Port));
	testPort1->sourceType = PORT_BT;
	testPort1->Transmit = Transmit1;
	testPort1->Ready = Ready;
	
	Port* testPort2 = (Port*)GetMem(sizeof(Port));
	testPort2->sourceType = PORT_BLE;
	testPort2->Transmit = Transmit2;
	testPort2->Ready = Ready;
	
	Port* testPort3 = (Port*)GetMem(sizeof(Port));
	testPort3->sourceType = PORT_SPIRIT;
	testPort3->Transmit = Transmit3;
	testPort3->Ready = Ready;
	
	RouterInit(MY_TYPE, MY_ID);
	RegisterPort(testPort1);
	RegisterPort(testPort2);
	RegisterPort(testPort3);
	
	// send on BLE
	RoutePacket(CopyPacket(&testPacketBLE));
	RouteNextPacket();
	// check packet on BLE (transmit2)
	CHECK(GetLastTx(1) == 0);
	CHECK(PacketsIdentical(GetLastTx(2), &testPacketBLE));
	CHECK(GetLastTx(3) == 0);
	ResetRoutingMocks();
	
	// send on BT
	RoutePacket(CopyPacket(&testPacketBT));
	RouteNextPacket();
	// check packet on BT (transmit1)
	CHECK(PacketsIdentical(GetLastTx(1), &testPacketBT));
	CHECK(GetLastTx(2) == 0);
	CHECK(GetLastTx(3) == 0);
	ResetRoutingMocks();
	
	// send on Spirit
	RoutePacket(CopyPacket(&testPacketSpirit));
	RouteNextPacket();
	// check packet on Spirit (transmit3)
	CHECK(GetLastTx(1) == 0);
	CHECK(GetLastTx(2) == 0);
	CHECK(PacketsIdentical(GetLastTx(3), &testPacketSpirit));
	ResetRoutingMocks();
	
	// send on BT
	RoutePacket(CopyPacket(&testPacketBT));
	RouteNextPacket();
	// check packet on BT (transmit1)
	CHECK(PacketsIdentical(GetLastTx(1), &testPacketBT));
	CHECK(GetLastTx(2) == 0);
	CHECK(GetLastTx(3) == 0);
	ResetRoutingMocks();
	
	RouterDeinit();
	CHECK(1);
}

TEST(RoutingTestGroup, TestAddressedToMe)
{
	Port* testPort1 = (Port*)GetMem(sizeof(Port));
	testPort1->sourceType = PORT_BT;
	testPort1->Transmit = Transmit1;
	testPort1->Ready = Ready;
	
	Port* testPort2 = (Port*)GetMem(sizeof(Port));
	testPort2->sourceType = PORT_INTERNAL;
	testPort2->Transmit = Transmit2;
	testPort2->Ready = Ready;
	
	Port* testPort3 = (Port*)GetMem(sizeof(Port));
	testPort3->sourceType = PORT_SPIRIT;
	testPort3->Transmit = Transmit3;
	testPort3->Ready = Ready;
	
	RouterInit(MY_TYPE, MY_ID);
	RegisterPort(testPort3);
	RegisterPort(testPort2);
	RegisterPort(testPort1);
	
	// send packet addressed to me
	RoutePacket(CopyPacket(&testPacketMe));
	RouteNextPacket();
	// check packet on internal (transmit2)
	CHECK(GetLastTx(1) == 0);
	CHECK(PacketsIdentical(GetLastTx(2), &testPacketMe));
	CHECK(GetLastTx(3) != 0);
	CHECK(GetLastTx(3)->command != 0);
	CHECK(GetLastTx(3)->command[0] == OP_LOW_LEVEL_ACK);
	ResetRoutingMocks();
	
	RouterDeinit();
	CHECK(1);
}

TEST(RoutingTestGroup, TestRetransmit1)
{
	Port* testPort1 = (Port*)GetMem(sizeof(Port));
	testPort1->sourceType = PORT_BT;
	testPort1->Transmit = Transmit1;
	testPort1->Ready = Ready;
	
	Port* testPort2 = (Port*)GetMem(sizeof(Port));
	testPort2->sourceType = PORT_INTERNAL;
	testPort2->Transmit = Transmit2;
	testPort2->Ready = Ready;
	
	Port* testPort3 = (Port*)GetMem(sizeof(Port));
	testPort3->sourceType = PORT_SPIRIT;
	testPort3->Transmit = Transmit3;
	testPort3->Ready = Ready;
	
	Port* testPort4 = (Port*)GetMem(sizeof(Port));
	testPort3->sourceType = PORT_USB;
	testPort3->Transmit = Transmit4;
	testPort3->Ready = Ready;
	
	RouterInit(MY_TYPE, MY_ID);
	RegisterPort(testPort3);
	RegisterPort(testPort2);
	RegisterPort(testPort1);
	RegisterPort(testPort4);
	
	// send packet not addressed to me
	RoutePacket(CopyPacket(&testPacketNotMe));
	RouteNextPacket();
	// check packet does not show on internal or spirit, but is forwarded to BT and USB
	CHECK(PacketsIdentical(GetLastTx(1), &testPacketNotMe));
	CHECK(GetLastTx(2) == 0);
	CHECK(GetLastTx(3) == 0);
	CHECK(PacketsIdentical(GetLastTx(4), &testPacketNotMe));
	ResetRoutingMocks();
	
	RouterDeinit();
	CHECK(1);
}

TEST(RoutingTestGroup, TestRetransmit2)
{
	Port* testPort1 = (Port*)GetMem(sizeof(Port));
	testPort1->sourceType = PORT_BT;
	testPort1->Transmit = Transmit1;
	testPort1->Ready = Ready;
	
	Port* testPort2 = (Port*)GetMem(sizeof(Port));
	testPort2->sourceType = PORT_INTERNAL;
	testPort2->Transmit = Transmit2;
	testPort2->Ready = Ready;
	
	Port* testPort3 = (Port*)GetMem(sizeof(Port));
	testPort3->sourceType = PORT_SPIRIT;
	testPort3->Transmit = Transmit3;
	testPort3->Ready = Ready;
	
	Port* testPort4 = (Port*)GetMem(sizeof(Port));
	testPort3->sourceType = PORT_USB;
	testPort3->Transmit = Transmit4;
	testPort3->Ready = Ready;
	
	RouterInit(MY_TYPE, MY_ID);
	RegisterPort(testPort3);
	RegisterPort(testPort2);
	RegisterPort(testPort1);
	RegisterPort(testPort4);
	
	// send broadcast
	RoutePacket(CopyPacket(&testPacketBroadcast));
	RouteNextPacket();
	// check packet does not show on spirit, but is forwarded to BT, USB, and internal since it is a broadcast
	CHECK(PacketsIdentical(GetLastTx(1), &testPacketBroadcast));
	CHECK(PacketsIdentical(GetLastTx(2), &testPacketBroadcast));
	CHECK(PacketsIdentical(GetLastTx(3), 0));
	CHECK(PacketsIdentical(GetLastTx(4), &testPacketBroadcast));
	ResetRoutingMocks();
	
	RouterDeinit();
	CHECK(1);
}

TEST(RoutingTestGroup, TestMultipleAckTx)
{
	Port* testPort1 = (Port*)GetMem(sizeof(Port));
	testPort1->sourceType = PORT_INTERNAL;
	testPort1->Transmit = Transmit1;
	testPort1->Ready = Ready;
	
	Port* testPort2 = (Port*)GetMem(sizeof(Port));
	testPort2->sourceType = PORT_SPIRIT;
	testPort2->Transmit = Transmit2;
	testPort2->Ready = Ready;
	
	RouterInit(MY_TYPE, MY_ID);
	RegisterPort(testPort2);
	RegisterPort(testPort1);
	
	for(int i=0; i<17; i++)
	{
		// send packet addressed to me
		RoutePacket(CopyPacket(&testPacketMe));
		RouteNextPacket();
		// check packet on internal (transmit1)
		CHECK(PacketsIdentical(GetLastTx(1), &testPacketMe));
		// check ack on spirit
		CHECK(GetLastTx(2) != 0);
		CHECK(GetLastTx(2)->command != 0);
		CHECK(GetLastTx(2)->command[0] == OP_LOW_LEVEL_ACK);
		CHECK(GetLastTx(2)->command[1]<<8 | GetLastTx(2)->command[2] == testPacketMe.mac);
		ResetRoutingMocks();
	}
	
	RouterDeinit();
	CHECK(1);
}

TEST(RoutingTestGroup, TestMultipleAckTx_TimingMismatch)
{
	Port* testPort1 = (Port*)GetMem(sizeof(Port));
	testPort1->sourceType = PORT_INTERNAL;
	testPort1->Transmit = Transmit1;
	testPort1->Ready = Ready;
	
	Port* testPort2 = (Port*)GetMem(sizeof(Port));
	testPort2->sourceType = PORT_SPIRIT;
	testPort2->Transmit = Transmit2;
	testPort2->Ready = Ready;
	
	RouterInit(MY_TYPE, MY_ID);
	RegisterPort(testPort2);
	RegisterPort(testPort1);
	
	for(int i=0; i<9; i++)
	{
		// send packet addressed to me
		RoutePacket(CopyPacket(&testPacketMe));
		RoutePacket(CopyPacket(&testPacketMe2));
		
		RouteNextPacket();
		// check packet on internal (transmit1)
		CHECK(PacketsIdentical(GetLastTx(1), &testPacketMe));
		// check ack on spirit
		CHECK(GetLastTx(2) != 0);
		CHECK(GetLastTx(2)->command != 0);
		CHECK(GetLastTx(2)->command[0] == OP_LOW_LEVEL_ACK);
		CHECK(GetLastTx(2)->command[1]<<8 | GetLastTx(2)->command[2] == testPacketMe.mac);
		ResetRoutingMocks();
		
		RouteNextPacket();
		// check packet on internal (transmit1)
		CHECK(PacketsIdentical(GetLastTx(1), &testPacketMe2));
		// check ack on spirit
		CHECK(GetLastTx(2) != 0);
		CHECK(GetLastTx(2)->command != 0);
		CHECK(GetLastTx(2)->command[0] == OP_LOW_LEVEL_ACK);
		CHECK(GetLastTx(2)->command[1]<<8 | GetLastTx(2)->command[2] == testPacketMe2.mac);
		ResetRoutingMocks();
	}
	
	RouterDeinit();
	CHECK(1);
}

TEST(RoutingTestGroup, TestMultipleAckRx)
{
	Port* testPort1 = (Port*)GetMem(sizeof(Port));
	testPort1->sourceType = PORT_INTERNAL;
	testPort1->Transmit = Transmit1;
	testPort1->Ready = Ready;
	
	Port* testPort2 = (Port*)GetMem(sizeof(Port));
	testPort2->sourceType = PORT_SPIRIT;
	testPort2->Transmit = Transmit2;
	testPort2->Ready = Ready;
	
	RouterInit(MY_TYPE, MY_ID);
	RegisterPort(testPort2);
	RegisterPort(testPort1);
	
	for(int i=0; i<17; i++)
	{
		// send packet addressed to mock
		RoutePacket(CopyPacket(&testPacketSpiritTX));
		RouteNextPacket();
		// check packet on spirit (transmit2)
		CHECK(GetLastTx(1) == 0);
		CHECK(PacketsIdentical(GetLastTx(2), &testPacketSpiritTX));
		ResetRoutingMocks();
		// send ack on spirit
		RoutePacket(CopyPacket(&testPacketSpiritTXAck));
		RouteNextPacket();
		// check handled in router
		CHECK(GetLastTx(1) == 0);
		CHECK(GetLastTx(2) == 0);
		ResetRoutingMocks();
	}
	
	RouterDeinit();
	CHECK(1);
}

TEST(RoutingTestGroup, TestMultipleAckRx_TimingMismatch)
{
	Port* testPort1 = (Port*)GetMem(sizeof(Port));
	testPort1->sourceType = PORT_INTERNAL;
	testPort1->Transmit = Transmit1;
	testPort1->Ready = Ready;
	
	Port* testPort2 = (Port*)GetMem(sizeof(Port));
	testPort2->sourceType = PORT_SPIRIT;
	testPort2->Transmit = Transmit2;
	testPort2->Ready = Ready;
	
	RouterInit(MY_TYPE, MY_ID);
	RegisterPort(testPort2);
	RegisterPort(testPort1);
	
	for(int i=0; i<18; i++)
	{
		// send packet1 addressed to mock
		RoutePacket(CopyPacket(&testPacketSpiritTX));
		RouteNextPacket();
		// check packet1 on spirit (transmit2)
		CHECK(GetLastTx(1) == 0);
		CHECK(PacketsIdentical(GetLastTx(2), &testPacketSpiritTX));
		ResetRoutingMocks();
		
		// send packet2 addressed to mock
		RoutePacket(CopyPacket(&testPacketSpiritTX2));
		RouteNextPacket();
		// check packet2 on spirit (transmit2)
		CHECK(GetLastTx(1) == 0);
		CHECK(PacketsIdentical(GetLastTx(2), &testPacketSpiritTX2));
		ResetRoutingMocks();
		
		
		// send ack1 on spirit
		RoutePacket(CopyPacket(&testPacketSpiritTXAck));
		RouteNextPacket();
		// check handled in router
		CHECK(GetLastTx(1) == 0);
		CHECK(GetLastTx(2) == 0);
		ResetRoutingMocks();
		
		// send ack2 on spirit
		RoutePacket(CopyPacket(&testPacketSpiritTXAck2));
		RouteNextPacket();
		// check handled in router
		CHECK(GetLastTx(1) == 0);
		CHECK(GetLastTx(2) == 0);
		ResetRoutingMocks();
	}
	
	RouterDeinit();
	CHECK(1);
}

TEST(RoutingTestGroup, TestAckBufferFull)
{
	Port* testPort1 = (Port*)GetMem(sizeof(Port));
	testPort1->sourceType = PORT_INTERNAL;
	testPort1->Transmit = Transmit1;
	testPort1->Ready = Ready;
	
	Port* testPort2 = (Port*)GetMem(sizeof(Port));
	testPort2->sourceType = PORT_SPIRIT;
	testPort2->Transmit = Transmit2;
	testPort2->Ready = Ready;
	
	RouterInit(MY_TYPE, MY_ID);
	RegisterPort(testPort2);
	RegisterPort(testPort1);
	
	for(int i=0; i<18; i++)
	{
		// send packet1 addressed to mock
		RoutePacket(CopyPacket(&testPacketSpiritTX));
		RouteNextPacket();
		// check packet1 on spirit (transmit2)
		CHECK(GetLastTx(1) == 0);
		CHECK(PacketsIdentical(GetLastTx(2), &testPacketSpiritTX));
		ResetRoutingMocks();
		
		// send packet2 addressed to mock
		RoutePacket(CopyPacket(&testPacketSpiritTX2));
		RouteNextPacket();
		// check packet2 on spirit (transmit2)
		CHECK(GetLastTx(1) == 0);
		CHECK(PacketsIdentical(GetLastTx(2), &testPacketSpiritTX2));
		ResetRoutingMocks();
	}
	
	RouterDeinit();
	CHECK(1);
}

TEST(RoutingTestGroup, TestInputBufferFull)
{
	Port* testPort1 = (Port*)GetMem(sizeof(Port));
	testPort1->sourceType = PORT_INTERNAL;
	testPort1->Transmit = Transmit1;
	testPort1->Ready = Ready;
	
	Port* testPort2 = (Port*)GetMem(sizeof(Port));
	testPort2->sourceType = PORT_SPIRIT;
	testPort2->Transmit = Transmit2;
	testPort2->Ready = Ready;
	
	RouterInit(MY_TYPE, MY_ID);
	RegisterPort(testPort2);
	RegisterPort(testPort1);
	
	CM2Packet* copied;
	for(int i=0; i<29; i++)
	{
		// send packet1 addressed to mock
		copied = CopyPacket(&testPacketSpiritTX);
		if (!RoutePacket(copied))
		{
			FreeMem((void**)&copied->command);
			FreeMem((void**)&copied);
		}
		copied = CopyPacket(&testPacketSpiritTX2);
		if (!RoutePacket(copied))
		{
			FreeMem((void**)&copied->command);
			FreeMem((void**)&copied);
		}
		// execute route half as fast as they come in
		RouteNextPacket();
		ResetRoutingMocks();
	}
	
	RouterDeinit();
	CHECK(1);
}*/
