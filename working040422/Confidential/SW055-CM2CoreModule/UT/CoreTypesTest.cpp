extern "C"
{
#include "CM2CoreTypesHeader.h"
#include "string.h"
}

#include "CppUTest/TestHarness.h"

TEST_GROUP(CoreTypesTestGroup)
{
	void setup()
	{
		
	}
	
	void teardown()
	{
		
	}
};

static CM2Packet testPacket1 = 
{
	PORT_SPIRIT,			//packet source
	PORT_INTERNAL,			//destination port
	0,						//retries
	1,						//protocol ver
	0x15,					//src type
	0xFA,					//dest type
	0x12345678,				//src ID
	0xcdcdcdcd,				//dest ID
	0x12,					//subaddress
	7,						//cmd length
	0x4567,					//MAC
	{7,6,5,4,3,2,1},		//command/payload
};
static CM2Packet resultpacket;
static uint8_t buf1[128], buf2[128];
static uint8_t len1, len2;

//Checks that flatten and unflatten work as expected
TEST(CoreTypesTestGroup, FlattenUnflattenFlatten)
{
	//flatten testPacket1 into buf1
	FlattenCM2PacketStatic(&testPacket1, &len1, buf1, TRUE);
	
	//unflatten buf1 into a packet
	CHECK(UnflattenCM2PacketStatic(&resultpacket, len1, buf1, PORT_SPIRIT, TRUE) == TRUE);
	
	//check that the packet matches the original
	CHECK(PacketsIdentical(&testPacket1, &resultpacket) == TRUE);
	
	//flatten the resulting packet into buf2
	FlattenCM2PacketStatic(&resultpacket, &len2, buf2, TRUE);
	
	//compare this flatten to the first one
	CHECK(len1 > 0);
	CHECK(len1 == len2);
	CHECK(memcmp(buf1,buf2,len1) == 0);
}
