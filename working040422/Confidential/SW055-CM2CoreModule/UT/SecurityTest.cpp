extern "C"
{
//#include <stdlib.h>
/*#include "MemoryMgmtHeader.h"
#include "TimersHeader.h"
#include "ThreadingHeader.h"
#include "CM2CoreHeader.h"*/
#include "CM2SecurityHeader.h"
//#include "CM2UTMocks.h"
/*#include "CM2UTUtils.h"
#include "CM2PairingTableModeHeader.h"
#include "CM2RoutingHeader.h"
#include "CM2RoutingMocks.h"*/
}

#include "CppUTest/TestHarness.h"

TEST_GROUP(SecurityTestGroup)
{
	void setup()
	{
		
	}
	
	void teardown()
	{
		
	}
};

TEST(SecurityTestGroup, TestLFSR16)
{
	//Checks that the LFSR is maximal with known-good exponents
	
	uint16_t latest = LinearFeedbackShiftRegister(4,0x80CB);
	uint16_t prev = 0, first = latest;
	uint32_t period = 0;
	
	while(period == 0 || latest != first)
	{
		prev = latest;
		latest = LinearFeedbackShiftRegister(prev,0x80CB);
		period++;
		
		CHECK(prev != latest);
	}
	
	CHECK(period == 0xFFFF);
}

TEST(SecurityTestGroup, TestRand16)
{
	//simple randomness tests for the PRNG
	
	uint32_t sum;
	uint16_t val, first = Rand16();
	bool valChanged = FALSE;
	
	for(int i=0; i<3; i++)
	{
		sum=0;
		for(long j=0; j<100000; j++)
		{
			val = Rand16();
			sum += val;
			
			if (val != first)
				valChanged = TRUE;
		}
		CHECK(sum < 3300000000);
		CHECK(sum > 3200000000);
	}
	
	CHECK(valChanged);
	
	//3276800000
}

uint32_t tick = 91;
uint32_t GetTicks(void)
{
	tick += 77;
	return tick;
}

TEST(SecurityTestGroup, TestRand16_WithTicks)
{
	//simple randomness tests for the PRNG with ticks
	
	RegisterTickInterface(GetTicks);
	
	uint32_t sum;
	uint16_t val, first = Rand16();
	bool valChanged = FALSE;
	
	for(int i=0; i<3; i++)
	{
		sum=0;
		for(long j=0; j<100000; j++)
		{
			val = Rand16();
			sum += val;
			
			if (val != first)
				valChanged = TRUE;
		}
		CHECK(sum < 3300000000);
		CHECK(sum > 3200000000);
	}
	
	CHECK(valChanged);
	
	//3276800000
}

TEST(SecurityTestGroup, TestChallengeResp)
{
	uint16_t chal = 0xf234;
	uint32_t CID = 0xFEDC2134, RID = 0x524A6111;
	
	//check that they always match
	for (int i=0; i<10; i++)
	{
		chal += i;
		CID += i;
		RID += i;
		CHECK(CalcChallengeResponse(chal, CID, RID) == CalcChallengeResponse(chal, CID, RID));
	}
	
	//check that it changes frequently
	uint16_t first = CalcChallengeResponse(chal, CID, RID);
	uint8_t count = 0;
	for (int i=0; i<100; i++)
	{
		chal++;
		if (first == CalcChallengeResponse(chal, CID, RID))
			count++;
	}
	CHECK(count < 10);
}
