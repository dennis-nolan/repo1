
#include "CM2RangeTestHeader.h"
#include "CM2CoreHeader.h"
#include "TimersHeader.h"
#include "CM2SecurityHeader.h"
#include "CM2RoutingHeader.h"
#include "RandomHeader.h"

static uint16_t lostPackets = 0, lastMAC = 0, pingCount = 0xCD, numToSend = 0xCD;
static uint8_t minDelay, delayVariation;

static CM2Packet pingPacket = 
{
	PORT_UNKNOWN,			//packet source
	PORT_UNKNOWN,			//destination SPIRIT
	0,						//retries
	PROTOCOL_VERSION,		//protocol ver
	0,						//src type
	0,						//dest type
	0,						//src ID
	0xFFFFFFFF,				//dest ID
	0x00,					//subaddress
	0x03,					//length
	0x0000,					//MAC to be filled out
	{OP_PING,0xFF,0xFF}		//command/payload
};

static void SendNext(uint8_t dummy)
{
	if (pingCount >= numToSend)
		return;
	
	pingPacket.command[1] = pingCount >> 8;
	pingPacket.command[2] = (uint8_t)pingCount;
	pingPacket.mac = CalcMAC(&pingPacket, pingPacket.srcID);
	lastMAC = pingPacket.mac;
	
	if (RoutePacket(&pingPacket) == FALSE)
        AbortRangeTest();//numToSend = 0;//running = FALSE;
	pingCount++;
	
	if (pingCount >= numToSend)
		AbortRangeTest();
	else if (delayVariation > 0)
	{
		//change timer delay for next tick
		uint16_t tempDelay = (uint16_t)(minDelay + ((float)delayVariation * GetRandom16())/0xFFFF);
		if (RegisterTimer(SendNext, 0, tempDelay, TRUE, FALSE) == FALSE)
			AbortRangeTest();
	}
}

void BeginRangeTest(uint32_t ID, uint16_t numPackets, uint8_t spacingMin, uint8_t spacingMax)
{
	AbortRangeTest();
	lostPackets = 0;
	minDelay = spacingMin;
	if (spacingMin < spacingMax)
		delayVariation = spacingMax - spacingMin;
	else
		delayVariation = 0;
	
	pingPacket.deviceSource = PORT_INTERNAL;
	pingPacket.deviceDest = PORT_SPIRIT;
	pingPacket.srcType = GetType();
	pingPacket.srcID = GetID();
	pingPacket.destType = 0;
	pingPacket.destID = ID;
	
	pingCount = 0;
	numToSend = numPackets;
	
	uint16_t tempDelay = minDelay;
	if (delayVariation > 0)
		tempDelay = (uint16_t)(minDelay + ((float)delayVariation * GetRandom16())/0xFFFF);

	if (RegisterTimer(SendNext, 0, tempDelay, TRUE, FALSE) == FALSE)
		numToSend = 0;
}

void AbortRangeTest(void)
{
	CancelTimer(SendNext,0);
	numToSend = 0;
	lastMAC = 0;
}

bool RangeTestInProgress(void)
{ return (bool)(pingCount<numToSend); }

uint16_t GetRangeTestPacketsLost(void)
{ return lostPackets; }

void RangeTestAckHandler(uint16_t MAC, bool success)
{
	if (lastMAC == MAC)
	{
		if (success)
			lastMAC = 0;
		else
			lostPackets++;
	}
	return;
}
