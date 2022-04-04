/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
******************************* END VERSION INFO ******************************/

//#include <stdlib.h>
//#include "MemoryMgmtHeader.h"
#include "CM2SecurityHeader.h"

static uint16_t AddToMAC(uint16_t MAC, uint8_t* message, uint8_t length);
//static uint16_t CalcChallengeResponse(uint16_t challenge, uint32_t challengerID, uint32_t responderID);
//static uint16_t GenerateChallenge(void);


static uint16_t keyTable[16] = {7847 ,37502,41095,28596,37454,20608,2432 ,34813,
						 		20968,49911,33753,18272,57386,15096,48773,61868};

/*static uint16_t RNGTable[16] = {2336 ,58410,60436,10293,46635,9703 ,41438,43146,
						 		16593,64706,18201,18704,62905,1676 ,31244,60283};*/


static uint16_t seed16 = 0x99A2;
static uint32_t (*tickInterface)(void) = 0;

//Galois LFSR - maximal LFSR if proper exponents are used.
uint16_t LinearFeedbackShiftRegister(uint16_t seed, uint16_t exponents)
{
	uint8_t outputBit = seed & 1;   // Get LSB (i.e., the output bit).
	seed >>= 1;                		// Shift register */
	if (outputBit)                  // If the output bit is 1, apply toggle mask.
		seed ^= exponents;
	return seed;
}

uint16_t Rand16(void)
{
	//if we were given an interface to get the tick count, use it.
	if (tickInterface != 0)
	{
		uint32_t result = tickInterface();
		seed16 ^= (result >> 16);
		seed16 ^= (result & 0xFFFF);
	}
	
	//use the LFSR to get the next number and save as next seed
	seed16 = LinearFeedbackShiftRegister(seed16, 0x9D0D);
	
	return seed16;
}

void RegisterTickInterface(uint32_t (*GetTicks)(void))
{
	tickInterface = GetTicks;
}

/*void CM2SecurityDeinit(void)
{
	
}*/

static uint16_t AddToMAC(uint16_t MAC, uint8_t* message, uint8_t length)
{
	/* MAC Algorithm:
		1. Split off the next nibble of the message
		2. Use nibble as an index into the key table
		3. XOR that value into the current MAC
		4. Do a 1-bit circular left shift on the MAC (keeping in mind that the value is 16-bits)
		5. Repeat for every message nibble
	Since the message come in bytes and not nibbles we do 2 nibbles per loop. */
	
	//uint16_t MAC = 0;
	
	for(uint8_t i=0; i<length; i++)
	{
		//Use the first nibble as an index into the key table
		//XOR this value with the current MAC
		MAC ^= keyTable[message[i] >> 4];
		
		//Rotate left by 1 bit
		MAC = (MAC << 1) | (MAC >> 15);
		
		//Do the exact same thing with the second nibble
		MAC ^= keyTable[0x0F & message[i]];
		MAC = (MAC << 1) | (MAC >> 15);
	}
	
	return MAC;
}

uint16_t CalcMAC(CM2Packet* packet, uint32_t NID)
{
	uint16_t mac = 0;
	uint8_t nidArray[4] = {NID>>24, NID>>16, NID>>8, NID};
	uint8_t destArray[4] = {packet->destID>>24, packet->destID>>16, packet->destID>>8, packet->destID};
	uint8_t srcArray[4] = {packet->srcID>>24, packet->srcID>>16, packet->srcID>>8, packet->srcID};
	
	mac = AddToMAC(mac, nidArray, 4);
	mac = AddToMAC(mac, &(packet->protocolVer), 1);
	mac = AddToMAC(mac, &(packet->srcType), 1);
	mac = AddToMAC(mac, srcArray, 4);
	mac = AddToMAC(mac, &(packet->destType), 1);
	mac = AddToMAC(mac, destArray, 4);
	mac = AddToMAC(mac, &(packet->subaddress), 1);
	mac = AddToMAC(mac, &(packet->commandLen), 1);
	mac = AddToMAC(mac, packet->command, packet->commandLen);
	
	return mac;
}

uint16_t CalcMAC2(uint8_t* packet, uint8_t startIndex, uint8_t endIndex, uint32_t NID)
{
	uint16_t mac = 0;
	uint8_t nidArray[4] = {NID>>24, NID>>16, NID>>8, NID};
	
	mac = AddToMAC(mac, nidArray, 4);
	mac = AddToMAC(mac, packet+startIndex, endIndex-startIndex);
	
	return mac;
}

uint16_t CalcChallengeResponse(uint16_t challenge, uint32_t challengerID, uint32_t responderID)
{
	uint16_t mac = 0;
	uint8_t challengeArray[2] = {challenge>>8, challenge};
	uint8_t cidArray[4] = {challengerID>>24, challengerID>>16, challengerID>>8, challengerID};
	uint8_t ridArray[4] = {responderID>>24, responderID>>16, responderID>>8, responderID};
	
	mac = AddToMAC(mac, challengeArray, 2);
	mac = AddToMAC(mac, cidArray, 4);
	mac = AddToMAC(mac, ridArray, 4);
	
	return mac;
}

uint16_t GenerateChallenge(void)
{
	return Rand16();
}

bool IsMacCorrect(CM2Packet* packet, uint32_t NID)
{
	if (!IS_PACKET_VALID(packet))
		return FALSE;
	
	//return (bool)(packet->mac == CalcMAC(packet, NID));
	if (packet->mac == CalcMAC(packet, NID))
		return TRUE;
	else
		return FALSE;
}