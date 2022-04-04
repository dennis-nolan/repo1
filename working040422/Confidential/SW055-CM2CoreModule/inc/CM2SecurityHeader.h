/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
******************************* END VERSION INFO ******************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CM2SECURITY_HEADER
#define CM2SECURITY_HEADER

#include "CM2CoreTypesHeader.h"

//void CM2SecurityDeinit(void);
uint16_t CalcMAC(CM2Packet* packet, uint32_t NID);
uint16_t CalcMAC2(uint8_t* packet, uint8_t startIndex, uint8_t endIndex, uint32_t NID);
bool IsMacCorrect(CM2Packet* packet, uint32_t NID);
//void SeedRNG(uint8_t* randomBytes, uint8_t numBytes);
uint16_t LinearFeedbackShiftRegister(uint16_t seed, uint16_t exponents);
uint16_t Rand16(void);
uint16_t GenerateChallenge(void);
uint16_t CalcChallengeResponse(uint16_t challenge, uint32_t challengerID, uint32_t responderID);///////////////////////
void RegisterTickInterface(uint32_t (*GetTicks)(void));

// close recursive include ifdef
#endif