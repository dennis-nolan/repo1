/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RANDOM_HEADER
#define RANDOM_HEADER

#include "CommonTypes.h"

//supply a new seed for the random number generator
//Note: 0 is invalid
void SetSeed(uint64_t val);

//inject randomness into the RNG
void AddEntropy(uint64_t rnd);

//0 to 0xFFFF result
uint16_t GetRandom16(void);

//0 to 0xFFFFFFFF result
uint32_t GetRandom32(void);

//close recursive include ifdef
#endif
