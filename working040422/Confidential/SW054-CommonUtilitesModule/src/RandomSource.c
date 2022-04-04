
#include "RandomHeader.h"

#define EXP64 (uint64_t)0xD800000000000000

static uint64_t lfsr = 0x2F9606E00E84175A;

/*#define EXP1 (uint32_t)0xA3000000
#define EXP2 (uint32_t)0x80200003

//Any nonzero start state will work.
static uint32_t lfsr = 0x2F66875A;
static bool useExp1 = TRUE;*/


// Cycles through every value except 0
static void CalcNextRandom(void)
{
	// taps: 32,30,26,25
	/*uint32_t bit = ((lfsr >> 32) ^ (lfsr >> 30) ^ (lfsr >> 26) ^ (lfsr >> 25) ) & 1;
	lfsr = (lfsr >> 1) | (bit << 31);*/
	
	//uint32_t lsb = lfsr & 1;
	uint64_t lsb = lfsr & 1;
	lfsr >>= 1;
	if (lsb)
	{
		lfsr ^= EXP64;
		
		/*if (useExp1)
			lfsr ^= EXP1;//0xA3000000;
		else
			lfsr ^= EXP2;*/
	}
	
	//LFSR does not work if it is ever 0
	if (0 == lfsr)
		lfsr = 0x35BA07BE066A142;
}

//0 is invalid
void SetSeed(uint64_t val)
{
	if (val > 0)
		lfsr = val;
	
	//select exponent
	//useExp1 = ((val & 1) == 0);
}

//inject randomness
void AddEntropy(uint64_t rnd)
{
	//reselect exponent
	//useExp1 = ((lfsr & 1) == 0);
	
	//mix in to current value
	uint32_t temp = lfsr ^ rnd;
	
	//don't allow a value of 0
	if (temp > 0)
		lfsr = temp;
}

//adjusted to allow a full 0 to 0xFFFF result
uint16_t GetRandom16(void)
{
	CalcNextRandom();
	return (uint16_t)((lfsr-1) & 0xFFFF);
}

//adjusted to allow a full 0 to 0xFFFFFFFF result
uint32_t GetRandom32(void)
{
	/*uint32_t temp = lfsr;
	CalcNextRandom();
	
	//Use previous random value to decide whether to set bit 0 to 0
	//This gives us the full 0 to 0xFFFFFFFF range instead of starting at 1
	if (temp <= 0xFFFF)
		return (lfsr & 0xFFFFFFFE);
	else
		return lfsr;*/
	
	CalcNextRandom();
	return (uint32_t)((lfsr-1) & 0xFFFFFFFF);
}
