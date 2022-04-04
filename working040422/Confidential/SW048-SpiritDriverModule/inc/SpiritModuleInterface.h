
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPIRITINTERFACE_HEADER
#define SPIRITINTERFACE_HEADER

#include "CommonTypes.h"
#include "CM2CoreTypesHeader.h"

#ifndef NUM_SPIRIT_RADIOS
#pragma message("NUM_SPIRIT_RADIOS is not defined. Defaulting to 1.")
#define NUM_SPIRIT_RADIOS 1
#endif

#ifndef SPIRIT_ERROR_BLANKING
#pragma message("SPIRIT_ERROR_BLANKING is not defined. Defaulting to 10 seconds.")
#define SPIRIT_ERROR_BLANKING 10000
#endif


typedef enum {
	SPIRIT_MODE_SHUTDOWN,
	SPIRIT_MODE_NORMAL,
	SPIRIT_MODE_RX_ONLY,
	SPIRIT_MODE_CW_TX,
	SPIRIT_MODE_RANDOM_TX,
	SPIRIT_MODE_RSSI_STREAM
} SpiritMode;

typedef enum {
#ifdef ALLOW_TEST_FREQUENCIES
	SPIRIT_TEST_FREQ_911_25,
	SPIRIT_TEST_FREQ_913_25,
	SPIRIT_TEST_FREQ_915_25,
	SPIRIT_TEST_FREQ_917_25,
	SPIRIT_TEST_FREQ_919_25,
#endif
	SPIRIT_FREQ_921_25,
	SPIRIT_FREQ_922_75,
	SPIRIT_FREQ_926_75,
	SPIRIT_FREQ_PLC_902,
	SPIRIT_FREQ_PLC_927_5,
	SPIRIT_FREQ_EU_868_3, //1%
	SPIRIT_FREQ_EU_868_95, //0.1%
        SPIRIT_FREQ_EU_868_3_NORMAL        
} SpiritFreq;

typedef struct {
	CM2PortType portType;
	SpiritFreq frequency;
	float txPower;
} SpiritRadioInit;

typedef struct {
	SpiritMode mode;
	SpiritFreq freq;
	TxStatus txStatus;
	uint8_t lastRSSI;
	uint8_t lastPacketRSSI;
	uint16_t generalErrors;
	uint16_t txErrors;
	uint16_t spiErrors;
	uint32_t lastErrorTick;
} SpiritDriverStatus;

// Turns off Spirit, delays, turns on Spirit, delays
void SpiritReset(CM2PortType portFlags);

// Accepts incoming bytes and buffers them for parsing
bool RxBytes(CM2PortType portFlags, uint8_t numBytes, uint8_t* data);

// When non-blocking TX is complete this is called with the status of the operation
//   If only using blocking transmit or if you don't care about the transmit result you can use the "weak" definition below.
void TxCallback(CM2PortType portFlags, uint16_t mac, TxStatus status);
//__weak void TxCallback(uint8_t portType, SpiritTxStatus status) {return;}

// Disables the LEDs (and other non-essentials) while transmitting
//   Devices with ample power available can instead rely on the "weak" definition below.
void TxBatterySaver(bool enable);
//__weak void TxBatterySaver(bool enable) {return;}

// close recursive include ifdef
#endif