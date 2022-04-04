
// Define to prevent recursive inclusion
#ifndef SPIRIT_PRIVATE_UTILS_H
#define SPIRIT_PRIVATE_UTILS_H

#include "CM2CoreTypesHeader.h"
#include "SpiritModuleInterface.h"
#include "SPIRIT_Irq.h"
#include "SPIRIT_Types.h"

typedef enum {
	SPIRIT_STATE_UNINIT,
	SPIRIT_STATE_READY,
	SPIRIT_STATE_TX,
	SPIRIT_STATE_RX,
	SPIRIT_STATE_CWTX
} SpiritDriverState;

typedef struct
{
	uint8_t len;
	uint8_t dest;
	uint8_t* payload;
} SpiritBasicPacket;

typedef struct {
	CM2PortType portType;
	SpiritFreq frequency;
	float txPower;
	bool ready;
	SpiritDriverStatus status;
	SpiritDriverState state;
uint8_t holdoffCount;/////////
bool holdoffTimeout;//////////
	uint8_t holdoffTimer;
	SpiritStatus spiritStatus;
	SpiritIrqs irqStatus;
	bool interrupted;
	uint32_t lastUpdate;
	bool txTimeout;
	uint32_t txStartTick;
	uint16_t lastPacketMAC;
} SpiritDevice;

bool AssertSpiritState(SpiritDevice* device, SpiritState state, bool updateFirst, uint16_t timeout);
void UpdateSpiritIRQs(SpiritDevice* device);
void SpiritEnterRx_Private(SpiritDevice* device);
void SpiritTxError(SpiritDevice* device);

#endif
