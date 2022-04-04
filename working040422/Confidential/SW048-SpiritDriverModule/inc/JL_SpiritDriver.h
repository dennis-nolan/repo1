/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	10/05/2017: Refactoring - simplifying/cleaning up driver operation.
	08/10/2016: Added accessor for RSSI info
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	06/21/2016: Initial creation.
******************************* END VERSION INFO ******************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPIRITDRIVER_JL_HEADER
#define SPIRITDRIVER_JL_HEADER

#include "CommonTypes.h"
#include "SPIRIT_Types.h"
#include "CM2CoreTypesHeader.h"
#include "SpiritModuleInterface.h"
#include "spiritPrivateUtils.h"

#define RSSI_TO_DB_FLOAT(r) (-120.0+((float)((r)-20))/2)
#define RSSI_TO_DB_INTEGER(r) (-120.0+((short)(r)-20)/2)

#define SPIRIT_MAX_TX_POWER 11.6 //dB
#define SPIRIT_MIN_TX_POWER -34 //dB

//Initialize the Spirit driver
//Pass in how many radios to use and a populated init structure for each
void SpiritInit(uint8_t numRadios, SpiritRadioInit* radioInitList, uint8_t devType, uint32_t NID);

//Background tasks to be run in the main loop context
void RunSpiritBackgroundTasks(void);
//void RunFastSpiritTasks(void);

//Transmit a CM2 packet via Spirit
TxStatus SpiritTx(CM2PortType portType, CM2Packet* packet, bool blocking);

//Interrupt handler. Pass in the SPI port associated with this interrupt.
void HandleSpiritInterrupt(CM2PortType portType);

//Returns a SpiritDriverStatus struct with information on the state of the driver for this port
SpiritDriverStatus GetSpiritDriverStatus(CM2PortType portType);

//Changes driver modes - primarily for testing and certification purposes.
void SetSpiritMode(CM2PortType portType, SpiritMode mode);

//Changes the frequency to the one specified
void SetSpiritFrequency(CM2PortType portType, SpiritFreq freq);

//Changes the transmit power to the one specified (-34dB to +11.6dB)
void SetSpiritTxPower(CM2PortType portType, float dB);

//Get the latest RSSI for the specified port
uint8_t GetRSSI(CM2PortType portType, bool duringPacket);

//Handle a SPI error related to Spirit
void SpiritSpiError(CM2PortType portType);

//Change in-radio filtering setting
void SetFiltering(CM2PortType portFlags, uint8_t deviceType, bool enable);

//Enable clients to force re-init
void SpiritReinit(CM2PortType type);

SpiritDevice* GetDeviceFromPortType(CM2PortType portFlags);
// close recursive include ifdef
#endif