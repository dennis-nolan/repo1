/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/10/2016: Added accessor for RSSI info
	08/03/2016: Converting to static memory.
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	06/21/2016: Initial creation.
******************************* END VERSION INFO ******************************/

#include "JL_SpiritDriver.h"
#include "CM2CoreTypesHeader.h"
#include "MCU_Interface.h"
#include "SPIRIT_Irq.h"
#include "SPIRIT_Commands.h"
#include "CM2RoutingHeader.h"
#include "TimersHeader.h"
#include "RawCircularBufferHeader.h"
#include "SpiritModuleInterface.h"
#include "CM2PortsHeader.h"
#include "SpiritErrataHeader.h"
#include "SpiritPrivateUtils.h"
#include "SpiritTx.h"

#define SPIRIT_UPDATE_TIMEOUT 65 //ms
#define SPIRIT_UPDATE_TIMEOUT_TX 0 //ms
#define ERROR_LIMIT 10
#define TX_ERROR_LIMIT 20
#define SPI_ERROR_LIMIT 10

//limits of the 900MHz ISM band
#define MAX_FREQ 928 //MHz
#define MIN_FREQ 902 //MHz

#define RX_FIFO_AF_THRESH 8 //bytes

//CSMA settings
#define RSSI_TH_FROM_DB(d)		((uint8_t)((d+130)*2)) //0.5dB steps, 20 correspond to -120 dBm
#define RX_RSSI_THRESH			-112 //dB, -112 default

#define XTAL_FREQ 50000000

//static void SpiritEnterRx_Private(SpiritDevice* device);
static void SpiritUpdateStatus(SpiritDevice* device);
static void PollSpirit(SpiritDevice* device);
static void SpiritInitSingleOld(SpiritDevice* device);

static SpiritDevice deviceList[NUM_SPIRIT_RADIOS];
static uint8_t fifoBuffer[96];

static uint8_t tempRegValue[4];

SpiritDevice* GetDeviceFromPortType(CM2PortType portFlags)
{
	for (uint8_t i=0; i<NUM_SPIRIT_RADIOS; i++)
	{
		if (deviceList[i].portType == portFlags || PORT_ALL == portFlags)
			return &(deviceList[i]);
	}
	return (SpiritDevice*)0;
}

static void SpiritError(SpiritDevice* device)
{
	//TODO-log error

	device->status.generalErrors++;

	if (device->status.generalErrors >= ERROR_LIMIT)
	{
		//reset spirit by doing a re-init
		SpiritInitSingleOld(device);
	}

	device->status.lastErrorTick = GetMsSinceStart();
}

void SpiritTxError(SpiritDevice* device)
{
	//TODO-log error

	device->status.txErrors++;

	if (device->status.txErrors >= TX_ERROR_LIMIT)
	{
		//reset spirit by doing a re-init
		SpiritInitSingleOld(device);
	}

	device->status.lastErrorTick = GetMsSinceStart();
}

void SpiritSpiError(CM2PortType portType)
{
	SpiritDevice* device = GetDeviceFromPortType(portType);
	if (device == 0)
		return;

	device->status.spiErrors++;

	if (device->status.spiErrors >= SPI_ERROR_LIMIT)
	{
		//reset spirit by doing a re-init
		SpiritInitSingleOld(device);
	}

	device->status.lastErrorTick = GetMsSinceStart();
}

void SpiritReinit(CM2PortType type)
{
	SpiritDevice* device = GetDeviceFromPortType(type);
	if (device == 0)
		return;

	SpiritInitSingleOld(device);
}

void UpdateSpiritIRQs(SpiritDevice* device)
{
	//uint8_t clearRegValue[4] = {0,0,0,0};
	uint8_t* pIrqPointer = (uint8_t*)(&device->irqStatus);

	//Reads IRQ_STATUS registers (and gets MC_STATE for free)
	device->spiritStatus = SpiReadRegisters(device->portType, IRQ_STATUS3_BASE, 4, tempRegValue);
	//device->spiritStatus = SpiWriteRegisters(device->portType, IRQ_STATUS3_BASE, 4, clearRegValue);

	//Build the IRQ Status word
	for(uint8_t i=0; i<4; i++)
	{
		//Note: this is OR-ing in the new IRQs rather than overwriting!
		*pIrqPointer |= tempRegValue[3-i];
		pIrqPointer++;
	}

	device->interrupted = FALSE;
}

/*static*/ void ClearSpiritIRQs(SpiritDevice* device)
{
	uint8_t* pIrqPointer = (uint8_t*)(&device->irqStatus);
	for(uint8_t i=0; i<4; i++)
	{
		*pIrqPointer = 0;
		pIrqPointer++;
	}
}

static bool WaitForSpiritState(SpiritDevice* device, SpiritState state, uint16_t timeout)
{
	//bool timeoutExpired = FALSE;
	//RegisterTimeout(&timeoutExpired, timeout);
	uint32_t start = GetMsSinceStart();

	do {
		if (GetMsElapsed(start) >= timeout)
			return FALSE;

		//SpiritUpdateStatus(device);

		device->spiritStatus = SpiReadRegisters(device->portType, RSSI_LEVEL_BASE, 1, &(device->status.lastRSSI));
	} while (device->spiritStatus.MC_STATE != state);

	//CancelTimeout(&timeoutExpired);

	return TRUE;
}

static bool GoToReady(SpiritDevice* device, uint16_t timeout)
{
	//valid only from STANDBY or SLEEP or LOCK
	//if (!(device->spiritStatus.MC_STATE == MC_STATE_STANDBY || device->spiritStatus.MC_STATE == MC_STATE_SLEEP || device->spiritStatus.MC_STATE == MC_STATE_LOCK))
	if (device->spiritStatus.MC_STATE == MC_STATE_TX || device->spiritStatus.MC_STATE == MC_STATE_RX)
	{
		device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);
	}

	/*if (WaitForSpiritState(device,MC_STATE_READY,2))
		return TRUE;
	else if (timeout > 2)
		timeout -= 2;*/

	device->spiritStatus = SpiCommandStrobe(device->portType, CMD_READY);

	volatile uint8_t dummy;
	if (FALSE == WaitForSpiritState(device,MC_STATE_READY,timeout))
	{
		dummy++;
		return FALSE;
	}
	return TRUE;
}

static SpiritState startingState;
static SpiritState prevCommandedState;

bool AssertSpiritState(SpiritDevice* device, SpiritState state, bool updateFirst, uint16_t timeout)
{
	if (updateFirst)
		device->spiritStatus = SpiReadRegisters(device->portType, RSSI_LEVEL_BASE, 1, &(device->status.lastRSSI));
		//SpiritUpdateStatus(device);

	if (device->spiritStatus.MC_STATE == state)
		return TRUE;

startingState = device->spiritStatus.MC_STATE;

	bool toReturn = TRUE;
	switch(state)
	{
		case MC_STATE_STANDBY:
			//valid only from READY
			if (device->spiritStatus.MC_STATE != MC_STATE_READY)
			{
				if (!GoToReady(device, 4))
				{
					toReturn = FALSE;
					break;
				}
			}

			device->spiritStatus = SpiCommandStrobe(device->portType, CMD_STANDBY);
			toReturn = WaitForSpiritState(device,MC_STATE_STANDBY,timeout/2);
			break;
		case MC_STATE_READY:
			toReturn = GoToReady(device, timeout);
			break;
		case MC_STATE_RX:
			//valid only from READY or LOCK RX
			if (MC_STATE_READY != device->spiritStatus.MC_STATE)// && !(MC_STATE_LOCK == device->spiritStatus.MC_STATE && MC_STATE_LOCK_RX == prevCommandedState))//(device->spiritStatus.MC_STATE != MC_STATE_READY)
			{
				if (!GoToReady(device, 5))
				{
					toReturn = FALSE;
					break;
				}
			}

			device->spiritStatus = SpiCommandStrobe(device->portType, COMMAND_LOCKRX);
			toReturn = WaitForSpiritState(device,MC_STATE_LOCK,5);

			device->spiritStatus = SpiCommandStrobe(device->portType, CMD_RX);
			toReturn = WaitForSpiritState(device,MC_STATE_RX,timeout-5);
			break;
		case MC_STATE_TX:
			//valid only from READY or LOCK TX
			if (MC_STATE_READY != device->spiritStatus.MC_STATE)// && MC_STATE_LOCK != device->spiritStatus.MC_STATE)
			{
				if (!GoToReady(device, 5))
				{
					toReturn = FALSE;
					break;
				}
			}

			device->spiritStatus = SpiCommandStrobe(device->portType, COMMAND_LOCKTX);
			toReturn = WaitForSpiritState(device,MC_STATE_LOCK,5);

			if (toReturn == FALSE)
				device->status.txErrors++;

			device->spiritStatus = SpiCommandStrobe(device->portType, CMD_TX);
			toReturn = WaitForSpiritState(device,MC_STATE_TX,timeout-10);
			break;
		/*case MC_STATE_LOCK_RX:
		case MC_STATE_LOCK_TX:
			//valid only from READY
			if (MC_STATE_READY != device->spiritStatus.MC_STATE)// && MC_STATE_LOCK != device->spiritStatus.MC_STATE)
			{
				if (!GoToReady(device, 4))
				{
					toReturn = FALSE;
					break;
				}
			}

			if (MC_STATE_LOCK_RX == state)
				device->spiritStatus = SpiCommandStrobe(device->portType, COMMAND_LOCKRX);
			else
				device->spiritStatus = SpiCommandStrobe(device->portType, COMMAND_LOCKTX);

			toReturn = WaitForSpiritState(device, MC_STATE_LOCK, timeout-4);
			break;*/
		case MC_STATE_SLEEP:
			//valid only from READY
		default:
			toReturn = FALSE;
	}

	prevCommandedState = state;

	volatile uint8_t dummy;
	if (FALSE == toReturn)
		dummy++;
	return toReturn;
}
/**
 * @brief  Fills a pointer to a structure of SpiritIrqs type reading the IRQ_STATUS registers.
 * @param  pxIrqStatus pointer to a variable of type @ref SpiritIrqs, through which the
 *         user can read the status of all the IRQs. All the bitfields equals to one correspond
 *         to the raised interrupts. This parameter is a pointer to a SpiritIrqs.
 *         For example suppose that the XO settling timeout is raised as well as the Sync word
 *         detection.
 * myIrqStatus.IRQ_XO_COUNT_EXPIRED and myIrqStatus.IRQ_VALID_SYNC are equals to 1
 * while all the other bitfields are equals to zero.
 */
static uint32_t syncCount = 0;
static void SpiritUpdateStatus(SpiritDevice* device)
{
	UpdateSpiritIRQs(device);

	//Get RSSI
	if (device->spiritStatus.MC_STATE == MC_STATE_RX && device->irqStatus.IRQ_VALID_SYNC == S_SET)
	{
		//RSSI is not populated immediately, wait to make sure it is correct
		//HAL_Delay(2);
		for (volatile uint32_t i=0; i<1000; i++);

		//read
		device->spiritStatus = SpiReadRegisters(device->portType, RSSI_LEVEL_BASE, 1, &(device->status.lastRSSI));

		//copy
		device->status.lastPacketRSSI = device->status.lastRSSI;
		syncCount++;
	}
	else
    {
		device->spiritStatus = SpiReadRegisters(device->portType, RSSI_LEVEL_BASE, 1, &(device->status.lastRSSI));
    }
}

static SpiritIrqs mask;
static void SetNormalRunningInterrupts(SpiritDevice* device)
{
	//set to interrupt pin
	tempRegValue[0] = CONF_GPIO_OUT_nIRQ|CONF_GPIO_MODE_DIG_OUTL;
	device->spiritStatus = SpiWriteRegisters(device->portType, GPIO0_CONF_BASE, 1, tempRegValue);

	//config rx fifo almost full
	/*uint8_t temp = RX_FIFO_AF_THRESH;
	device->spiritStatus = SpiWriteRegisters(FIFO_CONFIG3_RXAFTHR_BASE, 1, &temp);*/


	mask.IRQ_RX_DATA_READY = S_SET;
	mask.IRQ_RX_FIFO_ALMOST_FULL = S_SET;
	mask.IRQ_RX_FIFO_ERROR = S_SET;
	mask.IRQ_TX_DATA_SENT = S_SET;
	mask.IRQ_RX_DATA_DISC = S_SET;
	//mask.IRQ_TX_FIFO_ALMOST_EMPTY = S_SET;
	//mask.IRQ_TX_FIFO_ERROR = S_SET;
/*mask.IRQ_MAX_BO_CCA_REACH = S_SET;
mask.IRQ_VALID_PREAMBLE = S_SET;
mask.IRQ_VALID_SYNC = S_SET;
mask.IRQ_RX_DATA_DISC = S_SET;
mask.IRQ_RSSI_ABOVE_TH = S_SET;*/
	device->spiritStatus = SpiWriteRegisters(device->portType, IRQ_MASK0_BASE, 4, (uint8_t*)&mask);

//QI_BASE QI_PQI_MASK
/*uint8_t reg = 0x29;
device->spiritStatus = SpiWriteRegisters(device->portType, QI_BASE, 1, &reg);
//reg = 0x0A;
//device->spiritStatus = SpiWriteRegisters(device->portType, PROTOCOL0_BASE, 1, &reg);

reg = PROTOCOL2_VCO_CALIBRATION_MASK | PROTOCOL2_RCO_CALIBRATION_MASK | PROTOCOL2_PQI_TIMEOUT_MASK | PROTOCOL2_SQI_TIMEOUT_MASK;
device->spiritStatus = SpiWriteRegisters(device->portType, PROTOCOL2_BASE, 1, &reg);

reg = PROTOCOL0_PERS_RX_MASK | PROTOCOL0_NACK_TX_MASK;
device->spiritStatus = SpiWriteRegisters(device->portType, PROTOCOL0_BASE, 1, &reg);*/;
//device->spiritStatus = SpiReadRegisters(device->portType, IRQ_MASK0_BASE, 4, (uint8_t*)&mask);

	/*SpiritIrqs mask;
	mask.IRQ_RX_DATA_READY = S_SET;
	device->spiritStatus = SpiWriteRegisters(IRQ_MASK0_BASE, 4, (uint8_t*)&mask);*/

	//note: mc_state[1] has tx full and rx empty flags
	//LINEAR_FIFO_STATUS[1] tx count, top bit reserved
	//LINEAR_FIFO_STATUS[0] rx count, top bit reserved
	//0xFF fifo addr

/*GPIO0_CONF_BASE
CONF_GPIO_OUT_TX_State
CONF_GPIO_IN_TX_Command*/
//uint8_t temp = CONF_GPIO_OUT_TX_State|CONF_GPIO_MODE_DIG_OUTL;
//uint8_t temp = CONF_GPIO_OUT_Valid_Preamble|CONF_GPIO_MODE_DIG_OUTL;
//uint8_t temp = CONF_GPIO_IN_TX_Command|CONF_GPIO_MODE_DIG_IN;
//device->spiritStatus = SpiWriteRegisters(device->portType, GPIO0_CONF_BASE, 1, (uint8_t*)&temp);
}

static void SetCSMA(SpiritDevice* device, bool enable)
{
	//uint8_t tempRegValue = 0xFF;

	/*if (enableSpiritCSMA && enable)
	{
		//RSSI_FLT
		tempRegValue[0] = (RSSI_FLT << 4) | 0x03; //0xE3;
		device->spiritStatus = SpiWriteRegisters(device->portType, RSSI_FLT_BASE, 1, tempRegValue);

		//tempRegValue = ; //seed LSB
		//device->spiritStatus = SpiWriteRegisters(device->portType, CSMA_CONFIG2_BASE, 1, tempRegValue);
		//tempRegValue = ; //seed MSB
		//device->spiritStatus = SpiWriteRegisters(device->portType, CSMA_CONFIG3_BASE, 1, tempRegValue);

		tempRegValue[0] = (BU_PRESCALER<<2) | CSMA_CCA_PERIOD; //BU_PRESCALER and CCA_PERIOD
		device->spiritStatus = SpiWriteRegisters(device->portType, CSMA_CONFIG1_BASE, 1, tempRegValue);

		//backoff
		tempRegValue[0] = (CCA_LENGTH<<4) | NBACKOFF_MAX; //CCA_LENGTH and NBACKOFF_MAX
		device->spiritStatus = SpiWriteRegisters(device->portType, CSMA_CONFIG0_BASE, 1, tempRegValue);

		//enable persistent CSMA
		if (CSMA_PERS)
		{
			device->spiritStatus = SpiReadRegisters(device->portType, PROTOCOL1_BASE, 1, tempRegValue);
			tempRegValue[0] |= PROTOCOL1_CSMA_PERS_ON_MASK;
			device->spiritStatus = SpiWriteRegisters(device->portType, PROTOCOL1_BASE, 1, tempRegValue);
		}

		//CMSA on
		device->spiritStatus = SpiReadRegisters(device->portType, PROTOCOL1_BASE, 1, tempRegValue);
		tempRegValue[0] |= PROTOCOL1_CSMA_ON_MASK;
		device->spiritStatus = SpiWriteRegisters(device->portType, PROTOCOL1_BASE, 1, tempRegValue);

		//RSSI thresh (0.5dB steps, 20 correspond to -120 dBm)
		tempRegValue[0] = RSSI_TH_FROM_DB(CSMA_RSSI_THRESH);
		device->spiritStatus = SpiWriteRegisters(device->portType, RSSI_TH_BASE, 1, tempRegValue);
	}
	else*/
	{
		//RSSI thresh (0.5dB steps, 20 correspond to -120 dBm)
		tempRegValue[0] = RSSI_TH_FROM_DB(RX_RSSI_THRESH);
		device->spiritStatus = SpiWriteRegisters(device->portType, RSSI_TH_BASE, 1, tempRegValue);

		//CMSA off
		device->spiritStatus = SpiReadRegisters(device->portType, PROTOCOL1_BASE, 1, tempRegValue);
		tempRegValue[0] &= ~PROTOCOL1_CSMA_ON_MASK;
		device->spiritStatus = SpiWriteRegisters(device->portType, PROTOCOL1_BASE, 1, tempRegValue);
	}
}

void SpiritEnterRx_Private(SpiritDevice* device)
{
	if (device->status.mode == SPIRIT_MODE_CW_TX || device->status.mode == SPIRIT_MODE_RANDOM_TX)
		return;

	//SpiritUpdateStatus(device);

	//if (device->state != SPIRIT_STATE_RX || device->spiritStatus.MC_STATE != MC_STATE_RX)
	/*{
		if (!AssertSpiritState(device, MC_STATE_READY, TRUE, 5))
			SpiritError(device);
	}

	SetCSMA(device, FALSE);
	device->status.txStatus = TX_IDLE;*/

	device->status.txStatus = TX_IDLE;
	//SetCSMA(device, FALSE);

/*if (!AssertSpiritState(device, MC_STATE_READY, TRUE, 5))
	SpiritError(device);*/

UpdateSpiritIRQs(device);
ClearSpiritIRQs(device);

tempRegValue[0] = 0x02;
device->spiritStatus = SpiWriteRegisters(device->portType, QI_BASE, 1, tempRegValue);
tempRegValue[0] = 0x46;
device->spiritStatus = SpiWriteRegisters(device->portType, PROTOCOL2_BASE, 1, tempRegValue);

	// wait for RX
	if (!AssertSpiritState(device, MC_STATE_RX, TRUE, 15))
		SpiritError(device);

	device->state = SPIRIT_STATE_RX;
}
#if 0
	/*SpiritUpdateStatus(device);

	if (device->state == SPIRIT_STATE_RX && device->spiritStatus.MC_STATE == MC_STATE_RX)
	{
		//verify everything is good to go
		return;
	}

	if (device->spiritStatus.MC_STATE != MC_STATE_READY)
	{
		device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);//, device->spiPort);

		// wait for ready
		WaitForSpiritState(device,MC_STATE_READY,5);
	}*/

	SetCSMA(device, FALSE);

	if (device->state == SPIRIT_STATE_RX && device->spiritStatus.MC_STATE == MC_STATE_RX)
		return;

	if (!AssertSpiritState(device, MC_STATE_READY, TRUE, 5))
		SpiritError(device);

	//SetCSMA(device, FALSE);

	//device->spiritStatus = SpiCommandStrobe(device->portType, CMD_RX);

	device->status.txStatus = TX_IDLE;

	/*device->spiritStatus = SpiCommandStrobe(device->portType, COMMAND_LOCKRX);
	WaitForSpiritState(device, MC_STATE_LOCK, 10);*/
	/*if (!AssertSpiritState(device, MC_STATE_LOCK_RX, FALSE, 5))
		SpiritError(device);*/

	// wait for RX
	if (!AssertSpiritState(device, MC_STATE_RX, FALSE, 15))
		SpiritError(device);
	device->state = SPIRIT_STATE_RX;
}
#endif

/* This is the function that initializes the SPIRIT with the configuration
that the user has exported using the GUI */
static void SpiritBaseConfigurationPLC(CM2PortType portFlags)
{
	uint8_t tmp[7];

	/* Be sure that the registers config is default */
	SpiCommandStrobe(portFlags, COMMAND_SRES);

	/* Extra current in after power on fix.
	In some samples, when a supply voltage below 2.6 V is applied to SPIRIT1 from a no power condition,
	an extra current is added to the typical current consumption.
	With this sequence, the extra current is erased.*/
	tmp[0]=0xCA;SpiWriteRegisters(portFlags, 0xB2, 1, tmp);
	tmp[0]=0x04;SpiWriteRegisters(portFlags, 0xA8, 1, tmp);
	SpiReadRegisters(portFlags, 0xA8, 1, tmp);
	tmp[0]=0x00;SpiWriteRegisters(portFlags, 0xA8, 1, tmp);

	tmp[0] = 0x36; /* reg. IF_OFFSET_ANA (0x07) */
	tmp[1] = 0x66; /* reg. SYNT3 (0x08) */
	tmp[2] = 0xC4; /* reg. SYNT2 (0x09) */
	tmp[3] = 0xCC; /* reg. SYNT1 (0x0A) */
	tmp[4] = 0xC9; /* reg. SYNT0 (0x0B) */
	tmp[5] = 0x01; /* reg. CH_SPACE (0x0C) */
	tmp[6] = 0xAC; /* reg. IF_OFFSET_DIG (0x0D) */
	SpiWriteRegisters(portFlags, 0x07, 7, tmp);
	tmp[0] = 0x01; /* reg. PA_POWER[8] (0x10) */
	SpiWriteRegisters(portFlags, 0x10, 1, tmp);
	tmp[0] = 0x48; /* reg. MOD1 (0x1A) */
	tmp[1] = 0x1E; /* reg. MOD0 (0x1B) */
	tmp[2] = 0x77; /* reg. FDEV0 (0x1C) */
	tmp[3] = 0x13; /* reg. CHFLT (0x1D) */
	tmp[4] = 0xC8; /* reg. AFC2 (0x1E) */
	SpiWriteRegisters(portFlags, 0x1A, 5, tmp);
	tmp[0] = 0x62; /* reg. AGCCTRL1 (0x25) */
	SpiWriteRegisters(portFlags, 0x25, 1, tmp);
	tmp[0] = 0x15; /* reg. ANT_SELECT_CONF (0x27) */
	SpiWriteRegisters(portFlags, 0x27, 1, tmp);
	tmp[0] = 0x0B; /* reg. PCKTCTRL2 (0x32) */
	tmp[1] = 0x51; /* reg. PCKTCTRL1 (0x33) */
	SpiWriteRegisters(portFlags, 0x32, 2, tmp);
	tmp[0] = 0x00; /* reg. SYNC4 (0x36) */
	tmp[1] = 0x00; /* reg. SYNC3 (0x37) */
	tmp[2] = 0xC2; /* reg. SYNC2 (0x38) */
	tmp[3] = 0xD5; /* reg. SYNC1 (0x39) */
	SpiWriteRegisters(portFlags, 0x36, 4, tmp);
	tmp[0] = 0x41; /* reg. PCKT_FLT_OPTIONS (0x4F) */
	tmp[1] = 0x40; /* reg. PROTOCOL[2] (0x50) */
	tmp[2] = 0x01; /* reg. PROTOCOL[1] (0x51) */
	SpiWriteRegisters(portFlags, 0x4F, 3, tmp);
	tmp[0] = 0x00; /* reg. RCO_VCO_CALIBR_IN[1] (0x6E) */
	tmp[1] = 0x00; /* reg. RCO_VCO_CALIBR_IN[0] (0x6F) */
	SpiWriteRegisters(portFlags, 0x6E, 2, tmp);
	tmp[0] = 0xA0; /* reg. SYNTH_CONFIG[0] (0x9F) */
	SpiWriteRegisters(portFlags, 0x9F, 1, tmp);
	tmp[0] = 0x25; /* reg. VCO_CONFIG (0xA1) */
	SpiWriteRegisters(portFlags, 0xA1, 1, tmp);
	tmp[0] = 0x35; /* reg. DEM_CONFIG (0xA3) */
	SpiWriteRegisters(portFlags, 0xA3, 1, tmp);

	/* VCO unwanted calibration workaround.
	With this sequence, the PA is on after the eventual VCO calibration expires.*/
	tmp[0]=0x22;SpiWriteRegisters(portFlags, 0xBC, 1, tmp);
}

/* This is the function that initializes the SPIRIT with the configuration
that the user has exported using the GUI */
static void SpiritBaseConfigurationOTA(CM2PortType portFlags, uint8_t deviceType)
{
	uint8_t tmp[7];

	/* Be sure that the registers config is default */
	SpiCommandStrobe(portFlags, COMMAND_SRES);

	/* Extra current in after power on fix.
	In some samples, when a supply voltage below 2.6 V is applied to SPIRIT1 from a no power condition,
	an extra current is added to the typical current consumption.
	With this sequence, the extra current is erased.*/

	tmp[0]=0xCA;SpiWriteRegisters(portFlags, 0xB2, 1, tmp);
	tmp[0]=0x04;SpiWriteRegisters(portFlags, 0xA8, 1, tmp);
	SpiReadRegisters(portFlags, 0xA8, 1, tmp);
	tmp[0]=0x00;SpiWriteRegisters(portFlags, 0xA8, 1, tmp);

	tmp[0] = 0x02; /* reg. GPIO0_CONF (0x05) */
	SpiWriteRegisters(portFlags, 0x05, 1, tmp);
	tmp[0] = 0x36; /* reg. IF_OFFSET_ANA (0x07) */
	tmp[1] = 0xA6; /* reg. SYNT3 (0x08) */
	tmp[2] = 0xE4; /* reg. SYNT2 (0x09) */
	tmp[3] = 0xF5; /* reg. SYNT1 (0x0A) */
	tmp[4] = 0xC1; /* reg. SYNT0 (0x0B) */
	tmp[5] = 0x0D; /* reg. CH_SPACE (0x0C) */
	tmp[6] = 0xAC; /* reg. IF_OFFSET_DIG (0x0D) */
	SpiWriteRegisters(portFlags, 0x07, 7, tmp);
	tmp[0] = 0x01; /* reg. PA_POWER[8] (0x10) */
	SpiWriteRegisters(portFlags, 0x10, 1, tmp);
	tmp[0] = 0x48; /* reg. MOD1 (0x1A) */
	tmp[1] = 0x0E; /* reg. MOD0 (0x1B) */
	tmp[2] = 0x82; /* reg. FDEV0 (0x1C) */
	tmp[3] = 0x10; /* reg. CHFLT (0x1D) */
	tmp[4] = 0xC8; /* reg. AFC2 (0x1E) */
	SpiWriteRegisters(portFlags, 0x1A, 5, tmp);
	tmp[0] = 0x14; /* reg. RSSI_TH (0x22) */
	SpiWriteRegisters(portFlags, 0x22, 1, tmp);
	tmp[0] = 0x62; /* reg. AGCCTRL1 (0x25) */
	SpiWriteRegisters(portFlags, 0x25, 1, tmp);
	tmp[0] = 0x15; /* reg. ANT_SELECT_CONF (0x27) */
	SpiWriteRegisters(portFlags, 0x27, 1, tmp);
	tmp[0] = 0x0C; /* reg. PCKTCTRL4 (0x30) */
	SpiWriteRegisters(portFlags, 0x30, 1, tmp);
	tmp[0] = 0x3F; /* reg. PCKTCTRL2 (0x32) */
	SpiWriteRegisters(portFlags, 0x32, 1, tmp);
	tmp[0] = 0x15; /* reg. PCKTLEN0 (0x35) */
	tmp[1] = 0xA8; /* reg. SYNC4 (0x36) */
	tmp[2] = 0x35; /* reg. SYNC3 (0x37) */
	tmp[3] = 0x26; /* reg. SYNC2 (0x38) */
	tmp[4] = 0x1A; /* reg. SYNC1 (0x39) */
	SpiWriteRegisters(portFlags, 0x35, 5, tmp);
	tmp[0] = 0xFF; /* reg. PCKT_FLT_GOALS[2] (0x4C) */
	tmp[1] = deviceType; /* reg. PCKT_FLT_GOALS[1] (0x4D) */
	SpiWriteRegisters(portFlags, 0x4C, 2, tmp);
	tmp[0] = 0x6E; /* reg. PCKT_FLT_OPTIONS (0x4F) */
	tmp[1] = 0x46; /* reg. PROTOCOL[2] (0x50) */
	tmp[2] = 0x01; /* reg. PROTOCOL[1] (0x51) */
	SpiWriteRegisters(portFlags, 0x4F, 3, tmp);
	tmp[0] = 0x00; /* reg. RCO_VCO_CALIBR_IN[1] (0x6E) */
	tmp[1] = 0x00; /* reg. RCO_VCO_CALIBR_IN[0] (0x6F) */
	SpiWriteRegisters(portFlags, 0x6E, 2, tmp);
	tmp[0] = 0x08; /* reg. IRQ_MASK[1] (0x92) */
	tmp[1] = 0x07; /* reg. IRQ_MASK[0] (0x93) */
	SpiWriteRegisters(portFlags, 0x92, 2, tmp);
	tmp[0] = 0xA0; /* reg. SYNTH_CONFIG[0] (0x9F) */
	SpiWriteRegisters(portFlags, 0x9F, 1, tmp);
	tmp[0] = 0x25; /* reg. VCO_CONFIG (0xA1) */
	SpiWriteRegisters(portFlags, 0xA1, 1, tmp);
	tmp[0] = 0x35; /* reg. DEM_CONFIG (0xA3) */
	SpiWriteRegisters(portFlags, 0xA3, 1, tmp);
	tmp[0] = 0x90; /* reg. PM_CONFIG[1] (0xA5) */
	SpiWriteRegisters(portFlags, 0xA5, 1, tmp);

	/* VCO unwanted calibration workaround.
	With this sequence, the PA is on after the eventual VCO calibration expires.*/
	tmp[0]=0x22;SpiWriteRegisters(portFlags, 0xBC, 1, tmp);
}

/* This is a VCO calibration routine used to recalibrate the VCO of SPIRIT1 in a safe way.
 IMPORTANT: It must be called from READY state. */
static void SpiritVcoCalibration(CM2PortType portFlags)
{
	uint8_t tmp[4];
	uint8_t cal_words[2];
	uint8_t state;

	SpiReadRegisters(portFlags, 0x9E, 1, tmp);
	tmp[0] |= 0x80;
	SpiWriteRegisters(portFlags, 0x9E, 1, tmp); /* REFDIV bit set (to be restored) */

	/* As a consequence we need to double the SYNT word to generate the target frequency */
	tmp[0] = 0x6D;
	tmp[1] = 0x89;
	tmp[2] = 0x99;
	tmp[3] = 0x91;
	SpiWriteRegisters(portFlags, 0x08, 4, tmp);

	tmp[0] = 0x25; SpiWriteRegisters(portFlags, 0xA1,1,tmp); /* increase VCO current (restore to 0x11) */

	SpiReadRegisters(portFlags, 0x50,1,tmp);
	tmp[0] |= 0x02;
	SpiWriteRegisters(portFlags, 0x50,1,tmp); /* enable VCO calibration (to be restored) */

	SpiCommandStrobe(portFlags, COMMAND_LOCKTX);
	do{
		SpiReadRegisters(portFlags, 0xC1, 1, &state);
	}while((state&0xFE) != 0x1E); /* wait until LOCK (MC_STATE = 0x0F <<1) */
	SpiReadRegisters(portFlags, 0xE5, 1, &cal_words[0]); /* calib out word for TX */

	SpiCommandStrobe(portFlags, COMMAND_READY);
	do{
		SpiReadRegisters(portFlags, 0xC1, 1, &state);
	}while((state&0xFE) != 0x06); /* wait until READY (MC_STATE = 0x03 <<1) */

	SpiCommandStrobe(portFlags, COMMAND_LOCKRX);
	do{
		SpiReadRegisters(portFlags, 0xC1, 1, &state);
	}while((state&0xFE) != 0x1E); /* wait until LOCK (MC_STATE = 0x0F <<1) */
	SpiReadRegisters(portFlags, 0xE5, 1, &cal_words[1]); /* calib out word for RX */

	SpiCommandStrobe(portFlags, COMMAND_READY);
	do{
		SpiReadRegisters(portFlags, 0xC1, 1, &state);
	}while((state&0xFE) != 0x06); /* wait until READY (MC_STATE = 0x03 <<1) */

	SpiReadRegisters(portFlags, 0x50,1,tmp);
	tmp[0] &= 0xFD;
	SpiWriteRegisters(portFlags, 0x50,1,tmp); /* VCO calib restored to 0 */

	SpiReadRegisters(portFlags, 0x9E, 1, tmp);
	tmp[0] &= 0x7F;
	SpiWriteRegisters(portFlags, 0x9E, 1, tmp); /* REFDIV bit reset */

	tmp[0] = 0x66;
	tmp[1] = 0xC4;
	tmp[2] = 0xCC;
	tmp[3] = 0xC9;
	SpiWriteRegisters(portFlags, 0x08, 4, tmp); /* SYNTH WORD restored */

	SpiWriteRegisters(portFlags, 0x6E,2,cal_words); /* write both calibration words */
}

static void SpiritInitSingle(SpiritDevice* device, SpiritRadioInit* radioInit, uint8_t deviceType, uint32_t NID)
{
	//reset structure members
	device->ready = FALSE;
	device->portType = radioInit->portType;
	device->status.mode = SPIRIT_MODE_SHUTDOWN;
	device->status.generalErrors = 0;
	device->status.txErrors = 0;
	device->status.spiErrors = 0;
	device->status.lastPacketRSSI = 0xFF;
	device->status.lastRSSI = 0xFF;

	//reset the hardware
	SpiritReset(device->portType);

	//configure registers
	if (PORT_SPIRIT_PLC == radioInit->portType)
		SpiritBaseConfigurationPLC(device->portType);
	else
		SpiritBaseConfigurationOTA(device->portType, deviceType);
	SpiritVcoCalibration(device->portType);
	SetSpiritFrequency(device->portType, radioInit->frequency);
	SetNormalRunningInterrupts(device);

	//put it into RX
	AssertSpiritState(device, MC_STATE_READY, TRUE, 100);
	device->state = SPIRIT_STATE_READY;
	device->status.txStatus = TX_IDLE;
	SpiritEnterRx_Private(device);

	//RegisterTimeout(&device->updateTimeout, SPIRIT_UPDATE_TIMEOUT);
	device->lastUpdate = GetMsSinceStart();

	//configure rx buffer and port handling
	//RawCircularBufferInit(&(deviceList[deviceIndex].port.rxByteBuffer), (void*)&deviceList[deviceIndex].rxBuffer, 1, 256);
	//deviceList[deviceIndex].portIndex = RegisterPort(&(deviceList[deviceIndex].port));

	device->status.mode = SPIRIT_MODE_NORMAL;
	device->ready = TRUE;
}

////////////////////////////// 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2a,0x2b,0x2c,0x2d,0x2e,0x2f,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4a,0x4b,0x4c,0x4d,0x4e,0x4f,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x5b,0x5c,0x5d,0x5e,0x5f,0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6a,0x6b,0x6c,0x6d,0x6e,0x6f,0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x7b,0x7c,0x7d,0x7e,0x7f,0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0xbe,0xbf,0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf,0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xdb,0xdc,0xdd,0xde,0xdf,0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xeb,0xec,0xed,0xee,0xef,0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff
static uint8_t spiritRegs[] = {0x0c,0xc0,0xa2,0xa2,0xa2,0x02,0x00,0x36,0xa6,0xe8,0xcc,0xc9,0x0d,0xac,0x00,0x00,0x01,0x0e,0x1a,0x25,0x35,0x40,0x4e,0x00,0x3F,0x00,0x48,0x1e,0x76,0x10,0xc8,0x18,0x25,0xe3,0x24,0x58,0x22,0x62,0x8a,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x07,0x3f,0x31,0x00,0x15,0xa8,0x35,0x26,0x1a,0x02,0x20,0x20,0x00,0x30,0x30,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x40,0x00,0x01,0x46,0x01,0x0A,0x01,0x00,0x01,0x00,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x28,0x29,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x07,0x00,0x05,0xe8,0x37,0x08,0x80,0xe3,0x00,0x00,0x00,0x5b,0xa0,0x34,0x11,0xd6,0x35,0x0c,0x90,0x00,0xe1,0x00,0x01,0x02,0x28,0x05,0x83,0xf5,0x00,0x08,0x00,0x42,0x00,0x21,0x10,0xff,0x00,0x01,0x00,0x03,0x03,0x04,0x00,0x00,0x00,0x02,0x67,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x30,0x00,0x00,0x00,0x00,0x00,0x03,0x20,0x0c,0x10,0x60,0x40,0x00,0x00,0x00};
//FEC disabled
//static uint8_t spiritRegs[] = {0x0c,0xc0,0xa2,0xa2,0xa2,0x02,0x00,0x36,0xa6,0xe8,0xcc,0xc9,0x0d,0xac,0x00,0x00,0x01,0x0e,0x1a,0x25,0x35,0x40,0x4e,0x00,0x3F,0x00,0x48,0x1e,0x76,0x10,0xc8,0x18,0x25,0xe3,0x14,0x58,0x22,0x62,0x8a,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x07,0x3f,0x30,0x00,0x15,0xa8,0x35,0x26,0x1a,0x02,0x20,0x20,0x00,0x30,0x30,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x40,0x00,0x01,0x46,0x01,0x08,0x01,0x00,0x01,0x00,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x28,0x29,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x07,0x00,0x05,0xe8,0x37,0x08,0x80,0xe3,0x00,0x00,0x00,0x5b,0xa0,0x34,0x11,0xd6,0x35,0x0c,0x90,0x00,0xe1,0x00,0x01,0x02,0x28,0x05,0x83,0xf5,0x00,0x08,0x00,0x42,0x00,0x21,0x10,0xff,0x00,0x01,0x00,0x03,0x03,0x04,0x00,0x00,0x00,0x02,0x67,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x30,0x00,0x00,0x00,0x00,0x00,0x03,0x20,0x0c,0x10,0x60,0x40,0x00,0x00,0x00};
static uint8_t val;
static void SpiritInitSingleOld(SpiritDevice* device)
{
	//reset structure members
	device->ready = FALSE;
	//device->portType = radioInit->portType;
	device->status.mode = SPIRIT_MODE_SHUTDOWN;
	device->status.generalErrors = 0;
	device->status.txErrors = 0;
	device->status.spiErrors = 0;
	device->status.lastPacketRSSI = 0xFF;
	device->status.lastRSSI = 0xFF;
	device->status.freq = SPIRIT_FREQ_921_25;

	//reset the hardware
	SpiritReset(device->portType);

	/* Workaround for Vtune */
	val = 0xA0;
	SpiWriteRegisters(device->portType, 0x9F, 1, &val);

	for (uint8_t j=0; j<=0xA7; j++)
	{
		if ((j < 0x6D || j > 0x8F) && j != 0xA3)
		{
			device->spiritStatus = SpiWriteRegisters(device->portType, j,1,&(spiritRegs[j]));
			SpiReadRegisters(device->portType, j, 1, &val);
			if (spiritRegs[j] != val)
				return;//ready is still false //while(1);
		}
	}

	val = 0x25;
	SpiWriteRegisters(device->portType, 0xa1, 1, &val);

	//set frequency & power
	SetSpiritFrequency(device->portType, device->frequency);
	SetSpiritTxPower(device->portType, device->txPower);

	SetNormalRunningInterrupts(device);
	AssertSpiritState(device, MC_STATE_READY, TRUE, 100);

	device->state = SPIRIT_STATE_READY;
	//deviceList[deviceIndex].txState = TX_STATE_IDLE;
	SpiritEnterRx_Private(device);

	//RegisterTimeout(&device->updateTimeout, SPIRIT_UPDATE_TIMEOUT);
	device->lastUpdate = GetMsSinceStart();

	device->status.mode = SPIRIT_MODE_NORMAL;
	device->ready = TRUE;
}

static void SetFiltering_private(SpiritDevice* device, uint8_t deviceType, bool enable)
{
	if (enable)
	{
		SpiWriteRegisters(device->portType, PCKT_FLT_GOALS_MULTICAST_BASE, 1, &deviceType);
		val = PCKT_FLT_OPTIONS_CRC_CHECK_MASK | PCKT_FLT_OPTIONS_DEST_VS_TX_ADDR_MASK | PCKT_FLT_OPTIONS_DEST_VS_MULTICAST_ADDR_MASK | PCKT_FLT_OPTIONS_DEST_VS_BROADCAST_ADDR_MASK;
	}
	else
		val = PCKT_FLT_OPTIONS_CRC_CHECK_MASK;

	SpiWriteRegisters(device->portType, PCKT_FLT_OPTIONS_BASE, 1, &val);
}

void SetFiltering(CM2PortType portFlags, uint8_t deviceType, bool enable)
{
	SpiritDevice* device = GetDeviceFromPortType(portFlags);
	if (device != 0)
		SetFiltering_private(device, deviceType, enable);
	return;
}


/*
IRQ_RX_DATA_READY:1;            //!< IRQ: RX data ready
IRQ_RX_DATA_DISC:1;             //!< IRQ: RX data discarded (upon filtering)
IRQ_TX_DATA_SENT:1;             //!< IRQ: TX data sent
IRQ_MAX_RE_TX_REACH:1;          //!< IRQ: Max re-TX reached
IRQ_CRC_ERROR:1;                //!< IRQ: CRC error
IRQ_TX_FIFO_ERROR:1;            //!< IRQ: TX FIFO underflow/overflow error
IRQ_RX_FIFO_ERROR:1;            //!< IRQ: RX FIFO underflow/overflow error
IRQ_TX_FIFO_ALMOST_FULL:1;      //!< IRQ: TX FIFO almost full

IRQ_TX_FIFO_ALMOST_EMPTY:1;     //!< IRQ: TX FIFO almost empty
IRQ_RX_FIFO_ALMOST_FULL:1;      //!< IRQ: RX FIFO almost full
IRQ_RX_FIFO_ALMOST_EMPTY:1;     //!< IRQ: RX FIFO almost empty
IRQ_MAX_BO_CCA_REACH:1;         //!< IRQ: Max number of back-off during CCA
IRQ_VALID_PREAMBLE:1;           //!< IRQ: Valid preamble detected
IRQ_VALID_SYNC:1;               //!< IRQ: Sync word detected
IRQ_RSSI_ABOVE_TH:1;            //!< IRQ: RSSI above threshold
IRQ_WKUP_TOUT_LDC:1;            //!< IRQ: Wake-up timeout in LDC mode

IRQ_READY:1;                    //!< IRQ: READY state
IRQ_STANDBY_DELAYED:1;          //!< IRQ: STANDBY state after MCU_CK_CONF_CLOCK_TAIL_X clock cycles
IRQ_LOW_BATT_LVL:1;             //!< IRQ: Battery level below threshold
IRQ_POR:1;                      //!< IRQ: Power On Reset
IRQ_BOR:1;                      //!< IRQ: Brown out event (both accurate and inaccurate)
IRQ_LOCK:1;                     //!< IRQ: LOCK state
IRQ_PM_COUNT_EXPIRED:1;         //!< IRQ: only for debug; Power Management startup timer expiration (see reg PM_START_COUNTER, 0xB5)
IRQ_XO_COUNT_EXPIRED:1;         //!< IRQ: only for debug; Crystal oscillator settling time counter expired

IRQ_SYNTH_LOCK_TIMEOUT:1;       //!< IRQ: only for debug; LOCK state timeout
IRQ_SYNTH_LOCK_STARTUP:1;       //!< IRQ: only for debug; see CALIBR_START_COUNTER
IRQ_SYNTH_CAL_TIMEOUT:1;        //!< IRQ: only for debug; SYNTH calibration timeout
IRQ_TX_START_TIME:1;            //!< IRQ: only for debug; TX circuitry startup time; see TX_START_COUNTER
IRQ_RX_START_TIME:1;            //!< IRQ: only for debug; RX circuitry startup time; see TX_START_COUNTER
IRQ_RX_TIMEOUT:1;               //!< IRQ: RX operation timeout
IRQ_AES_END:1;                  //!< IRQ: AES End of operation
:1;                             //!< Reserved bit
*/
static uint8_t len = 0;
static void PollSpirit(SpiritDevice* device)
{
	device->spiritStatus = SpiReadRegisters(device->portType, LINEAR_FIFO_STATUS0_BASE,1,&len);
	if (len > 0)
	{
		device->spiritStatus = SpiReadFifo(device->portType, len, fifoBuffer);
		if (!RxBytes(device->portType, len, fifoBuffer))
		{}
	}
/*uint8_t tempRegValue[4];
uint8_t* pIrqPointer = (uint8_t*)(&device->irqStatus);

//Reads IRQ_STATUS registers (and gets MC_STATE for free)
device->spiritStatus = SpiReadRegisters(device->portType, IRQ_STATUS3_BASE, 4, tempRegValue);

//Build the IRQ Status word
for(uint8_t i=0; i<4; i++)
{
	*pIrqPointer = tempRegValue[3-i];
	pIrqPointer++;
}*/
UpdateSpiritIRQs(device);

RunTxStateMachine(device);

	/*if (FALSE == device->holdoffTimeout && (device->irqStatus.IRQ_VALID_PREAMBLE || device->irqStatus.IRQ_RSSI_ABOVE_TH))
	{
		device->holdoffCount++;
		if (device->holdoffCount < NBACKOFF_MAX)
		{
			//hold off again
			BeginHoldoff(device);
		}
		else //ran out of holdoff attempts
		{
			device->status.txStatus = TX_FAIL;

			//re-enable LEDs
			TxBatterySaver(FALSE);

			//go back to rx
			SpiritEnterRx_Private(device);

			//return status
			//SpiritTxError(device);
			TxCallback(device->portType, device->lastPacketMAC, TX_FAIL);
		}
	}
	else if (device->irqStatus.IRQ_RX_DATA_DISC || device->irqStatus.IRQ_RX_DATA_READY || device->irqStatus.IRQ_CRC_ERROR)
	{
		CancelTimeout(&packetInactive);
		packetInactive = TRUE;
	}*/

	//Get RSSI
	/*if (device->spiritStatus.MC_STATE == MC_STATE_RX && device->irqStatus.IRQ_VALID_SYNC == S_SET)
	{
		//RSSI is not populated immediately, wait to make sure it is correct
		//HAL_Delay(2);
		for (volatile uint32_t i=0; i<1000; i++);

		//read
		device->spiritStatus = SpiReadRegisters(RSSI_LEVEL_BASE, 1, &(device->lastRSSI));

		//copy
		device->lastPacketRSSI = device->lastRSSI;
		syncCount++;
	}
	else*/
		device->spiritStatus = SpiReadRegisters(device->portType, RSSI_LEVEL_BASE, 1, &(device->status.lastRSSI));

	if (device->state == SPIRIT_STATE_RX)
		AssertSpiritState(device,MC_STATE_RX, FALSE, 15);

	/*else if (TX_IN_PROGRESS == device->status.txStatus)
	{
		//check for completion
		if (device->txTimeout)
		{
			device->status.txStatus = TX_FAIL;

			//make sure we're not hanging out in holdoff
			if (device->spiritStatus.MC_STATE == MC_STATE_TX)
				device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);
			else if (device->spiritStatus.MC_STATE == MC_STATE_READY)
				device->spiritStatus = SpiCommandStrobe(device->portType, CMD_FLUSHTXFIFO);
			//else if state is sleep
			device->spiritStatus = SpiCommandStrobe(device->portType, CMD_READY);
		}
		else if (device->irqStatus.IRQ_MAX_BO_CCA_REACH)
		{
			device->status.txStatus = TX_FAIL;
			CancelTimeout(&device->txTimeout);
		}
		else if (device->irqStatus.IRQ_TX_DATA_SENT)
		{
			device->status.txStatus = TX_SUCCESS;
			CancelTimeout(&device->txTimeout);
		}

		//handle completion
		if (TX_IN_PROGRESS != device->status.txStatus)
		{
LedPatternSet(0,0,0);
			//re-enable LEDs
			TxBatterySaver(FALSE);

			//go back to rx
			SpiritEnterRx_Private(device);

			//return status
			if (TX_SUCCESS == device->status.txStatus)
				TxCallback(device->portType, device->lastPacketMAC, TX_SUCCESS);
			else
			{
				//SpiritTxError(device);
				TxCallback(device->portType, device->lastPacketMAC, TX_FAIL);
			}
		}
	}	*/

//ClearSpiritIRQs(device);

	device->interrupted = FALSE;
	//device->updateTimeout = FALSE;
	//RegisterTimeout(&device->updateTimeout, SPIRIT_UPDATE_TIMEOUT);
	device->lastUpdate = GetMsSinceStart();
}

static void SpiritSetCwTx(CM2PortType portFlags, bool enable)
{
	SpiritDevice* device = GetDeviceFromPortType(portFlags);
	if (device == 0)
		return;

	AssertSpiritState(device,MC_STATE_READY, TRUE, 10);

	/*SpiCommandStrobe(device->portType, CMD_SABORT);//, device->spiPort);

	// wait for ready
	WaitForSpiritState(device,MC_STATE_READY,100);*/

	if (enable)
	{
		//turn off and clear interrupts
		uint8_t tempRegValue[4]={0x00,0x00,0x00,0x00};
		device->spiritStatus = SpiWriteRegisters(device->portType, IRQ_MASK3_BASE, 4, tempRegValue);
		device->spiritStatus = SpiReadRegisters(device->portType, IRQ_STATUS3_BASE, 4, tempRegValue);
	}
	else
	{
		SetNormalRunningInterrupts(device);
	}

	// Reads the modulation register MOD0 and mask the CW field
	uint8_t tempRegValue2;
	device->spiritStatus = SpiReadRegisters(device->portType, MOD0_BASE, 1, &tempRegValue2);
	if(enable)
		tempRegValue2 |= MOD0_CW;
	else
		tempRegValue2 &= (~MOD0_CW);

	// Writes the new value in the MOD0 register
	device->spiritStatus = SpiWriteRegisters(device->portType, MOD0_BASE, 1, &tempRegValue2);

	if(enable)
	{
		//packet config?
		tempRegValue2 = 0x08;//0x0C;
		device->spiritStatus = SpiWriteRegisters(device->portType, PCKTCTRL1_BASE, 1, &tempRegValue2);

		//go to TX mode
		SpiCommandStrobe(device->portType, CMD_TX);//, device->spiPort);
		device->state = SPIRIT_STATE_TX;

		// wait for tx mode
		WaitForSpiritState(device,MC_STATE_TX,10);
		device->state = SPIRIT_STATE_TX;

//disable persistent TX
/*device->spiritStatus = SpiReadRegisters(device->portType, PROTOCOL0_BASE, 1, &tempRegValue2);
tempRegValue2 &= 0xFE;
device->spiritStatus = SpiWriteRegisters(device->portType, PROTOCOL0_BASE, 1, &tempRegValue2);*/

/*device->spiritStatus = SpiCommandStrobe(device->portType, COMMAND_LOCKTX);
WaitForSpiritState(device,MC_STATE_LOCK,20);*/
/*uint8_t temp = CONF_GPIO_IN_TX_Command|CONF_GPIO_MODE_DIG_IN;
device->spiritStatus = SpiWriteRegisters(device->portType, GPIO0_CONF_BASE, 1, (uint8_t*)&temp);
device->state = SPIRIT_STATE_TX;*/
//while(1);
	}
	else
	{
		//packet reconfig
		tempRegValue2 = 0x20;
		device->spiritStatus = SpiWriteRegisters(device->portType, PCKTCTRL1_BASE, 1, &tempRegValue2);

		//return to RX
		SpiritEnterRx_Private(device);
	}
}

static void SpiritSetRandTx(CM2PortType portFlags, bool enable)
{
	SpiritDevice* device = GetDeviceFromPortType(portFlags);
	if (device == 0)
		return;

	AssertSpiritState(device,MC_STATE_READY, TRUE, 100);

	/*SpiCommandStrobe(device->portType, CMD_SABORT);//, device->spiPort);

	// wait for ready
	WaitForSpiritState(device,MC_STATE_READY,100);*/

	if (enable)
	{
		//turn off and clear interrupts
		uint8_t tempRegValue[4]={0x00,0x00,0x00,0x00};
		device->spiritStatus = SpiWriteRegisters(device->portType, IRQ_MASK3_BASE, 4, tempRegValue);
		device->spiritStatus = SpiReadRegisters(device->portType, IRQ_STATUS3_BASE, 4, tempRegValue);
	}
	else
	{
		SetNormalRunningInterrupts(device);
	}

	// Reads the modulation register PCKTCTRL1 and mask the TX_SOURCE field
	uint8_t tempRegValue2;
	device->spiritStatus = SpiReadRegisters(device->portType, PCKTCTRL1_BASE, 1, &tempRegValue2);
	if(enable)
		tempRegValue2 |= PCKTCTRL1_TX_SOURCE_MASK;
	else
		tempRegValue2 &= (~PCKTCTRL1_TX_SOURCE_MASK);

	// Writes the new value in the PCKTCTRL1 register
	device->spiritStatus = SpiWriteRegisters(device->portType, MOD0_BASE, 1, &tempRegValue2);

	device->spiritStatus = SpiReadRegisters(device->portType, MOD0_BASE, 1, &tempRegValue2);
	if(enable)
		tempRegValue2 |= MOD0_CW;
	else
		tempRegValue2 &= (~MOD0_CW);

	if(enable)
	{
		//packet config?
		//tempRegValue2 = 0x08;//0x0C;
		//device->spiritStatus = SpiWriteRegisters(device->portType, PCKTCTRL1_BASE, 1, &tempRegValue2);

//enable persistent TX
device->spiritStatus = SpiReadRegisters(device->portType, PROTOCOL0_BASE, 1, &tempRegValue2);
tempRegValue2 |= 1;//&= 0xFE;
device->spiritStatus = SpiWriteRegisters(device->portType, PROTOCOL0_BASE, 1, &tempRegValue2);

/*device->spiritStatus = SpiCommandStrobe(device->portType, COMMAND_LOCKTX);
WaitForSpiritState(device,MC_STATE_LOCK,20);*/
/*uint8_t temp = CONF_GPIO_IN_TX_Command|CONF_GPIO_MODE_DIG_IN;
device->spiritStatus = SpiWriteRegisters(device->portType, GPIO0_CONF_BASE, 1, (uint8_t*)&temp);
device->state = SPIRIT_STATE_TX;*/
//while(1);

		//go to TX mode
		SpiCommandStrobe(device->portType, CMD_TX);//, device->spiPort);
		device->state = SPIRIT_STATE_TX;

		// wait for tx mode
		WaitForSpiritState(device,MC_STATE_TX,10);
		device->state = SPIRIT_STATE_TX;

		//AssertSpiritState(device,MC_STATE_TX, FALSE, 100);
	}

	//reduce max current by disabling LEDs while transmitting
	TxBatterySaver(TRUE);

	//strobe tx
	//go to TX mode
	device->spiritStatus = SpiCommandStrobe(device->portType, CMD_TX);//, device->spiPort);

	// wait for tx mode
	if (!WaitForSpiritState(device,MC_STATE_TX,5))
	{
		//packet reconfig
		tempRegValue2 = 0x20;
		device->spiritStatus = SpiWriteRegisters(device->portType, PCKTCTRL1_BASE, 1, &tempRegValue2);

		//return to RX
		SpiritEnterRx_Private(device);
	}
}

static void UpdateRSSI(SpiritDevice* device)
{
	//update IRQ status
	SpiritUpdateStatus(device);

	//enter RX if we aren't already
	if (device->spiritStatus.MC_STATE != MC_STATE_RX)
	{
		//enter RX
		SpiritEnterRx_Private(device);

		//wait for the next time we enter here to abort and read
		return;
	}

	//wait

	//abort RX
	device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);

	//wait
	WaitForSpiritState(device,MC_STATE_READY,5);

	//read RSSI
	device->spiritStatus = SpiReadRegisters(device->portType, RSSI_LEVEL_BASE, 1, &(device->status.lastRSSI));
	//float rssiDB = -120.0+((float)(tempRegValue-20))/2;

	//reenter RX
	SpiritEnterRx_Private(device);
}

void SpiritInit(uint8_t numRadios, SpiritRadioInit* radioInitList, uint8_t devType, uint32_t NID)
{
	for (uint8_t i=0; i<NUM_SPIRIT_RADIOS; i++)
	{
		deviceList[i].ready = FALSE;
		deviceList[i].status.mode = SPIRIT_MODE_SHUTDOWN;

		if (i < numRadios)
		{
			deviceList[i].frequency = radioInitList[i].frequency;
			deviceList[i].portType = radioInitList[i].portType;
			SpiritInitSingleOld(&deviceList[i]);
			//SetFiltering_private(&deviceList[i], devType, TRUE);
		}
	}
}

static uint8_t i;
void RunSpiritBackgroundTasks(void)
{
	for (/*uint8_t*/ i=0; i<NUM_SPIRIT_RADIOS; i++)
	{
		if (!deviceList[i].ready || deviceList[i].status.mode == SPIRIT_MODE_SHUTDOWN)
			continue;

		if (SPIRIT_ERROR_BLANKING != 0xFFFFFFFF && GetMsElapsed(deviceList[i].status.lastErrorTick) > SPIRIT_ERROR_BLANKING)
		{
			deviceList[i].status.generalErrors = 0;
			deviceList[i].status.txErrors = 0;
			deviceList[i].status.spiErrors = 0;
		}

		uint32_t elapsed = GetMsElapsed(deviceList[i].lastUpdate);

		if (deviceList[i].status.mode == SPIRIT_MODE_RSSI_STREAM)
			UpdateRSSI(&deviceList[i]);
		//else if (deviceList[i].portType != PORT_UNKNOWN && (deviceList[i].interrupted || deviceList[i].updateTimeout)) SPIRIT_UPDATE_TIMEOUT
		else if (deviceList[i].portType != PORT_UNKNOWN && (deviceList[i].interrupted || elapsed >= SPIRIT_UPDATE_TIMEOUT || (elapsed >= SPIRIT_UPDATE_TIMEOUT_TX && deviceList[i].state == SPIRIT_STATE_TX)))
			PollSpirit(&deviceList[i]);
	}
}

#if 0
void RunFastSpiritTasks(void)
{
	for (uint8_t i=0; i<NUM_SPIRIT_RADIOS; i++)
	{
		if (deviceList[i].ready &&
			deviceList[i].status.mode != SPIRIT_MODE_SHUTDOWN)/* &&
			deviceList[i].status.txStatus == TX_HOLDOFF)*/
		{
			if (TRUE == deviceList[i].interrupted)
				UpdateSpiritIRQs(&deviceList[i]);

			RunTxFastTasks(&deviceList[i]);
		}
	}
}
#endif

TxStatus SpiritTx(CM2PortType portType, CM2Packet* packet, bool blocking)
{
	SpiritDevice* device = GetDeviceFromPortType(portType);
	if (0 == device)
		return TX_FAIL;

	if (!blocking)
		return BeginNonBlockingTx(device, packet);

	TxStatus result;
	uint32_t startTime = GetMsSinceStart();

	while (TX_FAIL == BeginNonBlockingTx(device, packet))
	{
		PollSpirit(device);
		if (GetMsElapsed(startTime) >= 5)
			return TX_FAIL;
	}

	do{
		if (GetMsElapsed(startTime) >= 10)
			return TX_FAIL;

		PollSpirit(device);
		result = device->status.txStatus;
	} while(TX_ACTIVE == result);
	return result;
}

static uint32_t interruptCount = 0;
void HandleSpiritInterrupt(CM2PortType portFlags)
{
	SpiritDevice* device = GetDeviceFromPortType(portFlags);
	if (device != 0)
	{
#ifndef MINIMAL_SPIRIT_INTERRUPT
		if (!SpiAlreadyActive())
		{
			UpdateSpiritIRQs(device);
			AttemptTxToRxTransition(device);
		}
#endif
		device->interrupted = TRUE;
		interruptCount++;
	}
}


SpiritDriverStatus GetSpiritDriverStatus(CM2PortType portFlags)
{
	SpiritDevice* device = GetDeviceFromPortType(portFlags);

	if (device != 0)
		return device->status;
	else
	{
	  	///////////////TODO - what's the right way to return failure here?
		SpiritDriverStatus nullStatus;
		nullStatus.mode = SPIRIT_MODE_SHUTDOWN;
		return nullStatus;
	}
}

static void SpiritShutdown(CM2PortType portType)
{
	SpiritDevice* device = GetDeviceFromPortType(portType);
	if (device == 0)
		return;

	device->ready = FALSE;

	//abort TX and RX operations
	AssertSpiritState(device, MC_STATE_STANDBY, TRUE, 100);

	//reset regs and turn off interrupts
	device->spiritStatus = SpiCommandStrobe(portType, CMD_SRES);

	//clear interrupts by reading them
	SpiritUpdateStatus(device);

	//errata fix
	uint8_t temp = 0xCA;
	SpiWriteRegisters(portType, 0xB2,1,&(temp));
	temp = 0x04;
	SpiWriteRegisters(portType, 0xA8,1,&(temp));
	for (volatile uint32_t i=0; i<0xFF; i++);
	temp = 0x00;
	SpiWriteRegisters(portType, 0xA8,1,&(temp));

	//delay
	for (volatile uint32_t i=0; i<0xFF; i++);
}

void SetSpiritMode(CM2PortType portType, SpiritMode mode)
{
	SpiritDevice* device = GetDeviceFromPortType(portType);
	if (device == 0)
		return;

	if (device->status.mode == mode)
		return;

	switch(mode)
	{
		case SPIRIT_MODE_SHUTDOWN:
			SpiritShutdown(portType);
			device->status.mode = SPIRIT_MODE_SHUTDOWN;
			break;
		case SPIRIT_MODE_RX_ONLY:
			if (device->status.mode == SPIRIT_MODE_CW_TX)
				SpiritSetCwTx(portType,FALSE);
			device->status.mode = SPIRIT_MODE_RX_ONLY;
			break;
		case SPIRIT_MODE_CW_TX:
			SpiritSetCwTx(portType, TRUE);
			device->status.mode = SPIRIT_MODE_CW_TX;
			break;
		case SPIRIT_MODE_RANDOM_TX:
			SpiritSetRandTx(portType, TRUE);
			device->status.mode = SPIRIT_MODE_RANDOM_TX;
			break;
		case SPIRIT_MODE_RSSI_STREAM:
			////////////////////////////////////
			break;
		default:
			//fall into normal mode
		case SPIRIT_MODE_NORMAL:
			if (device->status.mode == SPIRIT_MODE_CW_TX)
				SpiritSetCwTx(portType,FALSE);
			else if (device->status.mode == SPIRIT_MODE_RANDOM_TX)
				SpiritSetRandTx(portType, FALSE);
			device->status.mode = SPIRIT_MODE_NORMAL;
			break;
	}
}

void SetSpiritFrequency(CM2PortType portFlags, SpiritFreq freq)
{
	SpiritDevice* device = GetDeviceFromPortType(portFlags);
	if (device == 0)
		return;

	if (device->status.freq == freq)
		return;

	switch(freq)
	{
		case SPIRIT_FREQ_921_25:
			//abort rx
			/*device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);
			WaitForSpiritState(device,MC_STATE_READY,100);*/
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);

		  	/*tempRegValue[0] = 0xA6;
			tempRegValue[1] = 0xEA;
			tempRegValue[2] = 0xB8;
			tempRegValue[3] = 0x51;*/
			tempRegValue[0] = 166;
			tempRegValue[1] = 232;
			tempRegValue[2] = 204;
			tempRegValue[3] = 201;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);

			tempRegValue[0] = 0x76;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);

			tempRegValue[0] = 0x1E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);

			//return to RX
			SpiritEnterRx_Private(device);
			break;
		case SPIRIT_FREQ_922_75:
			//abort rx
			/*device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);
			WaitForSpiritState(device,MC_STATE_READY,100);*/
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);

			tempRegValue[0] = 0xA6;//166;
			tempRegValue[1] = 0xEB;//235;
			tempRegValue[2] = 0xAE;//174;
			tempRegValue[3] = 0x09;//17;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			//HAL_Delay(1);

			tempRegValue[0] = 0x76;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);

			tempRegValue[0] = 0x1E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);

			//return to RX
			SpiritEnterRx_Private(device);
			break;
		case SPIRIT_FREQ_926_75: //////////////////////
			//abort rx
			/*device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);
			WaitForSpiritState(device,MC_STATE_READY,100);*/
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);

			tempRegValue[0] = 0xA6;
			tempRegValue[1] = 0xF3;
			tempRegValue[2] = 0x5C;
			tempRegValue[3] = 0x21;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			//HAL_Delay(1);

			tempRegValue[0] = 0x76;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);

			tempRegValue[0] = 0x1E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);

			//return to RX
			SpiritEnterRx_Private(device);
			break;
		case SPIRIT_FREQ_PLC_902: //////////////////////
			//abort rx
			/*device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);
			WaitForSpiritState(device,MC_STATE_READY,100);*/
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);

			tempRegValue[0] = 0x66;
			tempRegValue[1] = 0xC3;
			tempRegValue[2] = 0xD7;
			tempRegValue[3] = 0x01;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			//HAL_Delay(1);

			tempRegValue[0] = 0x76;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);

			tempRegValue[0] = 0x1E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);

			//return to RX
			SpiritEnterRx_Private(device);
			break;
		case SPIRIT_FREQ_PLC_927_5: //////////////////////
			//abort rx
			/*device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);
			WaitForSpiritState(device,MC_STATE_READY,100);*/
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);

			tempRegValue[0] = 0xA6;
			tempRegValue[1] = 0xF4;
			tempRegValue[2] = 0xCC;
			tempRegValue[3] = 0xC9;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			//HAL_Delay(1);

			tempRegValue[0] = 0x76;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);

			tempRegValue[0] = 0x1E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);

			//return to RX
			SpiritEnterRx_Private(device);
			break;
		case SPIRIT_FREQ_EU_868_3: //////////////////////
			//abort rx
			/*device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);
			WaitForSpiritState(device,MC_STATE_READY,100);*/
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);

			tempRegValue[0] = 0x06;
			tempRegValue[1] = 0x83;
			tempRegValue[2] = 0x22;
			tempRegValue[3] = 0xD1;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			//HAL_Delay(1);

			tempRegValue[0] = 0x64;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);

			tempRegValue[0] = 0x5E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);

			//return to RX-
			SpiritEnterRx_Private(device);
			break;
                case SPIRIT_FREQ_EU_868_3_NORMAL:
                        //abort rx
			/*device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);
			WaitForSpiritState(device,MC_STATE_READY,100);*/
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);

			tempRegValue[0] = 0x06;
			tempRegValue[1] = 0x83;
			tempRegValue[2] = 0x22;
			tempRegValue[3] = 0xD1;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			//HAL_Delay(1);

			tempRegValue[0] = 0x76;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);

			tempRegValue[0] = 0x1E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);

			//return to RX-
			SpiritEnterRx_Private(device);
			break;

		case SPIRIT_FREQ_EU_868_95: //////////////////////
			//abort rx
			/*device->spiritStatus = SpiCommandStrobe(device->portType, CMD_SABORT);
			WaitForSpiritState(device,MC_STATE_READY,100);*/
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);

			tempRegValue[0] = 0x06;
			tempRegValue[1] = 0x84;
			tempRegValue[2] = 0x62;
			tempRegValue[3] = 0x49;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			//HAL_Delay(1);

			tempRegValue[0] = 0x64;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);

			tempRegValue[0] = 0x5E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);

			//return to RX
			SpiritEnterRx_Private(device);
			break;
#ifdef ALLOW_TEST_FREQUENCIES
		case SPIRIT_TEST_FREQ_911_25:
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);
		  	tempRegValue[0] = 0x86;
			tempRegValue[1] = 0xD5;
			tempRegValue[2] = 0x99;
			tempRegValue[3] = 0x91;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			tempRegValue[0] = 0x76;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);
			tempRegValue[0] = 0x1E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);
			SpiritEnterRx_Private(device);
			break;
		case SPIRIT_TEST_FREQ_913_25:
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);
		  	tempRegValue[0] = 0x86;
			tempRegValue[1] = 0xDD;
			tempRegValue[2] = 0x47;
			tempRegValue[3] = 0xA9;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			tempRegValue[0] = 0x76;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);
			tempRegValue[0] = 0x1E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);
			SpiritEnterRx_Private(device);
			break;
		case SPIRIT_TEST_FREQ_915_25:
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);
		  	tempRegValue[0] = 0x86;
			tempRegValue[1] = 0xD5;
			tempRegValue[2] = 0x99;
			tempRegValue[3] = 0x91;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			tempRegValue[0] = 0x76;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);
			tempRegValue[0] = 0x1E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);
			SpiritEnterRx_Private(device);
			break;
		case SPIRIT_TEST_FREQ_917_25:
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);
		  	tempRegValue[0] = 0x86;
			tempRegValue[1] = 0xE1;
			tempRegValue[2] = 0x1E;
			tempRegValue[3] = 0xB1;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			tempRegValue[0] = 0x76;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);
			tempRegValue[0] = 0x1E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);
			SpiritEnterRx_Private(device);
			break;
		case SPIRIT_TEST_FREQ_919_25:
			AssertSpiritState(device,MC_STATE_READY, TRUE, 10);
		  	tempRegValue[0] = 0x86;
			tempRegValue[1] = 0xE4;
			tempRegValue[2] = 0xF5;
			tempRegValue[3] = 0xC1;
			SpiWriteRegisters(device->portType, SYNT3_BASE, 4, tempRegValue);
			tempRegValue[0] = 0x76;
			SpiWriteRegisters(device->portType, FDEV0_BASE, 1, tempRegValue);
			tempRegValue[0] = 0x1E;
			SpiWriteRegisters(device->portType, MOD0_BASE, 1, tempRegValue);
			SpiritEnterRx_Private(device);
			break;
#endif
		default:
			return;
	}

	device->status.freq = freq;
}

void SetSpiritTxPower(CM2PortType portType, float dB)
{
	SpiritDevice* device = GetDeviceFromPortType(portType);
	if (device == 0)
		return;

	if (dB > SPIRIT_MAX_TX_POWER)
		dB = SPIRIT_MAX_TX_POWER;
	else if (dB < SPIRIT_MIN_TX_POWER)
		dB = SPIRIT_MIN_TX_POWER;

	device->txPower = dB;

	if (AssertSpiritState(device, MC_STATE_READY, TRUE, 5) == FALSE)
		return; //failed

	//set power
	uint8_t val = (uint8_t)(1 + (SPIRIT_MAX_TX_POWER - dB)*2);
	if (val < 1)
		val = 1;
	else if (val > 90)
		val = 90;
	device->spiritStatus = SpiWriteRegisters(device->portType, PA_POWER1_BASE,1,&val);

	val = 0;
	device->spiritStatus = SpiWriteRegisters(device->portType, PA_POWER0_BASE,1,&val);

	//return to RX
	SpiritEnterRx_Private(device);
}

uint8_t GetRSSI(CM2PortType portType, bool duringPacket)
{
	SpiritDevice* device = GetDeviceFromPortType(portType);
	if (device == 0)
		return 0;

	if (duringPacket)
		return device->status.lastPacketRSSI;
	else
		return device->status.lastRSSI;
}

