
#include "SpiritTx.h"
#include "CM2CoreTypesHeader.h"
#include "MCU_Interface.h"
#include "SPIRIT_Commands.h"
#include "TimersHeader.h"
#include "SpiritPrivateUtils.h"
//#include "stm32l0xx_hal.h"////////////
//#include "HardwareDefines.h"////////////

/*typedef enum {
	SPIRIT_TX_IDLE, //no timeouts active
	SPIRIT_TX_HOLDOFF, //max holdoff + current holdoff active
	SPIRIT_TX_ACTIVE//, //tx timeout active
	//SPIRIT_TX_FAILING
} SpiritInternalTxState;*/

#define TX_ERROR_LIMIT 20
#define TX_TIMEOUT_LEN 15

#define RSSI_TH_FROM_DB(d)		((uint8_t)((d+130)*2)) //0.5dB steps, 20 correspond to -120 dBm
static int MANUAL_CSMA_RSSI_THRESH	= -65; //dB
static bool enableManualCSMA		= FALSE;//TRUE;
//static uint8_t NBACKOFF_MAX			= 5;//0x07; //0 to 7
#define MAX_HOLDOFF_TIME 15 //ms
#define MIN_CS_LISTEN_TIME 1 //increments of 0.1 ms
#define MAX_CS_LISTEN_TIME 5 //increments of 0.1 ms
static uint8_t tempRegValue[2];
static uint8_t txPayload[96];
static uint8_t payloadLen;
//static uint8_t destType;

static bool maxHoldoffTimeout;//////
//static bool failTX = FALSE;
//static bool enterHoldoff = FALSE;
static bool startTimeout = FALSE;
static uint16_t holdoffTimeouts = 0;
static bool returnedToRxInISR = FALSE;
//static SpiritInternalTxState internalTxState = SPIRIT_TX_IDLE;
//static GPIO_InitTypeDef GPIO_InitStruct = {GPIO_PIN_3, GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_HIGH};

static void BeginHoldoff(SpiritDevice* device, bool forceAbort)
{
	//device->status.txStatus = TX_HOLDOFF;

	//abort and restart RX if needed
	if (forceAbort || device->irqStatus.IRQ_RSSI_ABOVE_TH)
	//if (forceAbort)
	{
		AssertSpiritState(device, MC_STATE_READY, TRUE, 10);
		UpdateSpiritIRQs(device);
		AssertSpiritState(device, MC_STATE_RX, TRUE, 10);
	}
	else
	{
		UpdateSpiritIRQs(device);
	}
	//UpdateSpiritIRQs(device);
	//ClearSpiritIRQs(device);
	//device->irqStatus.IRQ_VALID_PREAMBLE = S_RESET;
	device->irqStatus.IRQ_RSSI_ABOVE_TH = S_RESET;

	//set up holdoff
	//device->holdoffTimeout = FALSE;
	//////////GetRandom16()
	uint16_t time = 12;////////MIN_CS_LISTEN_TIME + GetRng(MAX_CS_LISTEN_TIME-MIN_CS_LISTEN_TIME) + device->holdoffCount;
	//RegisterTimeout(&device->holdoffTimeout, time);
	device->holdoffTimer = time;//12;//200;//12;//1.2ms
	device->holdoffCount++;
	device->status.txStatus = TX_HOLDOFF;
}

/*void SpiritTxError(SpiritDevice* device)
{
	//TODO-log error

	device->status.txErrors++;

	if (device->status.txErrors >= TX_ERROR_LIMIT)
	{
		//reset spirit by doing a re-init
		SpiritInitSingleOld(device);
	}
}*/

/*typedef enum {GPIO_Normal=0, GPIO_TX_Ready, GPIO_TX_Trigger} SpiritGpioCmd;
static SpiritIrqs mask;
static void SetSpiritGpio(SpiritDevice* device, SpiritGpioCmd cmd)
{
	if (GPIO_Normal == cmd)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		//set to interrupt pin
		tempRegValue[0] = CONF_GPIO_OUT_nIRQ|CONF_GPIO_MODE_DIG_OUTL;
		device->spiritStatus = SpiWriteRegisters(device->portType, GPIO0_CONF_BASE, 1, tempRegValue);

		//set interrupts
		mask.IRQ_RSSI_ABOVE_TH = S_SET;
		mask.IRQ_TX_DATA_SENT = S_SET;
		device->spiritStatus = SpiWriteRegisters(device->portType, IRQ_MASK0_BASE, 4, (uint8_t*)&mask);
	}
	else if (GPIO_TX_Ready == cmd)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		//UpdateSpiritIRQs(&device);

		//set for TX command
		tempRegValue[0] = CONF_GPIO_IN_TX_Command|CONF_GPIO_MODE_DIG_IN;
		device->spiritStatus = SpiWriteRegisters(device->portType, GPIO0_CONF_BASE, 1, tempRegValue);
	}
	else if (GPIO_TX_Trigger == cmd)
	{
		//GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		//GPIO_InitStruct.Pull = GPIO_PULLUP;
		//HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	}
}*/

static bool BeginActualTX(SpiritDevice* device, bool useGPIO)
{
	//internalTxState = SPIRIT_TX_ACTIVE;
	device->status.txStatus = TX_ACTIVE;

//AssertSpiritState(device, MC_STATE_READY, TRUE, 5);


	//reduce max current by disabling LEDs while transmitting
	TxBatterySaver(TRUE);

	//begin TX
	/*if (useGPIO)
	{
		SetSpiritGpio(device, GPIO_TX_Trigger);
	}
	else*/ if (AssertSpiritState(device, MC_STATE_TX, TRUE, 5) == FALSE)
	{
		device->status.txStatus = TX_FAIL;

		//re-enable LEDs
		TxBatterySaver(FALSE);

		return FALSE;
	}

	//start timeout
	//RegisterTimeout(&device->txTimeout, 10);
	startTimeout = TRUE;
	return TRUE;
}

#if 0
void RunTxFastTasks(SpiritDevice* device)
{
/*if (device->holdoffTimer > 0)
	device->holdoffTimer--;
return;*/

	//holdoff is the only thing that needs this precision
	if (FALSE == enableManualCSMA || device->status.txStatus != TX_HOLDOFF || 0 == device->holdoffTimer)
	{
		return;
	}

	device->holdoffTimer--;

	if (0 == device->holdoffTimer &&
		//S_RESET == device->irqStatus.IRQ_RSSI_ABOVE_TH &&
		FALSE == maxHoldoffTimeout)
	{
		//if (TRUE == device->interrupted)
			UpdateSpiritIRQs(device);

		if (S_RESET == device->irqStatus.IRQ_RSSI_ABOVE_TH)
		{
			//clear to send!
			if (BeginActualTX(device, TRUE) == FALSE)
				failTX = TRUE;
		}
	}
}
#endif

static void CompleteTX(SpiritDevice* device, bool success)
{
	//re-enable LEDs
	TxBatterySaver(FALSE);

	//go back to rx
	SpiritEnterRx_Private(device);

	if (success)
	{
		device->status.txStatus = TX_SUCCESS;
	}
	else
	{
		device->status.txStatus = TX_FAIL;
		SpiritTxError(device);
	}

	TxCallback(device->portType, device->lastPacketMAC, device->status.txStatus);
}

void RunTxStateMachine(SpiritDevice* device)
{
	/*if (failTX && (device->status.txStatus == TX_HOLDOFF || device->status.txStatus == TX_ACTIVE))
	{
		CompleteTX(device, FALSE);
		failTX = FALSE;
	}
	else*/ if (startTimeout && (device->status.txStatus == TX_HOLDOFF || device->status.txStatus == TX_ACTIVE))
	{
		//RegisterTimeout(&device->txTimeout, 10);
		device->txStartTick = GetMsSinceStart();
		//SetSpiritGpio(device, GPIO_Normal);
		startTimeout = FALSE;
	}
	/*else if (enterHoldoff && device->status.txStatus == TX_HOLDOFF)
	{
		BeginHoldoff(device, FALSE);
		enterHoldoff = FALSE;
	}*/

	switch(device->status.txStatus)
	{
	case TX_HOLDOFF:
		//if we're holding off 2 timers are active: the overall timer and the listen timer
		if (maxHoldoffTimeout)
		{
			holdoffTimeouts++;
			CompleteTX(device, FALSE);
		}
		else
		{
			/*UpdateSpiritIRQs(device);
			if (device->irqStatus.IRQ_RSSI_ABOVE_TH)
				BeginHoldoff(device, TRUE);
			else*/ if (device->holdoffTimer == 0)
			{
				UpdateSpiritIRQs(device);
				if (device->irqStatus.IRQ_RSSI_ABOVE_TH)
					BeginHoldoff(device, TRUE);
				else if (BeginActualTX(device, TRUE) == FALSE)
					CompleteTX(device, FALSE);
			}
		}
		break;
	case TX_ACTIVE:
		//if TX is active only the tx timeout timer is running
		//check for successful TX or timer timeout
		if (device->irqStatus.IRQ_TX_DATA_SENT)
		{
			CompleteTX(device, TRUE);
		}
		else if (GetMsElapsed(device->txStartTick) >= TX_TIMEOUT_LEN)//(device->txTimeout)
		{
			CompleteTX(device, FALSE);
		}
		//else TX still active
		break;
	case TX_SUCCESS:
	case TX_FAIL:
	case TX_IDLE:
		if (SPIRIT_STATE_TX == device->state && GetMsElapsed(device->txStartTick) >= TX_TIMEOUT_LEN)
			SpiritEnterRx_Private(device);
		break;
	default:
		device->status.txStatus = TX_IDLE;
		break;
	}

	returnedToRxInISR = FALSE;
}

TxStatus BeginNonBlockingTx(SpiritDevice* device, CM2Packet* packet)
{
	//if we're not in normal mode then return.
	//if we're already transmitting then fail
	if (FALSE == device->ready || device->status.mode != SPIRIT_MODE_NORMAL || device->state == SPIRIT_STATE_TX)
	{
		//device->status.txStatus = TX_FAIL;
		return TX_FAIL;//device->status.txStatus;
	}

	//go to ready state
	if (AssertSpiritState(device, MC_STATE_READY, TRUE, 4) == FALSE)
	{
		SpiritTxError(device);
		device->status.txStatus = TX_FAIL;
		SpiritEnterRx_Private(device);
		return TX_FAIL;
	}

	if (enableManualCSMA)
	{
		//use spirit gpio to move to TX
		//SetSpiritGpio(device, GPIO_Normal);//GPIO_TX_Ready);
	}

	//set device state
	device->txStartTick = GetMsSinceStart();
	device->state = SPIRIT_STATE_TX;
	//device->status.txStatus = TX_HOLDOFF;//IN_PROGRESS;
	device->lastPacketMAC = packet->mac;

	//flush fifo
	device->spiritStatus = SpiCommandStrobe(device->portType, CMD_FLUSHTXFIFO);

	//flatten packet for serial
	FlattenCM2PacketStatic(packet, &payloadLen, txPayload, TRUE);

	//set payload length
	tempRegValue[1]=payloadLen+5;//1;
	tempRegValue[0]=0;//(payloadLen+5)>>8;//1)>>8;
	device->spiritStatus = SpiWriteRegisters(device->portType, PCKTLEN1_BASE, 2, tempRegValue);

	//set dest
	device->spiritStatus = SpiWriteRegisters(device->portType, PCKT_FLT_GOALS_SOURCE_ADDR_BASE, 1, &(packet->destType));
//destType = packet->destType;

	//write fifo
	device->spiritStatus = SpiWriteFifo(device->portType, payloadLen, txPayload);

	if (enableManualCSMA)
	{
		//set RSSI thresh
		tempRegValue[0] = RSSI_TH_FROM_DB(MANUAL_CSMA_RSSI_THRESH);
		device->spiritStatus = SpiWriteRegisters(device->portType, RSSI_TH_BASE, 1, tempRegValue);

		//set CS and PQI filtering
		tempRegValue[0] = 0x29; //thresh + QI_PQI_MASK
		device->spiritStatus = SpiWriteRegisters(device->portType, QI_BASE, 1, tempRegValue);
		tempRegValue[0] = PROTOCOL2_VCO_CALIBRATION_MASK | PROTOCOL2_RCO_CALIBRATION_MASK | /*PROTOCOL2_PQI_TIMEOUT_MASK | PROTOCOL2_SQI_TIMEOUT_MASK |*/ PROTOCOL2_CS_TIMEOUT_MASK;
		device->spiritStatus = SpiWriteRegisters(device->portType, PROTOCOL2_BASE, 1, tempRegValue);

		//set interrupts
		/*SpiritIrqs mask;
		mask.IRQ_RSSI_ABOVE_TH = S_SET;
		mask.IRQ_TX_DATA_SENT = S_SET;
		device->spiritStatus = SpiWriteRegisters(device->portType, IRQ_MASK0_BASE, 4, (uint8_t*)&mask);*/

		//start holdoff to check for activity
		device->holdoffCount = 0;
		RegisterTimeout(&maxHoldoffTimeout, MAX_HOLDOFF_TIME);
		BeginHoldoff(device, FALSE);
//RunTxStateMachine(device);
		return TX_HOLDOFF;
	}
	else
	{
		//LED1_ON;
		if (BeginActualTX(device, FALSE))
		{
			//RegisterTimeout(&device->txTimeout, 10);
			//device->txStartTick = GetMsSinceStart();
			return TX_ACTIVE;
		}
		else
		{
			SpiritTxError(device);
			SpiritEnterRx_Private(device);
			return TX_FAIL;
		}
		//LED1_OFF;
	}
}

void AttemptTxToRxTransition(SpiritDevice* device)
{
    //IRQ and status should have already been read.

    if (TX_ACTIVE == device->status.txStatus &&
        device->irqStatus.IRQ_TX_DATA_SENT &&
        MC_STATE_RX != device->spiritStatus.MC_STATE)
    {
        device->spiritStatus = SpiCommandStrobe(device->portType, CMD_RX);
        returnedToRxInISR = TRUE;
    }
}
