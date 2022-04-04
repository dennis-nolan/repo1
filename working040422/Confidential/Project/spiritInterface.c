//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: appSpiritInterface.c
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Processor: STM32F103R
// TOOLS: IAR Workbench
// DATE:
// CONTENTS: This file contains
//------------------------------------------------------------------------------
// HISTORY: This file
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#include "defs.h"
#include "version.h"
#include "commontypes.h"
#include "CM2CoreHeader.h"
#include "JL_SpiritDriver.h"
#include "CM2SecurityHeader.h"
#include "SpiritInterfaceheader.h"
#include "CM2RoutingHeader.h"
#include "MCInterfaceClass.h"
#include "parameters conversion_f10x.h"
#include "mctasks.h"
#include "intrinsics.h"
#include "CM2FirmwareUpdateHeader.h"
#include "driverFlash.h"
#include "avengersCoreheader.h"
#include "cm2CoreTypesHeader.h"
#include "productionInfo.h"
#include "appMotorRampHeader.h"
#include "cm2PortsHeader.h"
#include "config.h"

//---------------------GLOBAL DEFINITIONS--------------------------
#define SPIRIT_INTERRUPT_PIN GPIO_Pin_7
#define SPIRIT_SHUTDOWN_PORT GPIOB
#define SPIRIT_SHUTDOWN_PIN GPIO_Pin_6

#define MAX_DUP_ENTRIES 5
uint8_t dupOffset = 0;
uint16_t dupTimers[MAX_DUP_ENTRIES];
uint32_t dupMACs[MAX_DUP_ENTRIES];


extern signed long myposcmd;

/*
gearfactor scaling:
a gearfactor of 148,615 reflects the required scaling between angle.cmd 
( where a value of 3000 represents 300 degrees movement at the output of 
the gearbox), and the global variable myposcmd ( where a value of 262,144 
represents one rotation of the motor at the input of the gearbox). 
This factor assumes a gearbox speed reduction of 7.97231:1

If the gearbox ratio is increased or decreased, the gearfactor must be 
increased or decreased proportionately.  For example, if the gear reduction is
increased to 10:1 then gearfactor must be 184,141

NOTE: the range of the product ( angle.cmd * gearfactor ) must always be kept 
less than +/-2,147,483,647 in order to avoid a numeric overflow.

*/

//#define gearfactor 148615
#define gearfactor 117146



AvengerPLCStatusTable currentAvengerPLCStatusTable;
PropCmdTable currentPropCmdTable;
AvengerConfigSteerTable steerConfigTable;
static bool stowed = true;
static uint8_t debugCnt = 99;
  //================================================
  //RAMP_STEP
  // the number to increase the RPM over 100s
  // the reason for this is to get some granularity over the 1 second number
  // so for 19.44 RPS/s = 1944
  //-------------------------------------
  // there is an ELBOW defined at 300 RPM
  // before the ELBOW the Step is RAMP_STEP_PREELBOW
  // after the ELBOW the Step is RAMP_STEP_POSTELBOW
  //-----------------------------------------------------
//#define SPEED_ELBOW     300
//#define RAMP_STEP_PREELBOW 19440
//#define RAMP_STEP_POSTELBOW 100000
//#define RAMP_TIME_BASE 100  //100msec time base

extern CMCI oMCI[MC_NUM];
extern signed short myspeed;

uint32_t tableContents[10]= {

  0x00229EFF,0x001E3282,0x001D9D6C,0x001DDBF4,
  0x001D9D21,0x001E293D,0x00226D25,0x001DD393,
  0x01889933,0x0409349F};

uint16_t opModeTimer = 0xFFFF;
uint16_t opMode = MODE_IDLE;
uint8_t  opModeSetBy = 0xff;

  //-----------prevOpPropulsioinSequence
  // tracking dups commands in OpPropulsion
uint8_t prevOpPropulsioinSequence = 0xff;

uint16_t dupTimer;
uint8_t receivedpacket[30];
uint8_t receivedoffset = 0;
uint16_t lastMAC = 0;

//static uint32_t CM2ID = 0xE86C222B; // JDO id of base for test 4-2-2021
//static uint32_t CM2ID = 0x61145C8B;  

static uint32_t CM2ID = 0x326F8272;  //This is the ID of Dennis's base board
//static uint32_t CM2ID = 0x026138F6;  //This is the ID of John's base board 64pin
//static uint32_t CM2ID = 0x026138F6;  //This is the ID of Normal
//static uint32_t CM2ID = 0X2E4287E5;  //This is the ID oF John's base board 100pin
static Port plc;
static uint8_t buf[128];
static PacketLog hist[8];

#define TEMPDATA_LENGTH 100
uint8_t tempData[TEMPDATA_LENGTH];

//-------------------
// motor speed control
//-------------------
//uint8_t currentStep=0;
int16_t executeRPM = 0;
//int16_t lastForwardRPM = 0;
//int16_t lastReverseRPM = 0;
#define STOW_ON         1
#define STOW_OFF        0
uint8_t currentStow = STOW_ON;

//---------------------LOCAL FUNCTION PROTOTYPES--------------------------
void PropulsionRamp(int16_t finalSpeed);
uint32_t GetID(void);
void HandlePlcAcking(CM2Packet* packet, Opcode ackType, uint8_t nackReason, bool blocking);
void AckResult(uint16_t mac, bool success);
bool CM2SendTableNotification(CM2Packet *ptr);
bool CM2SendPeriodValues(void);
static bool ProcessSteeringTableData(CM2Packet * packet);
static bool ProcessSteeringTableRead(CM2Packet * packet);
bool SendStatusTable2(uint8_t destType, uint8_t lastSequence, SteerCmdTable* receivedCmds);
void UpdateStatusTable(void);
bool SendStatusTable(uint8_t destType, uint8_t lastSequence);

static CM2Packet requestID =
{
	PORT_INTERNAL,
	PORT_SPIRIT_PLC,//destination
	0,//retries
	PROTOCOL_VERSION,
	DEVICE_TYPE,		//src type to be filled out in init
  	0x01,		//dest type
	0xFFFFFFFF,	//src ID
	0xFFFFFFFF,	//dest ID
	0xFF,		//no subaddress
	0x02,		//2 byte payload
	0x0000,		//MAC to be filled out
	{OP_REQUEST, OP_VERSION} //command
};



//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// ---------------------------FUNCTIONS ----------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppModeGet
//------------------------------------------------------------------------------
//
//==============================================================================
uint8_t AppModeGet(void)
{
  return opMode;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppModeHandleTask
//------------------------------------------------------------------------------
//
//==============================================================================
void AppModeHandleTask(void)
{
  if (opModeTimer != 0xffff)
  {
    if (opModeTimer == 1)
    {
      //time out - time to make the switch
      opModeTimer = 0;
      switch (opMode)
      {
        case MODE_CONFIG:
        case MODE_IDLE:
        case MODE_NORMAL:
        {
          opModeSetBy = 0xff;
          opModeTimer = 0xffff;
          opMode = MODE_NORMAL;
          break;
        }
      }
    }
  }
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppModeSwitch
//------------------------------------------------------------------------------
//
//==============================================================================
void AppModeSwitch(uint8_t mode,uint16_t time,uint8_t setBy)
{
  switch(mode)
  {
    case MODE_CONFIG:
    case MODE_IDLE:
    {
      //-------------------
      // turn off motors
      //-------------------
      executeRPM = 0;
      PropulsionRamp(0);
      turnMotorOff = 1;
      opModeSetBy = setBy;
      opModeTimer = time;
      opMode = mode;
      break;
    }
    case MODE_NORMAL:
    {
      opModeSetBy = setBy;
      opModeTimer = time;
      opMode = mode;
      break;
    }
  }
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppRoutePacket
//------------------------------------------------------------------------------
//
//==============================================================================
bool AppRoutePacket(CM2Packet *packet)
{
  bool status = FALSE;
    RoutePacket(packet);
    status = TRUE;
  return status;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  RequestProductID
//------------------------------------------------------------------------------
//
//==============================================================================
void RequestProductID(void)
{
	requestID.mac = CalcMAC(&requestID, GetID());
	AppRoutePacket(&requestID);
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  GetID
//------------------------------------------------------------------------------
//
//==============================================================================
uint32_t GetID(void)
{
  uint32_t ltemp,ltemp2;

  if (CM2ID == 0xffffffff)
  {
    if ((myflashConfigSection.Item.appMode!= 0)&&(AppProductionVerifyInfo()!=0))
    {
      ltemp = myflashConfigSection.Item.testID[7];
      ltemp = ltemp&0x0f;
      ltemp = ltemp<<28;
      ltemp2 = myflashConfigSection.Item.testID[6];
      ltemp2 = ltemp2&0x0f;
      ltemp2 = ltemp2<<24;
      ltemp |= ltemp2;
      ltemp2 = myflashConfigSection.Item.testID[5];
      ltemp2 = ltemp2&0x0f;
      ltemp2 = ltemp2<<20;
      ltemp |= ltemp2;
      ltemp2 = myflashConfigSection.Item.testID[4];
      ltemp2 = ltemp2&0x0f;
      ltemp2 = ltemp2<<16;
      ltemp |= ltemp2;
      ltemp2 = myflashConfigSection.Item.testID[3];
      ltemp2 = ltemp2&0x0f;
      ltemp2 = ltemp2<<12;
      ltemp |= ltemp2;
      ltemp2 = myflashConfigSection.Item.testID[2];
      ltemp2 = ltemp2&0x0f;
      ltemp2 = ltemp2<<8;
      ltemp |= ltemp2;
      ltemp2 = myflashConfigSection.Item.testID[1];
      ltemp2 = ltemp2&0x0f;
      ltemp2 = ltemp2<<4;
      ltemp |= ltemp2;
      ltemp2 = myflashConfigSection.Item.testID[0];
      ltemp2 = ltemp2&0x0f;
      ltemp |= ltemp2;
      CM2ID =ltemp;
    }
  }
  return CM2ID;
}



//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:testSpiritMainMini
//------------------------------------------------------------------------------
//
//==============================================================================
void testSpiritMainMini (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  //-------------------------------
  // initialize spirit radio pins
  // Enable GPIO clock and release reset
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
                        RCC_APB2Periph_GPIOA, (FunctionalState)ENABLE);

    /* Remap TIM2 pins */
//  GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, (FunctionalState)ENABLE);

  //speed is common to all pins
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;


//   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  //-------------------------------
  // SDN pin = C.4 = Output,pullup,speed 40mhz
  //--------------------------------
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_SET);

  //-------------------------------
  // ext1 line GPIO 0 - input,
  // #define M2S_GPIO_0_PORT                        GPIOC
  // #define M2S_GPIO_0_PIN                         GPIO_Pin_7
  // #define M2S_GPIO_0_RCC_PORT                    RCC_AHBPeriph_GPIOC
  // #define M2S_GPIO_0_SPEED                       GPIO_Speed_40MHz
  // #define M2S_GPIO_0_PUPD                        GPIO_PuPd_NOPULL
  // #define M2S_GPIO_0_OTYPE                       GPIO_OType_PP
  // #define M2S_GPIO_0_EXTI_PORT_SOURCE            EXTI_PortSourceGPIOC
  // #define M2S_GPIO_0_EXTI_PIN_SOURCE             EXTI_PinSource7
  // #define M2S_GPIO_0_EXTI_LINE                   EXTI_Line7
  // #define M2S_GPIO_0_EXTI_MODE                   EXTI_Mode_Interrupt
  // #define M2S_GPIO_0_EXTI_TRIGGER                EXTI_Trigger_Falling
  // #define M2S_GPIO_0_EXTI_IRQN                   EXTI9_5_IRQn
  // #define M2S_GPIO_0_EXTI_PREEMPTION_PRIORITY    2
  // #define M2S_GPIO_0_EXTI_SUB_PRIORITY           2
  // #define M2S_GPIO_0_EXTI_IRQ_HANDLER            EXTI9_5_IRQHandler
  //--------------------------------
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource7);

//  EXTI_DeInit(); ///look at this later BETH
  EXTI_ClearITPendingBit(EXTI_Line7);
  EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = (FunctionalState)DISABLE;
  //------------------------------------
  // Connects EXTI Line to MCU GPIO Pin
//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource7);
  EXTI_InitStructure.EXTI_LineCmd = (FunctionalState)ENABLE;
  EXTI_Init(&EXTI_InitStructure);


  //------------------ Power up the radio
  // take c.5 = 0
  // Delay to allow the circuit POR, about 700 us
//  GPIO_WriteBit(GPIOC,GPIO_Pin_5,Bit_RESET);
//  for(volatile uint32_t i=0;i<0x1E00;i++);
  //---------------------------------
  /* Spirit SPI config */
  SpiritSpiInit();

  //--------------------------------------
  // enable the GPIO0 interrupt for the spirit radio
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = (FunctionalState)DISABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static CM2Packet ack =
{
	PORT_INTERNAL,
	PORT_UNKNOWN,//destination
	0,//retries
	PROTOCOL_VERSION,
	DEVICE_TYPE,		//src type to be filled out in init
  	DIRECT_ADDRESSING,
	0x00000000,	//src ID to be filled out in init
	0x00000000,	//dest ID to be filled out before acking
	0x00,		//no subaddress
	0x03,		//3 byte payload
	0x0000,		//MAC to be filled out before acking
	{OP_LOW_LEVEL_ACK,0,0} //will contain opcode + msg MAC
};

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
//
//==============================================================================
void HandlePlcAcking(CM2Packet* packet, Opcode ackType, uint8_t nackReason, bool blocking)
{
  if (DEV_TYPE_BASE == packet->srcType && OP_TABLE_DATA == packet->command[0] && TBL_PROP_CONFIG == packet->command[2])
  {
//        return;
  }
  if (DEV_TYPE_BASE == packet->srcType && OP_TABLE_DATA == packet->command[0] && TBL_PROP_CMD == packet->command[2])
  {
    return;
  }
  if (DEV_TYPE_BASE == packet->srcType && OP_TABLE_DATA == packet->command[0] && TBL_AVENGER_PLC_STATUS == packet->command[2])
  {
    return;
  }
  if (DEV_TYPE_BASE == packet->srcType && OP_TABLE_READ_REQ == packet->command[0] && TBL_PROP_STATUS == packet->command[1])
  {
    return;
  }

  if ((packet->command[0] == OP_PING) ||
      (nackReason == NACK_DONWLOADFAIL) ||
      (((ackType == OP_ACK)&&((packet->command[0] == OP_EX_PACKET_START)||(packet->command[0] == OP_EX_PACKET_DATA)))))
  {
    if (OP_ACK != ackType && OP_NACK != ackType && OP_LOW_LEVEL_ACK != ackType)
       return;

    if (OP_LOW_LEVEL_ACK == packet->command[0])
       return; //don't ack acks

    if (GetID() != packet->destID)// && GetID() != packet->srcID)
       return; //not part of our PLC

    if (DEVICE_TYPE != packet->destType)
       return; //not talking to us

    //populate the packet details
    ack.deviceSource = PORT_INTERNAL;
    ack.deviceDest = packet->deviceSource;
    ack.srcID = GetID();
    ack.srcType = DEVICE_TYPE;
    ack.destType = packet->srcType;
    ack.destID = packet->srcID;

    ack.command[0] = ackType;
    ack.command[1] = (packet->mac)>>8;
    ack.command[2] = packet->mac;
    ack.command[3] = nackReason;
    if (OP_NACK != ackType)
      ack.commandLen = 3;
    else
      ack.commandLen = 4;

    ack.mac = CalcMAC(&ack, GetID());
    //send the ack
    if (blocking)
    {
       TxOnSpecifiedPort(&ack, TRUE);
    }
    else if (OP_LOW_LEVEL_ACK != ackType)
    {
      RoutePacket(&ack);
    }
    else
    {
      if (TX_FAIL == TxOnSpecifiedPort(&ack, FALSE))
      {
        //firstTryFail++;
        RoutePacket(&ack);
      }
    }
  }
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppSpiritInit
//------------------------------------------------------------------------------
//
//==============================================================================
void AppSpiritInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  float powerSetting;


  testSpiritMainMini();
  CM2CoreInit();
  CM2CoreConfigure(GetID(), DEVICE_TYPE, 0xFFFFFFFF, 0, 0, 0);

  __enable_interrupt();

  SpiritRadioInit radioInit;
  radioInit.portType = PORT_SPIRIT_PLC;
  radioInit.frequency = SPIRIT_FREQ_PLC_927_5;
  radioInit.txPower = SPIRIT_MAX_TX_POWER;
  SpiritInit(1, &radioInit,DEVICE_TYPE, 0xFFFFFFFF);


  plc.portType = PORT_SPIRIT_PLC;
  RawCircularBufferInit(&plc.rxByteBuffer,buf,1,128);
  RawCircularBufferInit(&plc.packetHistory,hist,sizeof(PacketLog),8);
  plc.Transmit = SpiritTx;
  plc.automaticLLAcks = FALSE;
  plc.filtering.flags.crcFiltering = TRUE;
  plc.filtering.flags.destIDFiltering = FALSE;
  plc.filtering.flags.destTypeFiltering = FALSE;  //beth TRUE;
  plc.filtering.flags.enforcePairing = FALSE;
  plc.filtering.flags.macFiltering = TRUE;
  plc.filtering.flags.rssiFiltering = TRUE;
  plc.filtering.flags.subaddrFiltering = FALSE;
  plc.filtering.ID = GetID();
  plc.filtering.deviceType = DEVICE_TYPE;
  plc.filtering.rssiThresh = -90;
  plc.filtering.subaddress = DEVICE_TYPE;
  plc.handleAckingCallback = HandlePlcAcking;
  RegisterPort(&plc);

//  SetFiltering(PORT_SPIRIT_PLC, DEVICE_TYPE, TRUE);

  RegisterPacketRxCallback(PacketRX);
  RegisterFlashInterface(APPSIZEALLOWED, FlashEraseScratch, FlashWriteScratch32);
//  powerSetting = tablex40.Item.plcPowerProp;
//  powerSetting = powerSetting-34;
//  SetSpiritTxPower(PORT_SPIRIT_PLC, powerSetting);

  powerSetting = DEFAULT_PLC_PWR_PROP;
  powerSetting = powerSetting-34;
  SetSpiritTxPower(PORT_SPIRIT_PLC, powerSetting);

  EXTI_ClearFlag(EXTI_Line7);
  EXTI_ClearITPendingBit(EXTI_Line7);
  //--------------------------------------
  // enable the GPIO0 interrupt for the spirit radio
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//DISABLE;
  NVIC_Init(&NVIC_InitStructure);
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  SetSpeed
//------------------------------------------------------------------------------
//
//==============================================================================
#define MY_MAX_APPLICATION_SPEED           6500  //JDO change max speed if you want
/*************************************************************************
 * Function Name: Speed
 * Parameters: spdcmd - The speed command received (in % 1-100) from the
 *                        input device (RF/BT/SW)
 * Return: uint16_t
 *
 * Description: Calculated speed (in 0.1Hz) for BLDC Start Motor command
 *
* @param  hFinalSpeed is the value of mechanical rotor speed reference at the
  *         end of the ramp expressed in tenths of HZ.
 1 RPM is equal to 0.0166666666667 hertz.
 1 rpm is equal to 1/60 hz
100 rpm is equal to 100/60 hz which is 100/6 .1hz
 *************************************************************************/
uint16_t SetSpeed(uint16_t spdcmd)
{
  uint16_t spd,newspd;

  newspd = spdcmd;
  if (newspd > MY_MAX_APPLICATION_SPEED)
  {
    newspd  = MY_MAX_APPLICATION_SPEED;
  }
  spd = newspd/6;
  return spd;
}

#define ELIZRAMP 1
#define TESTING_RAMP 0
extern bool wait_for_user_reset;
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  HandlePropCmd
//------------------------------------------------------------------------------
//
//==============================================================================
uint8_t  HandlePropCmd(PropCmdTable* command)
{
  uint8_t parameter,prevOpMode;
  uint8_t status,directionChanged,executeNow;
  uint16_t itemp;
  int8_t ichar,newStep;

  status = 1;

  // BOC SISKO 10-28-2020
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)!=0)
  {
      if((wait_for_user_reset == TRUE) &&
         (command->cmd_action == 0x01))
      {
          OCL_UserReset();
      }
      return 1;
  }

  if((command->cmd_clearVoltageFault == 1) && (command->clearVoltageFault == 1))
  {
       MCI_FaultAcknowledged(oMCI[0]);
  }
  // EOC SISKO
  //---------------------------
  // handle system mode
  //---------------------------
//  if (command->cmd_systemMode != 0)
//  {
    prevOpMode = AppModeGet();
    if (prevOpMode != command->systemMode)
    {
      AppModeSwitch(command->systemMode,0xffff,0x01);
    }
//  }
  prevOpMode = AppModeGet();
  if ((prevOpMode == MODE_CONFIG)||(prevOpMode == MODE_IDLE))
  {
    if ((motorOn != 0)&&(turnMotorOff== 0))
    {
      executeRPM = 0;
      PropulsionRPMRamp(0);
      turnMotorOff = 1;
    }
  }
  //--------------------------
  // check deployed/stow for setting minimum
  // steps based on this change
  // when moved to stowed then change to stowed value
  //--------------------------
  if (command->cmd_stowed != 0)
  {
    if (command->stow != 0)
    {
      //----------------------
      // make sure we are off
      if ((motorOn != 0)&&(turnMotorOff== 0))
      {
        executeRPM = 0;
        PropulsionRPMRamp(0);
        turnMotorOff = 1;
      }
      currentStow = STOW_ON;
    }
    else
    {
      currentStow = STOW_OFF;
    }
  }
  //---------------------------
  // handle FORCE rpm
  //---------------------------
//  if (command->cmd_goToRPM != 0)
//  {

    executeRPM = command->goToRPM;
    PropulsionRPMRamp(executeRPM);
    if (command->goToRPM ==0)
    {
      turnMotorOff = 1;
    }
    else
    {
      if ((opMode!= MODE_IDLE)&&(opMode!= MODE_CONFIG))
      {
        if ((motorOn == 0)||(turnMotorOff!=0))
        {
          MotorStartPropulsion();
          turnMotorOff = 0;
        }
      }
    }
//  }

  switch (command->action)
  {
    case 204:
    {
      if ((opMode!= MODE_IDLE)&&(opMode!= MODE_CONFIG))
      {
        if ((motorOn == 0)||(turnMotorOff!=0))
        {
          MotorStartPropulsion();
          turnMotorOff = 0;
        }
      }
      break;
    }
    case 206:
    {
      if ((motorOn != 0)&&(turnMotorOff== 0))
      {
        executeRPM = 0;
        PropulsionRPMRamp(0);
        turnMotorOff = 1;
      }
      break;
    }
  }
  if (command->cmd_action != 0)
  {
    parameter = command->action;
    if (((parameter>=0)&&(parameter<=200))&&(opMode!= MODE_IDLE)&&(opMode!= MODE_CONFIG))
    {
      if (parameter == 0)
      {
        executeRPM = 0;
        PropulsionRPMRamp(0);
        turnMotorOff = 1;
      }
    }
    else
    {
      status = 0;
    }
  }

  return status;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION: CM2SupervisorTask
//------------------------------------------------------------------------------
//
//==============================================================================
void CM2SupervisorTask(void)
{
  switch (currentPropCmdTable.action)
  {
    case 204:
    {
      if ((opMode!= MODE_IDLE)&&(opMode!= MODE_CONFIG))
      {
        if ((motorOn == 0)||(turnMotorOff!=0))
        {
          MotorStartPropulsion();
          turnMotorOff = 0;
        }
      }
      else
      {
          if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)==0)
          {
              currentPropCmdTable.action  = 0;
          }
      }
      break;
    }
    case 206:
    {
      if ((motorOn != 0)&&(turnMotorOff== 0))
      {
        executeRPM = 0;
        PropulsionRPMRamp(0);
        turnMotorOff = 1;
      }
      break;
    }
  }
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION: CM2WriteTableData
//------------------------------------------------------------------------------
//
//==============================================================================
uint16_t  CM2WriteTableData(uint8_t *packetCommand,uint8_t length)
{
  uint8_t offset,temp;
  uint8_t reason = 0;
  uint8_t i,change;
  float powerSetting;

  change = 0;

  //-----------------------------
  // offset is in [3]
  // table data starts at [4];
  //
  offset = packetCommand[3];
  if (packetCommand[1] != 0x4D)
  {
    reason = NACK_TABLEBADWRITECODE;
  }
  if (reason == 0)
  {
    switch (packetCommand[2])
    {
      case TBL_PROP_CONFIG:
      {
        if ((offset+length)< (MAXSIZE_TABLEx40+1))
        {
          for (i=0;i<length;i++)
          {
            if ((offset+i) >1)
            {
              temp = packetCommand[i+4];
              if (tablex40.Index[i] != temp)
              {
                change = 1;
                tablex40.Index[i]= temp;
              }
            }
          }
          if (change != 0)
          {
            if(tablex40.Item.configBitGroundingOn == 1)
            {
                SetElectorylsisHandling(TRUE);
                FlashUpdateValue(36, 0x01);
            }
            else
            {
                SetElectorylsisHandling(FALSE);
                FlashUpdateValue(36, 0x00);
            }

            FlashConfigUpdate();
            powerSetting = tablex40.Item.plcPowerProp;
            powerSetting = powerSetting-34;
            SetSpiritTxPower(PORT_SPIRIT_PLC, powerSetting);
          }
        }
        else
        {
          reason = NACK_TABLETOOLONG;
        }
        break;
      }
    }
  }
  return reason;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION: HandleTableCommand
//------------------------------------------------------------------------------
//  so at tempBuffer[0] = WHICH table
//        tempBuffer[1] = where to start reading
//        tempBuffer[2] = how much to read
//==============================================================================
uint8_t  HandleTableCommand(uint8_t *packet, uint8_t *tempBuffer)
{
  uint16_t length;
  uint8_t *ptr,i;
  uint32_t ltemp2;

  ptr = tempBuffer;
  length = 0;
  switch (packet[0])
  {
    case 0x01:
    {
      *ptr++ = OP_TABLE_DATA;
      *ptr++ = 0xff;
      //----------table ID
      *ptr++ = 0x01;
      //---------TABLE offset
      *ptr++ = 0x00;
      //----------table version
      *ptr++ = 0x01;
      //--------table flags
      *ptr = 0x00;
      *ptr++ |= TABLEFLAG_READ_ONLY;
      //----------------------- table contents
      // SW identifier of application code running	24
      //    MSB,Mid,LSB. 3 digit code of the SWxxx number.
      // SW identifier of bootloader	24
      //    MSB,Mid,LSB. 3 digit code of the SWxxx number.
      // Model	16	MSB, LSB model number inserted at production time if available
      // Model Revision	16	MSB, LSB  2 digit/character Revision inserted at production in ASCII
      // Lot Code	40	5 bytes of Lot code inserted at production  in ASCII
      // Bootloader Version 	40 	5 byes of bootloader version XW.YZn Not in ASCII.
      // Firmware Version	40	5 byes of firmware version XW.YZn Not in ASCII.
      //-----------------------------------------------
      *ptr++ = '0';
      *ptr++ = '5';
      *ptr++ = '3';
      *ptr++ = '0';
      *ptr++ = '6';
      *ptr++ = '8';

      *ptr++ = myflashConfigSection.Item.model[1];
      *ptr++ = myflashConfigSection.Item.model[0];
      *ptr++ = myflashConfigSection.Item.rev[1];
      *ptr++ = myflashConfigSection.Item.rev[0];
      *ptr++ = myflashConfigSection.Item.lotCode[4];
      *ptr++ = myflashConfigSection.Item.lotCode[3];
      *ptr++ = myflashConfigSection.Item.lotCode[2];
      *ptr++ = myflashConfigSection.Item.lotCode[1];
      *ptr++ = myflashConfigSection.Item.lotCode[0];

      ltemp2 = boot.version;
      ltemp2 = ltemp2 >>20;
      ltemp2 &= 0x0000000f;
      ltemp2 |= 0x30;
      *ptr++ = ltemp2;

      ltemp2 = boot.version;
      ltemp2 = ltemp2 >>16;
      ltemp2 &= 0x0000000f;
      ltemp2 |= 0x30;
      *ptr++ = ltemp2;

      ltemp2 = boot.version;
      ltemp2 = ltemp2 >>12;
      ltemp2 &= 0x0000000f;
      ltemp2 |= 0x30;
      *ptr++ = ltemp2;

      ltemp2 = boot.version;
      ltemp2 = ltemp2 >>8;
      ltemp2 &= 0x0000000f;
      ltemp2 |= 0x30;
      *ptr++ = ltemp2;

      ltemp2 = boot.version;
      ltemp2 &= 0x000000ff;
      *ptr++ = ltemp2;

      *ptr++ = SW_VER0;
      *ptr++ = SW_VER1;
      *ptr++ = SW_VER3;
      *ptr++ = SW_VER4;
      *ptr++ = SW_VER5;

      length = 31;
      break;
    }
     case 0x41:
    {
      *ptr++ = OP_TABLE_DATA;
      *ptr++ = 0xff;
      //----------table ID
      *ptr++ = 0x41;
      //---------TABLE offset
      *ptr++ = 0x00;


      if (motorOn!= 0)
      {
        tablex41.MotorOn = 1;
      }
      else
      {
          tablex41.MotorOn = 0;
      }

      if (turnMotorOff== 0)
      {
          tablex41.turningOff = 0;
      }
      else
      {
          tablex41.turningOff = 1;
      }

      tablex41.stow = 0;
      if (currentStow == STOW_ON)
      {
        tablex41.stow = 1;
      }

      if(motorOn == 0)
      {
          tablex41.RPMMeasured = 0; /*!< Hold reported rpm to 0 when motor is OFF */
      }
      else
      {
          /*!< filter noise when turning motor OFF, */
          if((-50<myspeed) && (myspeed<50))
          {
              tablex41.RPMMeasured = 0;
          }
          else
          {
              tablex41.RPMMeasured = (int16_t)(myspeed * -1);
          }
      }

      tablex41.motorStartFailureCount = motorStartFailureCount;
      tablex41.motorStopFailureCount = motorStopFailureCount;
      tablex41.SystemMode = opMode;
      tablex41.motorMode = GetMotorMode();
      tablex41.angleDifference = GetMotorAngleDifference();
      memcpy(ptr, &tablex41, sizeof(PropulsionStatusTable));
      length = sizeof(PropulsionStatusTable)+4;
      break;
    }
    case 0x40:
    {
      *ptr++ = OP_TABLE_DATA;
      *ptr++ = 0xff;
      //----------table ID
      *ptr++ = 0x40;
      //---------TABLE offset
      *ptr++ = 0x00;
      //----------------------- table contents
      for (i=0;i<MAXSIZE_TABLEx40;i++)
      {
        *ptr++ = tablex40.Index[i];
      }
      length = MAXSIZE_TABLEx40+4;
      break;
    }
  }
  return length;
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  CM2SendTableData
//------------------------------------------------------------------------------
//
//==============================================================================
uint8_t CM2SendTableData(CM2Packet *ptr,uint8_t *tableData)
{

  uint8_t length;
  uint8_t *commandPtr;

  commandPtr = &(ptr->command[1]);

  //---------------------------------
  //abort retries for last packet sent
  //AbortAllRetries();

  CM2Packet tablePacket =
  {
    PORT_INTERNAL,      //packet source
    ptr->deviceSource,        //destination port
    0xFF,                  //retries
    PROTOCOL_VERSION,   //protocol ver
    DEVICE_TYPE,        //src type
    ptr->srcType,               //dest type
    GetID(),          //src ID
    ptr->srcID,         //dest ID
    0x00,               //subaddress
    1,                  //length
    0x0000,             //MAC to be filled out
    {OP_TABLE_DATA},    //command/payload
  };

  length = HandleTableCommand(commandPtr,&tablePacket.command[0]);

  if (length >0)
  {
    tablePacket.retries = 0;
    tablePacket.commandLen = length;
    tablePacket.srcType = DEVICE_TYPE;
    tablePacket.destType = ptr->srcType;
    tablePacket.mac = CalcMAC(&tablePacket, GetID());
    AppRoutePacket(&tablePacket);
  }
  return length;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  CM2SendTableUpdate
//------------------------------------------------------------------------------
//
//=============================================================================
bool CM2SendTableUpdate(uint8_t whichone)
{
  uint8_t which;
  uint8_t length;

  CM2Packet pairPacket =
  {
    PORT_INTERNAL,    //packet source
    PORT_SPIRIT_PLC,      //destination port
    0,             //retries
    PROTOCOL_VERSION, //protocol ver
    DEVICE_TYPE,      //src type
    0xFF,             //dest type
    GetID(),        //src ID
    GetID(),       //dest ID
    0x00,             //subaddress
    0,                //length
    0x0000,           //MAC to be filled out
    {OP_TABLE_DATA},        //command/payload
  };
  //----------------------
  // only valid to use this command for this table 0x13 .....
  which = whichone;
  length = HandleTableCommand(&which,&pairPacket.command[0]);
  if (length >0)
  {
    pairPacket.commandLen = length;
    pairPacket.srcType = DEVICE_TYPE;
    pairPacket.commandLen = length;
    pairPacket.mac = CalcMAC(&pairPacket, GetID());
    AbortSimilar(&pairPacket);
    AppRoutePacket(&pairPacket);
  }
  return 0;
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  SendOPVERSION
//------------------------------------------------------------------------------
//
//==============================================================================
bool SendOPVERSION(CM2Packet *packet)
{
  CM2Packet verPacket =
  {
    PORT_INTERNAL,        //packet source
    PORT_SPIRIT_PLC, //destination port
    0xFF,                 //retries
    PROTOCOL_VERSION,     //protocol ver
    DEVICE_TYPE,          //src type
    0,                    //dest type
    GetID(),             //src ID
    packet->srcID,        //dest ID
    0x00,                 //subaddress
    18,                   //length
    0x0000,               //MAC to be filled out
    {OP_VERSION},
  };

  verPacket.command[0] = OP_VERSION;
  verPacket.command[1] = (uint8_t)(app.version>>24);
  verPacket.command[2] = (uint8_t)(app.version>>16);
  verPacket.command[3] = (uint8_t)(app.version>>8);
  verPacket.command[4] = (uint8_t) app.version;

  verPacket.command[5] = (uint8_t)(boot.version>>24);
  verPacket.command[6] = (uint8_t)(boot.version>>16);
  verPacket.command[7] = (uint8_t)(boot.version>>8);
  verPacket.command[8] = (uint8_t) boot.version;

  verPacket.command[9] = myflashConfigSection.Item.model[1];
  verPacket.command[10] = myflashConfigSection.Item.model[0];
  verPacket.command[11] = myflashConfigSection.Item.rev[1];
  verPacket.command[12] = myflashConfigSection.Item.rev[0];

  verPacket.command[13] =  myflashConfigSection.Item.lotCode[4];
  verPacket.command[14] =  myflashConfigSection.Item.lotCode[3];
  verPacket.command[15] =  myflashConfigSection.Item.lotCode[2];
  verPacket.command[16] =  myflashConfigSection.Item.lotCode[1];
  verPacket.command[17] =  myflashConfigSection.Item.lotCode[0];

  verPacket.srcType = DEVICE_TYPE;
  verPacket.destType = packet->srcType;
  verPacket.mac = CalcMAC(&verPacket,GetID());
  AbortSimilar(&verPacket);
  return AppRoutePacket(&verPacket);

}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  CM2SendACK
//------------------------------------------------------------------------------
//
//==============================================================================
bool CM2SendACK(CM2Packet *ptr)
{
  uint16_t temp,mac;

  mac = ptr->mac;
  CM2Packet ACKPacket =
  {
    PORT_INTERNAL,    //packet source
    PORT_SPIRIT_PLC,      //destination port
    0x00,             //retries
    PROTOCOL_VERSION, //protocol ver
    DEVICE_TYPE,      //src type
    0x00,             //dest type
    GetID(),        //src ID
    ptr->srcID,       //dest ID
    0x00,             //subaddress
    3,                //length
    0x0000,           //MAC to be filled out
    {OP_ACK},        //command/payload
  };
  ACKPacket.srcType = DEVICE_TYPE;
  ACKPacket.destType = ptr->srcType;
  temp = mac>>8;
  ACKPacket.command[1] = temp;
  temp = mac & 0xff;
  ACKPacket.command[2] = temp;
  ACKPacket.mac = CalcMAC(&ACKPacket, GetID());
  return AppRoutePacket(&ACKPacket);
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  CM2SendNACK
//------------------------------------------------------------------------------
//
//==============================================================================
bool CM2SendNACK(CM2Packet *packet, uint8_t reason)
{
  uint16_t temp;
  //abort retries for last packet sent
  //AbortAllRetries();

  CM2Packet NACKPacket =
  {
    PORT_INTERNAL,    //packet source
    PORT_SPIRIT_PLC,      //destination port
    0xFF,             //retries
    PROTOCOL_VERSION, //protocol ver
    DEVICE_TYPE,      //src type
    0x00,             //dest type
    GetID(),        //src ID
    packet->srcID,       //dest ID
    0x00,             //subaddress
    4,                //length
    0x0000,           //MAC to be filled out
    {OP_NACK},        //command/payload
  };
  NACKPacket.srcType = DEVICE_TYPE;
  NACKPacket.destType = packet->srcType;
  temp = packet->mac>>8;
  NACKPacket.command[1] = temp;
  temp = (packet->mac) & 0xff;
  NACKPacket.command[2] = temp;
  NACKPacket.command[3] = reason;
  NACKPacket.mac = CalcMAC(&NACKPacket, GetID());
  AbortSimilar(&NACKPacket);
  return RoutePacket(&NACKPacket);
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  PacketRX
//------------------------------------------------------------------------------
//
//==============================================================================
RxPacketResult PacketRX(CM2Packet* packet)
{
  uint8_t newOne,i,reason;
  uint16_t ltemp,ltemp2;

  RxPacketResult result = RX_PACKET_UNHANDLED;
  // Check if the app cares about this packet,
  // if so then handle it here

  spiritSupTimer = SPIRIT_SUP_TIME;
  if(((packet != 0)&&((packet->destType == DEV_TYPE_STEER)||(packet->destType == DEV_TYPE_PROP)  )) ||
      ((packet->command[0] == OP_EX_PACKET_DATA) ||
       (packet->command[0] == OP_EX_PACKET_START)))
  {
    //---------------------------------------
    // Handle dups
    for (i=0;i<MAX_DUP_ENTRIES;i++)
    {
      if (dupTimers[i] == 0)
      {
        dupMACs[i] = 0;
      }
    }
    newOne = TRUE;
    for (i=0;i<MAX_DUP_ENTRIES;i++)
    {
      if (packet->mac == dupMACs[i])
      {
        newOne = FALSE;
      }
    }
    if (newOne == TRUE)
    {
      //---------------------
      // add to oldest spot in the list
      if (dupOffset >= MAX_DUP_ENTRIES)
      {
        dupOffset=0;
      }
      dupMACs[dupOffset] = packet->mac;
      dupTimers[dupOffset] = 100;
      dupOffset++;
    }
    //==============================================
    if (newOne == TRUE)
    {
      if (packet->destID != GetID())
      {
         if (packet->command[0] == OP_VERSION)
         {
           if (PORT_SPIRIT_PLC == packet->deviceSource && 0xFFFFFFFF == GetID())
           {
             CM2ID = packet->srcID;
           }
         }
         result = RX_PACKET_HANDLED;
      }
      else
      {
        switch(packet->command[0])
        {
          case 0x18:
          {
            result = RX_PACKET_HANDLED;
            CM2SendACK(packet);
            //-----------------------
            // start test
            //----------------------
            AppTestProcessSpecial(packet);
            if (testState == TESTSTATE_CW)
            {
              while (1)
              {
 //                 WDOGTask();
              }
            }
            break;
          }
          case OP_SIGNAL:
          {
//            opSignalTime = DELAY_3SEC;
//            CM2SendACK(packet);
            result = RX_PACKET_HANDLED;
            break;
          }
          case OP_MODE_WRITE:
          {
            if (packet->srcType == 0x01)
            {
              ltemp = packet->command[2];
              ltemp = ltemp<<8;
              ltemp2 = packet->command[3];
              ltemp |= ltemp2;
              AppModeSwitch(packet->command[1],ltemp,packet->srcType);
            }
            result = RX_PACKET_HANDLED;
            break;
          }
          case OP_COMM:
          {
            result = RX_PACKET_HANDLED;
            switch (packet->command[1])
            {
              case 0x22:
              {
               StartTimingAdjust(1);
                break;
              }
            }
            break;
          }
          case OP_REQUEST:
          {
            //what is being requested?
            switch(packet->command[1])
            {
              case OP_VERSION:
              {
                SendOPVERSION(packet);
                result = RX_PACKET_HANDLED;
                break;
              }
              case 0xE9:
              {
                ltemp = 0;
                ltemp = ltemp | ((packet->command[2]) << 8);
                ltemp = ltemp | (packet->command[3]);
                SetAngleErrorTolerance(ltemp);
                result = RX_PACKET_HANDLED;
                break;
              }
              default:
                break;
            }
            break;
          }
          case OP_VERSION:
          {
            if (PORT_SPIRIT_PLC == packet->deviceSource && 0xFFFFFFFF == GetID())
            {
              CM2ID = packet->srcID;
            }
            break;
          }
            case OP_TABLE_READ_REQ:
            {
              if ((packet->command[2] == 0x00)&& (packet->command[3]== 0xFF))
              {
                //-----------WHICH table.
                switch(packet->command[1])
                {
                  case TBL_STEERING_STATUS:
                  case TBL_STEER_CONFIG:
                  {
                      ProcessSteeringTableRead(packet);
                      result = RX_PACKET_HANDLED;
                      break;
                    }
                  case 0x01:
 //removed prop only                 case TBL_PROP_CONFIG:
 //removed prop only                 case TBL_PROP_STATUS:
                  {
                    if (CM2SendTableData(packet,tempData)== 0)
                    {
                       CM2SendNACK(packet,0);
                                       result = RX_PACKET_ERR_OTHER;
                    }
                                    else
                                       result = RX_PACKET_HANDLED;
                    break;
                  }
                  default:
                  {
                    //----------reason needs to be updated here
                    if (packet->destType == DEV_TYPE_STEER)
                    {
                      CM2SendNACK(packet,NACK_TABLENOTSUPPORTED);
                                    result = RX_PACKET_ERR_OTHER;
                    }
                    break;
                  }
                }
              }
              else
              {
                    CM2SendNACK(packet,NACK_SUPPORT_READALLONLY);
                                    result = RX_PACKET_ERR_OTHER;
              }
              break;
            }
            case OP_TABLE_DATA:
            {
              if ((packet->destType == DEV_TYPE_STEER)||(packet->destType == DEV_TYPE_PROP))
              {
                 //-----------WHICH table.
                switch(packet->command[2])
                {
                  case TBL_STEER_CMD:
                  case TBL_STEER_CONFIG: 
                  {
                    if (ProcessSteeringTableData(packet))
                    {
                      result = RX_PACKET_HANDLED;
                    }
                    break;
                  }                
                  case TBL_PROP_CMD:
                  {
                      if (IsTimingAdjustActive()==0)
                      {
                        if (tablex41.lastSequence != packet->command[6])
                        {
                          memcpy(&currentPropCmdTable,&packet->command[7],(sizeof(PropCmdTable)));
                          HandlePropCmd(&currentPropCmdTable);
                        }
                      }
                      packet->command[1] = TBL_PROP_STATUS;
                      tablex41.lastSequence = packet->command[6];
                      if (CM2SendTableData(packet,tempData)== 0)
                      {
                      }

                    result = RX_PACKET_HANDLED;
                    break;
                  }
/*                 
                  case TBL_PROP_CONFIG:
                  {
                    reason = CM2WriteTableData(&packet->command[0],((packet->commandLen) -4));
                    if (reason != 0)
                    {
                      CM2SendNACK(packet,reason);
                                      result = RX_PACKET_ERR_OTHER;
                    }
                    else
                    {
                      //-----------------------------
                      // send updated table out
                      if (CM2SendTableUpdate(packet->command[2])==0)
                      {
                      }
                      result = RX_PACKET_HANDLED;
                    }
                    break;
                  }
*/
                  default:
                  {
                    result = RX_PACKET_ERR_OTHER;
                    break;
                  }
                }
              }
              else
              {
                result = RX_PACKET_HANDLED;
              }
              break;
            }
        case OP_RESET:
            {
                DeviceReset();
                result = RX_PACKET_HANDLED;
                break;
            }
        case OP_EX_PACKET_START:
            {
                if ((PORT_SPIRIT_PLC == packet->deviceSource) && (packet->srcID == GetID())&&
                    (0xFFFFFFFF != GetID())&&(packet->destID == GetID()) && (packet->destType == DEVICE_TYPE))
                {
                    __NOP();
                }
                else
                {
                    result = RX_PACKET_HANDLED;
                }
            break;
            }
        case OP_EX_PACKET_DATA:
            {
                if ((PORT_SPIRIT_PLC == packet->deviceSource) && (packet->srcID == GetID())&&
                    (0xFFFFFFFF != GetID())&&(packet->destID == GetID()) && (packet->destType == DEVICE_TYPE))
                {
                    __NOP();
                }
                else
                {
                    result = RX_PACKET_HANDLED;
                }
            break;
            }

        default:
            {
                break;
            }
        }
      }
    }
  }
   return result;
}

static CM2Packet cfgPacket =
{
	PORT_INTERNAL,
	PORT_SPIRIT_PLC,//destination
	0,//DEFAULT_RETRIES_NORM,//retries
	PROTOCOL_VERSION,
	DEV_TYPE_STEER,		//src type
	0xFF,		//dest type
	0xFFFFFFFF,	//src ID
	0xFFFFFFFF,	//dest ID
	0xFF,		//no subaddress
	0x04+sizeof(AvengerConfigSteerTable),		//payload len
	0x0000,		//MAC to be filled out
	{OP_TABLE_DATA,0xFF,TBL_STEER_CONFIG,0} //command
};

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:processTableData
//------------------------------------------------------------------------------
//
//==============================================================================
enum {
    STR_MODE_RELEASE,
    STR_MODE_OPERATE,
    STR_MODE_UPDATE
};
static uint16_t lastPacketRelHome = INT16_MIN;
static bool deployed = false,enteredNormal = false;;
static AdvancedSteerCmd cmd;
static uint8_t steeringMode = STR_MODE_RELEASE;
static bool ProcessSteeringTableData(CM2Packet * packet)
{
    CM2CmdField_t * cmdField = (CM2CmdField_t *) & packet->command[0];

    uint8_t prevMode;

    switch (packet->command[2])
    {
      case TBL_STEER_CMD:
      {
        //handle command packet
        SteerCmdTable* steerCommands = (SteerCmdTable*)&packet->command[7];

        enteredNormal = false;
        prevMode = AppModeGet();
        if (steerCommands->cmd_systemMode)
        {
           //if (steerCommands->systemMode != GetMode())
           {
              AppModeSwitch(steerCommands->systemMode, 0xFFFF,1);
           }
        }
        if (MODE_NORMAL == AppModeGet() && MODE_NORMAL != prevMode)
            enteredNormal = true;

        //respond
        if (steerCommands->returnStatus == 0)
        {
          SendStatusTable2(DEV_TYPE_BASE, packet->command[6], steerCommands);
        }

        if (stowed && !steerCommands->stow)
            deployed = true;
        stowed = steerCommands->stow;

        //reset all fields, may be overwritten below
        cmd.setTarget = TA_NO_ACTION;
        cmd.limitSpeed = 0;
        cmd.minimumTime = 0;
        cmd.angle = 0;
        cmd.speedLimit = 0xFFFF;
        cmd.time = 0xFFFF;
        cmd.holdRelease = HR_NO_ACTION;
        cmd.setHome = SH_NO_ACTION;
        cmd.homeAngle = 0;

        //check to see if this packet sets the home position
        if (steerCommands->cmd_setHome)
        {
            if (steerCommands->setHomeRelative)
            {
                //don't repeat relative set home commands
                if (lastPacketRelHome != steerCommands->setHome)
                {
                    //SteeringControlRegs_SetHome(steerCommands->setHome, !steerCommands->setHomeRelative);
                    cmd.homeAngle = steerCommands->setHome;
                    if (0 == steerCommands->setHome)
                        cmd.setHome = SH_HERE;
                    else
                        cmd.setHome = SH_RELATIVE;
                    lastPacketRelHome = steerCommands->setHome;
                }
                //else ignore command
            }
            else //absolute
            {
                lastPacketRelHome = INT16_MIN;

                cmd.setHome = SH_ABSOLUTE;
                cmd.homeAngle = steerCommands->setHome;
            }
        }
        else
            lastPacketRelHome = INT16_MIN;

        //figure out what mode we're going to be in for this session
        if (MODE_NORMAL != AppModeGet() || stowed || SH_NO_ACTION != cmd.setHome ||
                 (steerCommands->cmd_released && steerCommands->released))
        {
            steeringMode = STR_MODE_RELEASE;
        }
        else
        {
            steeringMode = STR_MODE_OPERATE;
        }

        //execute modes
        if (STR_MODE_UPDATE == steeringMode)
        {
   
        }
        else if (STR_MODE_OPERATE == steeringMode)
        {
            if (deployed || /*(enteredNormal && !stowed) ||*/ steerCommands->externalSteerCmd ||
                (steerCommands->cmd_released && !steerCommands->released))
            {
                cmd.holdRelease = HR_HOLD;
            }
            //----------------DENNIS - TARGET ANGLE hooks
            if (steerCommands->cmd_targetAngle)
            {
                //update the TA in the status table right away, only if absolute
 /* sending a command to the PIC - removed EMH                    
                if (steerCommands->returnStatus && !steerCommands->targetRelative)
                    SteeringControlRegs_UpdateTargetAngle(steerCommands->targetAngle);
*/
                if (steerCommands->targetRelative)
                    cmd.setTarget = TA_RELATIVE;
                else
                    cmd.setTarget = TA_ABSOLUTE;

                cmd.limitSpeed = steerCommands->cmd_limitSpeed;
                cmd.minimumTime = steerCommands->cmd_minimumTime;
                cmd.angle = steerCommands->targetAngle;
                cmd.speedLimit = steerCommands->speedLimit;
                cmd.time = steerCommands->minTime;
            }

//dcnq 8-18-21
myposcmd = (cmd.angle * gearfactor)>>6;



            //don't bother sending if nothing is happening
            if (HR_NO_ACTION != cmd.holdRelease || TA_NO_ACTION != cmd.setTarget)
            {
/* sending a command to the PIC - removed EMH              
                cmd.crc16 = CalcStrCRC16(((uint8_t*)&cmd)+2, sizeof(AdvancedSteerCmd)-2);
                SteeringControlComm_SendAdvCmd(&cmd);
*/
                deployed = false;
            }
        }
        else //STR_MODE_RELEASE
        {
            cmd.holdRelease = HR_RELEASE;
/* sending a command to the PIC - removed EMH                   
            cmd.crc16 = CalcStrCRC16(((uint8_t*)&cmd)+2, sizeof(AdvancedSteerCmd)-2);
            SteeringControlComm_SendAdvCmd(&cmd);
*/
        }
        break;
      }
      case TBL_STEER_CONFIG:
      {
        if (packet->commandLen >= (4 + sizeof(AvengerConfigSteerTable)))
        {
          cfgPacket.destType = packet->srcType;
          cfgPacket.srcID = GetID();
          cfgPacket.destID = GetID();
          memcpy(&cfgPacket.command[0], &packet->command[0], sizeof(AvengerConfigSteerTable));
          memcpy(&steerConfigTable,&packet->command[4],sizeof(AvengerConfigSteerTable));
          cfgPacket.command[1] = 0xff;
          cfgPacket.mac = CalcMAC(&cfgPacket, cfgPacket.destID);
          AppRoutePacket(&cfgPacket);
          AvengerConfigSteerTable* tbl = (AvengerConfigSteerTable*)&packet->command[4];
          SetSpiritTxPower(PORT_SPIRIT_PLC, (float)tbl->plcPowerSteer - 34.0);
        }
        break;
      }
    }
}

static CM2Packet replyPacket =
{
    PORT_INTERNAL,      // packet source
    PORT_SPIRIT_PLC,    // destination port
    0,                  // retries
    PROTOCOL_VERSION,   // protocol ver
    DEVICE_TYPE,        // src type
    0xFF,               // dest type
    0xFFFFFFFF,         // src ID
    0xFFFFFFFF,         // dest ID
    0x00,               // subaddress
    0,                  // length
    0x0000              // MAC to be filled out
};

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  processTableRead
//------------------------------------------------------------------------------
//
//==============================================================================
static bool ProcessSteeringTableRead(CM2Packet * packet)
{
  bool status = 0;

  switch(packet->command[1])
  {
    case TBL_STEERING_STATUS:
    {
      SendStatusTable(packet->srcType, 0xFF);
      status = 1;
      break;
    }
    case TBL_STEER_CONFIG:
    {
      replyPacket.destType = packet->srcType;
      replyPacket.destID = GetID();
      replyPacket.srcID = GetID();
      replyPacket.command[0] = OP_TABLE_DATA;
      replyPacket.command[1] = 0xff;
      replyPacket.command[2] = TBL_STEER_CONFIG;
      replyPacket.command[3] = 0x00;
      memcpy(&replyPacket.command[4], &steerConfigTable, sizeof(AvengerConfigSteerTable));
      replyPacket.commandLen = sizeof(AvengerConfigSteerTable) + 4;
      replyPacket.mac = CalcMAC(&replyPacket, GetID());
      AppRoutePacket(& replyPacket);
      status = 1;
      break;
    }
  }
  return status;
}

static CM2Packet statusPacket =
{
	PORT_INTERNAL,
	PORT_SPIRIT_PLC,//destination
	0,//DEFAULT_RETRIES_NORM,//retries
	PROTOCOL_VERSION,
	DEV_TYPE_STEER,		//src type
	0xFF,		//dest type
	0xFFFFFFFF,	//src ID
	0xFFFFFFFF,	//dest ID
	0xFF,		//no subaddress
	0x04+sizeof(SteeringStatusTable),		//payload len
	0x0000,		//MAC to be filled out
	{OP_TABLE_DATA,0xFF,TBL_STEERING_STATUS,0} //command
};
static SteeringStatusTable* statusTable = (SteeringStatusTable*)(&statusPacket.command[4]);

bool SendStatusTable(uint8_t destType, uint8_t lastSequence)
{
    UpdateStatusTable();

    //if (DEV_TYPE_BASE != destType || statusPacket.srcID != GetID())
    {
        statusPacket.destType = destType;
        statusTable->lastSequence = lastSequence;
        statusPacket.mac = CalcMAC(&statusPacket, statusPacket.destID);
    }

    return AppRoutePacket(&statusPacket);
}

bool SendStatusTable2(uint8_t destType, uint8_t lastSequence, SteerCmdTable* receivedCmds)
{
    UpdateStatusTable();

    if (MODE_NORMAL == AppModeGet() && receivedCmds->cmd_targetAngle)
    {
/* sending a command to the PIC - removed EMH             
        statusTable->targetAngle = receivedCmds->targetAngle;
        if (receivedCmds->targetRelative)
            statusTable->targetAngle += steering_motor_data[SteeringReg_Current_Angle];
*/
    }

    statusPacket.destType = DEV_TYPE_BASE;
    statusTable->lastSequence = lastSequence;
    statusPacket.mac = CalcMAC(&statusPacket, statusPacket.destID);

    return AppRoutePacket(&statusPacket);
}



void UpdateStatusTable(void)
{
  uint16_t itemp,itemp2;

    debugCnt = 0;
    //populate from saved regs
    statusTable->tblVer = 0;
    statusTable->tblFlags = 1;
    statusTable->boardVoltage = 0xFFFF;/////////
    statusTable->systemMode = AppModeGet();
    //-------------------
    // fix to match versions that are not exactly right from steering motor
/* sending a command to the PIC - removed EMH        
    itemp = steering_motor_data[SteeringReg_Firmware_Version_Major];
    itemp = itemp<<8;
    itemp2 = steering_motor_data[SteeringReg_Firmware_Version_Minor];
    itemp2 &= 0xff;
    itemp |= itemp2;
    statusTable->motorFwVerMajor = 0; //steering_motor_data[SteeringReg_Firmware_Version_Major];
    statusTable->motorFwVerMinor = itemp; //steering_motor_data[SteeringReg_Firmware_Version_Minor];
    statusTable->hwRevision = steering_motor_data[SteeringReg_Hardware_Rev];
    //statusTable->torque = steering_motor_data[SteeringReg_Current_Torque];
    debugCnt = 10;
    statusTable->current = steering_motor_data[SteeringReg_Amps_Measured];
    //statusTable->speed = steering_motor_data[SteeringReg_Current_Speed];
    statusTable->currentAngle = steering_motor_data[SteeringReg_Current_Angle];
    statusTable->targetAngle = steering_motor_data[SteeringReg_Target_Angle];
    statusTable->temperature = steering_motor_data[SteeringReg_BoardTemp];
    statusTable->steeringReleased = steering_motor_data[SteeringReg_Override];
    statusTable->downloadingMotor = AppMotorDownloadActive();
    statusTable->lastMotorDownloadSuccess = appDownloadSuccess;
    statusTable->lastMotorDownloadFailure = appDownloadFailure;
    statusTable->lastMotorDownloadFailureChecksum = appDownloadFailureChecksum;
    statusTable->lastMotorDownloadFailureGetInfo = appDownloadFailureGetInfo;
    statusTable->lastCmdDevice = 0xFF;
    statusTable->storedMotorFWVersion = motorScratch.version;
    statusTable->downloadChunkOffset = motorDownloadChunkOffset;
    statusTable->downloadChunkTotal = motorDownloadChunkCount;
    statusTable->picHealthBad = IsPicHealthBad();
    statusTable->picNoCheckin = !picCheckedin;
*/
   // ----  JDO this is where the table value is inserted for current angle
    // ----  This goes back to the base board so it always keeps track of steering angle
    statusTable->currentAngle = cmd.angle;
    
    statusPacket.destID = GetID();
    statusPacket.srcID = statusPacket.destID;
    statusPacket.destType = DEV_TYPE_BASE; //default
    //statusPacket.mac = CalcMAC(&statusPacket, statusPacket.destID);
    debugCnt = 20;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
//
//==============================================================================
extern uint8_t spiritWdog;
void Delay(uint8_t timing)
{
   spiritWdog = timing;
   while (spiritWdog >0)
   {
   }
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
//
//==============================================================================
void SpiritReset(CM2PortType portType)
{
	GPIO_WriteBit(SPIRIT_SHUTDOWN_PORT, SPIRIT_SHUTDOWN_PIN, Bit_SET);
	Delay(25);
	GPIO_WriteBit(SPIRIT_SHUTDOWN_PORT, SPIRIT_SHUTDOWN_PIN, Bit_RESET);
	Delay(25);
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
//
//==============================================================================
void TxBatterySaver(bool enable)
{
	// Can be left empty since this is not a small-battery device
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
//
//==============================================================================
void EXTI9_5_IRQHandler(void)
{
  //------------------------------
  // Check the flag status of EXTI line
  //------------------------------
  if(EXTI_GetFlagStatus(EXTI_Line7))
  {
    EXTI_ClearFlag(EXTI_Line7);
    EXTI_ClearITPendingBit(EXTI_Line7);
    HandleSpiritInterrupt(PORT_SPIRIT_PLC);
    schedByte |= SCHEDBYTE_SPIRITBACKGROUND;
    schedByte |= SCHEDBYTE_CM2COREBACKGROUND;
    schedByte |= SCHEDBYTE_CM2PORTS;
  }
}



