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

#define SPIRIT_INTERRUPT_PIN GPIO_Pin_7
#define SPIRIT_SHUTDOWN_PORT GPIOB
#define SPIRIT_SHUTDOWN_PIN GPIO_Pin_6

typedef enum {STATUS_NONE=0,STATUS_WAITING,STATUS_FAIL,STATUS_SUCCESS};

#define MAX_VERSION_LENGTH			16    
typedef struct
{
	uint32_t checksum;
	uint32_t appLength;
	uint32_t checksumStartOffset;
 	uint32_t	secondaryChecksumAddr;       
	uint32_t	version;
	uint8_t		verString[MAX_VERSION_LENGTH];
	uint8_t		verLength;
} AppInfo;
AppInfo boot; 

uint32_t myLocalID; 
 
CM2DIAG cm2Diag; 

bool newFirmware = TRUE;
uint16_t newFirmwareTimer; 
extern CMCI oMCI[MC_NUM];
extern uint16_t newRPM; 

void AckResult(uint16_t mac, bool success);

//uint16_t cm2TotalSent; 
//uint16_t cm2TotalNack; 
uint8_t cm2Device = 0x10;
//uint16_t cm2SendCount; 
//uint16_t cm2ReceiveCountFail; 
static uint16_t pingCount = 0;
//static uint8_t pingCmd[3] = {OP_PING,0,0};
static uint16_t lastUpdateMac = 0;
static uint8_t lastUpdatePacketStatus = STATUS_NONE;

//----- Address defines for Micro
#define DEVICE_TYPE 0x83
#define DEVICETX_TYPE 0x39
//#define MY_ID 0x32103211   //AND 0 at end  /////////////////////////testing only
#define TEST_NETWORK_ID 0xFEDCBA98 //////////////////////////////

uint32_t spiritMYID; 

//I think I have #1 working in my new spirit driver now. 
//It repeatedly goes into RX mode, aborts, read the RSSI, 
//then returns to RX mode. In the live watch window I can 
//see the value jump up when I turn on a continuous wave 
//transmit remote near it, and I can see it move up and 
//down as I move the transmitter closer and farther away. 
//In the current driver it would be something like:
/*
                SpiritCmdStrobeRx();
                delay or wait for state to be MC_STATE_RX
                SpiritCmdStrobeSabort();
                delay or wait for state to be MC_STATE_READY
                rssi = SpiritQiGetRssi();
*/


uint32_t tableContents[10]= {

  0x00229EFF,0x001E3282,0x001D9D6C,0x001DDBF4,
  0x001D9D21,0x001E293D,0x00226D25,0x001DD393,
  0xffffffff,0xffffffff};

/*
---USE THE BELOW ID FOR THE SHAKER TABLE

uint32_t tableContents[10]= {  
    0x001E14E0,0x001E14E0,0x001E14E0,0x001E14E0,
    0x001E14E0,0x001E14E0,0x001E14E0,0x001E14E0,
    0xFFFFFFFF,0xFFFFFFFF};
*/

uint16_t dupTimer; 
uint8_t allpacket[30]; 
uint8_t alloffset = 0; 
uint8_t receivedpacket[30]; 
uint8_t receivedoffset = 0; 
uint16_t lastMAC = 0; 
extern uint8_t   remote_type;
uint8_t remoteDownloading = 0; 

uint8_t decodedData[8]; 
extern uint8_t echoData[10]; 

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  CM2SendPeriodValues
//------------------------------------------------------------------------------
//  
//==============================================================================
bool CM2SendTimingAdjustTable(void)
{
  uint8_t i,*ptr; 
  int16_t itemp; 
  uint16_t uitemp; 
  
  AbortAllRetries();
  
  CM2Packet pairPacket = 
  {
    PORT_INTERNAL,    //packet source
    PORT_SPIRIT,      //destination port
    0,             //retries
    PROTOCOL_VERSION, //protocol ver
    DEVICETX_TYPE,      //src type
    0xFF,             //dest type
    myLocalID,        //src ID
    0xFFFFFFFF,       //dest ID
    0x00,             //subaddress
    6+2,                //length
    0x0000,           //MAC to be filled out
    {OP_TABLE_DATA,0xff,0x11,0x00,0x01,0x01},        //command/payload
  };
  itemp = GetTimingAdjust();
  uitemp = itemp;
  uitemp = uitemp>>8; 
  pairPacket.command[6] = uitemp; 
  uitemp = itemp;
  uitemp = uitemp&0xff; 
  pairPacket.command[7] = uitemp;   
  
  pairPacket.mac = CalcMAC(&pairPacket, myLocalID);
  return RoutePacket(&pairPacket);
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  CM2SendPeriodValues
//------------------------------------------------------------------------------
//  
//==============================================================================
bool CM2SendStatusTable(uint8_t *ptr)
{
  uint8_t i; 
   
  AbortAllRetries();
  
  CM2Packet pairPacket = 
  {
    PORT_INTERNAL,    //packet source
    PORT_SPIRIT,      //destination port
    0,             //retries
    PROTOCOL_VERSION, //protocol ver
    DEVICETX_TYPE,      //src type
    0xFF,             //dest type
    myLocalID,        //src ID
    0xFFFFFFFF,       //dest ID
    0x00,             //subaddress
    6+28,                //length
    0x0000,           //MAC to be filled out
    {OP_TABLE_DATA,0xff,0x10,0x00,0x01,0x01},        //command/payload
  };
  for (i=0;i<28;i++)
  {
    pairPacket.command[6+i] = *ptr++;
  }
   
 
  pairPacket.mac = CalcMAC(&pairPacket, myLocalID);
  return RoutePacket(&pairPacket);
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  
//------------------------------------------------------------------------------
//  
//==============================================================================
bool CM2SendPairRequest(void)
{
  //abort retries for last packet sent
  AbortAllRetries();
  
  CM2Packet pairPacket = 
  {
    PORT_INTERNAL,    //packet source
    PORT_SPIRIT,      //destination port
    0xFF,             //retries
    PROTOCOL_VERSION, //protocol ver
    DEVICE_TYPE,      //src type
    0xFF,             //dest type
    myLocalID,        //src ID
    0xFFFFFFFF,       //dest ID
    0x00,             //subaddress
    1,                //length
    0x0000,           //MAC to be filled out
    {OP_PAIR},        //command/payload
  };
  pairPacket.mac = CalcMAC(&pairPacket, myLocalID);
  return RoutePacket(&pairPacket);
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//  pingPacketHydro
//==============================================================================
static CM2Packet pingPacketHydro = 
{
                PORT_INTERNAL,      //packet source
                PORT_SPIRIT,        //destination SPIRIT
		0xFF,
                PROTOCOL_VERSION,   //protocol ver
                DEVICE_TYPE,        //src type
                0x80,  //0x10,      //dest type
                0,                                                                                            //src ID
                0xFFFFFFFF,         //dest ID
                0x00,               //subaddress
                0x03,               //length
                0x0000,             //MAC to be filled out before acking
		{OP_PING,0,0},      //command/payload
};
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//  genericDoCmd
//==============================================================================
static CM2Packet genericDoCmd = 
{
	PORT_INTERNAL,			//packet source
	PORT_SPIRIT,			//destination SPIRIT
	3,  //12,//0xFF,
	PROTOCOL_VERSION,		//protocol ver
	DEVICE_TYPE,			//src type
	0x80,					//dest type - anchors
	0,						//src ID
	0xFFFFFFFF,				//dest ID
	0x00,					//subaddress
	0x04,					//length
	0x0000,					//MAC to be filled out before acking
	{OP_DO,0xFF,0xFF,0xFF}					//command/payload
};
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//  pingPacketRemote
//==============================================================================
static CM2Packet pingPacketRemote = 
{
                PORT_INTERNAL,                                             //packet source
                PORT_SPIRIT,                                    //destination SPIRIT
				0xFF,
                PROTOCOL_VERSION,                   //protocol ver
                DEVICE_TYPE,                                    //src type
                0x10,                                                                     //dest type
				0,                                                                                            //src ID
                0xFFFFFFFF,                                                       //dest ID
                0x00,                                                                     //subaddress
                0x03,                                                                     //length
                0x0000,                                                                  //MAC to be filled out before acking
				{OP_PING,0,0}                                                           //command/payload
};
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//  callbackPacket
//============================================================================== 
//request the remote to call back when idle
/* CM2Packet callbackPacket = 
{
                PORT_INTERNAL,                                             //packet source
                PORT_SPIRIT,                                    //destination port
                0xFF,                                                                     //retries
                PROTOCOL_VERSION,                   //protocol ver
                DEVICE_TYPE,                                    //src type
                0,                                                                                            //dest type anchors
                spiritMYID,                                 //src ID
                packet->srcID,                                   //dest ID
                0x00,                                                                     //subaddress
                2,                                                                                            //length
                0x0000,                                                                 //MAC to be filled out
                {OP_REQUEST,OP_IDLE_CALLBACK},      //command/payload
};
*/
//callbackPacket.mac = CalcMAC(&callbackPacket, spiritMYID);
//RoutePacket(&callbackPacket);

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
  
  //---------------------------
  // disable accelerometer CS D.2
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
//  GPIO_Init(GPIOD, &GPIO_InitStructure);
//  GPIO_WriteBit(GPIOD,GPIO_Pin_2,Bit_SET);

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


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  
//------------------------------------------------------------------------------
//  
//==============================================================================
void AppSpiritInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  uint32_t ltemp1,ltemp2;
  //--------------------------------
  // initialize the ID for spirit packets 
  // comes from the ID off the eeprom CHIP. 
  //---------------------------------
/*  
  ltemp1 = table0.Item.EEPROMID[0];
  ltemp1 = ltemp1<<24; 
  ltemp2 = table0.Item.EEPROMID[1];
  ltemp2 = ltemp2<<16; 
  ltemp1 |= ltemp2; 
  ltemp2 = table0.Item.EEPROMID[2];
  ltemp2 = ltemp2<<8; 
  ltemp1 |= ltemp2; 
  ltemp2 = table0.Item.EEPROMID[3];
  ltemp1 |= ltemp2; 
*/
  ltemp1 = 0x001C8899;
  spiritMYID = ltemp1; 
  myLocalID = spiritMYID; 
  
  testSpiritMainMini(); 
  CM2CoreInit();
#if 0
    GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_SET);    
#endif   	
//////TODO - pull pairing table from flash to pass to CM2CoreConfigure

  CM2CoreConfigure(spiritMYID, DEVICE_TYPE, TEST_NETWORK_ID, tableContents, 8, 10);
#if 0  
    GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_SET);    
#endif   

  RegisterAckResultCallback(AckResult);
	
  __enable_interrupt();	
  
  uint8_t ports[] = {1};
  SpiritInit(1, ports, spiritMYID, DEVICE_TYPE);
/*
  SpiritSetCwTx(*ports,TRUE);
  while (1)
  {
#if WDOGON  
    //----------------------
    // toggle the wdog
    //----------------------
    IWDG_ReloadCounter();
#endif
      
  }
 //test
*/  
  EXTI_ClearFlag(EXTI_Line7);   //M2S_GPIO_0_EXTI_LINE);
  EXTI_ClearITPendingBit(EXTI_Line7);  //M2S_GPIO_0_EXTI_LINE);      
  //--------------------------------------
  // enable the GPIO0 interrupt for the spirit radio
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//DISABLE;
  NVIC_Init(&NVIC_InitStructure);     
}
 


#if 0
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  HandleCM2Pair
//------------------------------------------------------------------------------
//  
//==============================================================================
void HandleCM2Pair(CM2Packet* packet )
{
  uint8_t idData[4],addr; 
  uint32_t ltemp,ltemp2; 
  
     if (packet->srcID!=0)
      {  
        ltemp = packet->srcID;
        ltemp2 = ltemp>>24; 
        ltemp2 &= 0x00ff;
        idData[0] = ltemp2;       
        
        ltemp2 = ltemp>>16; 
        ltemp2 &= 0x00ff;
        idData[1] = ltemp2; 
        ltemp2 = ltemp>>8; 
        ltemp2 &= 0x00ff;
        idData[2] = ltemp2;   
        ltemp2 = ltemp; 
        ltemp2 &= 0x00ff;
        idData[3] = ltemp2;     
        //-----------------------------------
        //read first address to write to
        __disable_interrupt();    
//        I2C_EE_BufferRead( &addr, PairedRemoteTableOffset_Setting, 1);
        addr = ReadSetting(PairedRemoteTableOffset_Setting,FALSE);  
        __enable_interrupt();    
        //------------------------------------
        //set first address to write to if it hasn't been set yet
        if (addr > 9)
        {
          addr = 0x0000;
          __disable_interrupt();     
//          I2C_EE_BufferWrite( &addr, PairedRemoteTableOffset_Setting, 1);
          WriteSetting(PairedRemoteTableOffset_Setting,addr,FALSE);          
          __enable_interrupt();    
        }
        __disable_interrupt();       
//         I2C_EE_BufferWrite( &idData[0],(addr*5)+ PairedRemoteID_Setting+0,1); 
//         I2C_EE_BufferWrite( &idData[1],(addr*5)+ PairedRemoteID_Setting+1,1);
//         I2C_EE_BufferWrite( &idData[2], (addr*5)+ PairedRemoteID_Setting+2,1);      
//         I2C_EE_BufferWrite( &idData[3], (addr*5)+ PairedRemoteID_Setting+3,1);  
         WriteSetting((addr*5)+PairedRemoteID_Setting+0,idData[0],FALSE);  
         WriteSetting((addr*5)+PairedRemoteID_Setting+1,idData[1],FALSE);  
         WriteSetting((addr*5)+PairedRemoteID_Setting+2,idData[2],FALSE);  
         WriteSetting((addr*5)+PairedRemoteID_Setting+3,idData[3],FALSE);           
        // you’ll get 0x11 for a 2-button fob, 0x13 for a dash, and 0x14 for a 1-button foot.
        // default is dash
        idData[0] = 0x00;  
        switch (packet->srcType)
        {
          case 0x11:
          {
            //-----------2 button keyfob w/o accel
            idData[0] = 0x01; 
            break;
          }
          case 0x14:
          {
            idData[0] = 0x06; 
            break;
          }
        }       
//        I2C_EE_BufferWrite( &idData[0],(addr*5)+ PairedRemoteID_Setting+4,1); 
        WriteSetting((addr*5)+ PairedRemoteID_Setting+4,idData[0],FALSE);  
     
        __enable_interrupt();    
        //-------------------
        //go to next address
        addr += 1;
        //-------------------------------
        //wrap address
        if (addr > 9)
          addr = 0;
        //----------------------------------
        //update next address to write to
        __disable_interrupt();   
//        I2C_EE_BufferWrite( &addr, PairedRemoteTableOffset_Setting, 1);
         WriteSetting(PairedRemoteTableOffset_Setting,addr,FALSE); 
        __enable_interrupt();    
        PairUpdateList(TRUE);
        PlayTune(TUNE_PROGRAM_ENTER);      
        timerPairSession = 0; 
        timerPairSessionSpecial = 0; 
      }  
}  
#endif 

#define MAX_APPLICATION_SPEED           6500  //JDO change max speed if you want 
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
  if (newspd > MAX_APPLICATION_SPEED)
  {
    newspd  = MAX_APPLICATION_SPEED; 
  }
  spd = newspd/6; 
  return spd;
}
 


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  HandleDoCommand
//------------------------------------------------------------------------------
//  
//==============================================================================
void HandleDoCommand(CM2Packet* packet)
{
  uint8_t status,speed,force; 
  uint8_t packetInstance,idData[3]; 
  int direction; 
  uint32_t ltemp,ltemp2; 
  
  speed = packet->command[1];
  force = packet->command[2];
  packetInstance = packet->subaddress | 0x30;
  direction = 0x7f & packet->command[3];	
  if (((packet->command[3]) >> 7) > 0)
                  direction *= -1;  

  //-------------------------
  
    GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);  
  if (alloffset >= 30)
  {
    alloffset = 0; 
  }  
  allpacket[alloffset] = direction; 
  alloffset++;   
  if (dupTimer == 0)
  {
    lastMAC = 0; 
  }
  //--------------------------
  // remove duplicates 
  if (packet->mac != lastMAC)
  { 
 
    lastMAC = packet->mac; 
    dupTimer = 500;  //DELAY_300MSEC;  
    
    if (receivedoffset >= 30)
    {
      receivedoffset = 0; 
    }  
    receivedpacket[receivedoffset] = direction; 
    receivedoffset++; 
 
//    if (timerPairSession ==0) 
//    {      
//      if (GetFirmwareUpdateStatus() != UPDATE_STATUS_IN_PROGRESS)
//      {    
//        if ((packetInstance == 0xFF) || (packetInstance == table0.Item.Instance) ||
//                        (table0.Item.Instance == BOTH))
//        {
        //---------------- some adjustments to the direction received. 

        //------------ some adjustment to speed numbers received
        // CM1 had speed of 1-100 so will half the values received from CM2
          if (speed > 200)
          {
            speed = 200;
          }
          if (speed == 1)
          {
            speed = 2; 
          }
          speed = speed/2; 
     
          if (speed == 0 || force == 0 || direction == 0)
          {
//            ProcessCommandStopMotor(INTERFACE_RF);
          }
          else
          {
          //--------------------------------
          // remote type speed adjustments. 
            idData[0] = 0x01;  
            switch (packet->srcType)
            {
              case 0x11:
              {
                //-----------2 button keyfob w/o accel
                idData[0] = 0x02; 
                break;
              }
              case 0x14:
              {
                idData[0] = 0x07; 
                break;
              }
            }   
            
//The keyfob commands for testing advenger
//
//•        Manual up (any speed) = increase RPM +100
//•        Manual Down (any speed) = decrease RPM -100
//•        Auto up (any speed) turns motor on
//•        Auto down (any speed) turns motor off
            
//            remote_type = idData[0]; 
            if (direction > 0 && direction <= 100)
            {
              //--------------------
              // up manual
              //-------------------- 
//              status = MotorStartManual(0, UP,speed,INTERFACE_RF);
              newRPM+=100;
              if (newRPM > MAX_APPLICATION_SPEED)
              {
                newRPM = MAX_APPLICATION_SPEED;
              }
              if (motorOn != 0)
              {
                MCI_ExecSpeedRamp(oMCI[0],-(SetSpeed(newRPM)),0);   
                MCI_StartMotor(oMCI[0]); 
              }  
            }
            else if (direction < 0 && direction >= -100)
            {
              //--down manual
//            status = MotorStartManual(0, DOWN,speed,INTERFACE_RF);
              
              if (newRPM > 100)
              {
                newRPM = newRPM - 100;
              }              
              else
              {
                newRPM = 0; 
              }
              if (motorOn != 0)
              {
              MCI_ExecSpeedRamp(oMCI[0],-(SetSpeed(newRPM)),0);   
              MCI_StartMotor(oMCI[0]);  
              }  
            }
           else if (direction == -110)
           {
             //TOGGLE MOTOR START/STOP 
              if (motorOn == 0)
              {
                if (newRPM > MAX_APPLICATION_SPEED)
                {
                  newRPM = MAX_APPLICATION_SPEED;
                }
                MCI_ExecSpeedRamp(oMCI[0],-(SetSpeed(newRPM)),0);   
                MCI_StartMotor(oMCI[0]);                               
                motorOn = 1; 
              }
              else
              {
                motorOn = 0;
                MCI_StopMotor(oMCI[0]);                 
              } 
           }
          }
  }  
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  
//------------------------------------------------------------------------------
//  
//==============================================================================
bool PacketRX(CM2Packet* packet)
{       
  uint32_t remoteVersion;
  bool packetHandled = FALSE;
  uint8_t idData[3];
  uint32_t ltemp,ltemp2; 	
  
  // Check if the app cares about this packet, 
  // if so then handle it here
  if (packet != 0)
  {
    cm2Diag.MessageCount++;
    switch(packet->command[0])
    { 
      case OP_COMM:
      {            
        packetHandled = TRUE;
 	if (packet->command[1] == 0x22)
        {
          StartTimingAdjust(1);
        }        
        break;
      }
      case OP_PAIR:
      {                      
	packetHandled = TRUE;
/* JOHNO if you want this in OP PAIR uncomment this section        
 	if (packet->command[1] == 0x22)
        {
          StartTimingAdjust();
        }                
BUT YOU WILL NEED TO COMMENT THE NVIC_SystemReset();
*/         
        NVIC_SystemReset();          

        break;
      }    
      case OP_REQUEST:
      {  
        //what is being requested?
        switch(packet->command[1])
        {
          case OP_VERSION:
          {
            //return the software version
            CM2Packet verPacket = 
            {
              PORT_INTERNAL,        //packet source
              packet->deviceSource, //destination port
              0xFF,                 //retries
              PROTOCOL_VERSION,     //protocol ver
              DEVICE_TYPE,          //src type
              0,                    //dest type
              myLocalID,             //src ID
              packet->srcID,        //dest ID
              0x00,                 //subaddress
              17,                   //length
              0x0000,               //MAC to be filled out
              {OP_VERSION,FW_VER>>24,FW_VER>>16,FW_VER>>8,(uint8_t)FW_VER,
              (uint8_t)(boot.version>>24),(uint8_t)(boot.version>>16),(uint8_t)(boot.version>>8),
              (uint8_t)boot.version,
              0},    //command/payload
            };
            verPacket.mac = CalcMAC(&verPacket,myLocalID);
            RoutePacket(&verPacket);
            packetHandled = TRUE;
            break;
          }
          default:
            break;
        }
        break;
      }      
      case OP_DO:
      {     
        //----below added to make sure not in adjustment and a DO command has 
        // been received. 
        if (IsTimingAdjustActive()==0)
        {
          HandleDoCommand(packet);
        }  
	packetHandled = TRUE;
/*        
        if (timerPairSession ==0)
        {  
          //check if this is a remote
          if ((packet->srcType & 0xF0) == 0x10)
          {  
            //-------------------------------
            // check which offset the srcID is in the 
            // pairlist to get the offset in the 
            //    rfGetVersions list to see if it is 
            //    time to request a version from this address. 
            //--------------------------------
            if (PairGetRemoteVersion(packet->srcID)!= 0)
            {
		//request the software version
		CM2Packet verPacket = 
		{
			PORT_INTERNAL,			//packet source
			PORT_SPIRIT,			//destination port
			0xFF,					//retries
			PROTOCOL_VERSION,		//protocol ver
			DEVICE_TYPE,			//src type
			0,						//dest type anchors
			spiritMYID,					//src ID
			packet->srcID,			//dest ID
			0x00,					//subaddress
			2,						//length
			0x0000,					//MAC to be filled out
			{OP_REQUEST,OP_VERSION},	//command/payload
		};
		verPacket.mac = CalcMAC(&verPacket, spiritMYID);
		RoutePacket(&verPacket);
                newFirmware = FALSE;
            }
          }
        }  
*/        
        break;
      }  
      case OP_VERSION:
      {
/*        
	if (((packet->srcType & 0xF0) == 0x10)&&(packet->srcID != 0x55555555))
	{
          //--------------------------
          // UPDATE version in table
          //--------------------------
          // 1. build version number 
          // 2. compare src addresses to pair table entry for offsets. 
          // 3. update matching src with version number. 
          remoteVersion = (packet->command[1]<<16 | packet->command[2]<<8 | packet->command[3]);
          PairUpdateRemoteVersion(remoteVersion,packet->srcID);
#if FCCVERSION

#else                                    
          if (remoteVersion < (newRemoteVersion>>8))  
          {
              if (remoteVersion > 23)
              {  
                CM2Packet callbackPacket = 
                {
                  PORT_INTERNAL,          //packet source
                  PORT_SPIRIT,            //destination port
                  0xFF,                   //retries
                  PROTOCOL_VERSION,       //protocol ver
                  DEVICE_TYPE,            //src type
                  0,                                                                                            //dest type anchors
                  spiritMYID,             //src ID
                  packet->srcID,          //dest ID
                  0x00,                   //subaddress
                  2,                                                                                        //length
                  0x0000,                 //MAC to be filled out
                  {OP_REQUEST,OP_IDLE_CALLBACK},      //command/payload
                };
                callbackPacket.mac = CalcMAC(&callbackPacket, spiritMYID);
                RoutePacket(&callbackPacket);            
              }
              else
              {
		newFirmware = FALSE;
		//start firmware update
                ProcessCommandStopMotor(INTERFACE_RF);
                BeginFirmwareUpdate(packet->srcID,0x08043000,remoteCode.appLength);
              }  
          }
#endif           
	}
*/        
	break;
      }
      case OP_IDLE_CALLBACK:
      {
/*        
	if (((packet->srcType & 0xF0) == 0x10)&&(packet->srcID != 0x55555555))
	{
          remoteVersion = PairCurrentRemoteVersion(packet->srcID);
#if FCCVERSION

#else                                    
          if (remoteVersion < (newRemoteVersion>>8))  
          {
		newFirmware = FALSE;
		//start firmware update
                BeginFirmwareUpdate(packet->srcID,0x08043000,remoteCode.appLength);  
          }
#endif           
	}
*/        
	break;
      }      
      
      
    }
  }
   return packetHandled;
}



//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  
//------------------------------------------------------------------------------
//  
//==============================================================================
void SendPing(void)
{
  switch (cm2Device)
  {
    case 0x80:
    {
                //send latest ping and update count (ping count ensures that the MAC is always different)
                pingPacketHydro.command[1] = pingCount>>8;
                pingPacketHydro.command[2] = pingCount;
                pingCount++;
                pingPacketHydro.mac = CalcMAC(&pingPacketHydro, spiritMYID);
                CM2Packet* copy = CopyPacket(&pingPacketHydro);
                if (!RoutePacket(copy))
                {
                                //failed to route the packet, so be sure to free the memory used by the copy
 //                               FreeMem((void**)&(copy->command));
 //                               FreeMem((void**)&copy);
                }
     break;
    }
  default: 
    {
                //send latest ping and update count (ping count ensures that the MAC is always different)
                pingPacketRemote.command[1] = pingCount>>8;
                pingPacketRemote.command[2] = pingCount;
                pingCount++;
                pingPacketRemote.mac = CalcMAC(&pingPacketRemote, spiritMYID);
                CM2Packet* copy = CopyPacket(&pingPacketRemote);
                if (!RoutePacket(copy))
                {
                                //failed to route the packet, so be sure to free the memory used by the copy
//                                FreeMem((void**)&(copy->command));
//                                FreeMem((void**)&copy);
                }
     break;
    }  
  }
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  
//------------------------------------------------------------------------------
//  
//==============================================================================
static void AckResult(uint16_t mac, bool success)
{
	//do what you want with the finding of loss of ack. The mac parameter is 
	//the mac of the packet that was acked or timed out.
  
//	UpdateAckResult(mac,success);
  
	if (success)
	{
		/*if (mac == lastUpdateMac)
			lastUpdatePacketStatus = STATUS_SUCCESS;*/
	}
	else
	{
		/*if (mac == lastUpdateMac)
			lastUpdatePacketStatus = STATUS_FAIL;*/
	}
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
void SpiritReset(void)
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
  NVIC_InitTypeDef NVIC_InitStructure;
//  GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_SET);
 
  //------------------------------
  // Check the flag status of EXTI line 
  //------------------------------
  if(EXTI_GetFlagStatus(EXTI_Line7))   
  {
    EXTI_ClearFlag(EXTI_Line7);   
    EXTI_ClearITPendingBit(EXTI_Line7);      
//    schedByte |= SCHEDBYTE_SPIRIT_ISR;  
//EMH  06-16-16       
//    SPIRIT_ISR();
///    SpiritBackgroundTasks();
	HandleSpiritInterrupt(1);
  
    //----------------------
    // toggle the wdog
    //----------------------
#if WDOGON    
//    IWDG_ReloadCounter();
#endif    
    //------------------------------------
    // Clear the EXTI line flag 
    //------------------------------------
 //   EXTI_ClearFlag(EXTI_Line5);   
 //   EXTI_ClearITPendingBit(EXTI_Line5);  
  } 
#if 0  
  if (rssiStream == TRUE)
  {
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);         
  }
#endif   
//  GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_RESET);
}

#if FCCVERSION
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  SendDoCmd
//------------------------------------------------------------------------------
//  
//==============================================================================
bool FCCSendDoCmd(void) 
{
                //abort retries for last packet sent
                AbortRetries(genericDoCmd.mac);
  
                genericDoCmd.subaddress = 0x30;
                genericDoCmd.command[1] = 0x66;
                genericDoCmd.command[2] = 0xFF;
                genericDoCmd.command[3] = 0x55;
                genericDoCmd.mac = CalcMAC(&genericDoCmd, spiritMYID);
                genericDoCmd.retries = 0;
                if (RoutePacket(&genericDoCmd))
                                return TRUE;
   
                return FALSE;
}
#endif 
