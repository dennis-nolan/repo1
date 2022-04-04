//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: appTestSupport.c
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Processor: STM32
// TOOLS: IAR Workbench 
// DATE:
// CONTENTS: This file contains  
//------------------------------------------------------------------------------
// HISTORY: This file  
//------------------------------------------------------------------------------
// VERSION  DATE     PERSON   DESCRIPTION
//  00.01   03/28/19  EMH   FIRST release. integrated into EA103 and EA095
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#include "defs.h"
#include "cm2routingHeader.h"
#include "threadingheader.h" 
#include "timersheader.h"
#include "SpiritInterfaceheader.h" 
#include "appTestSupport.h"
#include "JL_SpiritDriver.h"
#include "cm2PairingTableModeHeader.h"
#include "cm2PortsHeader.h"
#include "ProductionInfo.h"

//---------------------GLOBAL DEFINITIONS--------------------------------
uint8_t appTestEvent;

//---------------------LOCAL VARIABLES------------------------------------
#define TESTDELAY_300MSEC   300
#define MAX_TEST_DUP_ENTRIES 5
uint8_t testDupOffset = 0; 
uint16_t testDupTimers[MAX_TEST_DUP_ENTRIES];
uint32_t testDupMACs[MAX_TEST_DUP_ENTRIES];
 
uint16_t testMsTimer=0; 
uint8_t testPingTimer = 0; 
uint8_t testSpiritTimer = 0;

uint16_t testPingMasterOffset; 
uint32_t testPingMasterID;
uint8_t testPingMasterDestType;
uint8_t testPingMasterInterval;
uint16_t testPingMasterGoodAck;
uint16_t testPingMasterCount;
uint16_t testPingMasterOffsetSent;

#define MAX_PING_MAC_OFFSET     3
uint16_t testPingMasterMACQueue[MAX_PING_MAC_OFFSET];
uint16_t testPingMasterMACOffset=0; 
uint8_t testCWTimer; 

uint16_t ackCount; 

#define TESTSTATE_IDLE          0
#define TESTSTATE_PINGMASTER    1
#define TESTSTATE_CW            2
uint8_t testState = TESTSTATE_IDLE; 


OpTest opTest; 
//---------------------LOCAL FUNCTION PROTOTYPES--------------------------  
RxPacketResult AppTestPacketRX(CM2Packet* packet); 

  CM2Packet GeneralPacket = 
  {
    PORT_INTERNAL,    //packet source
    PORT_SPIRIT,      //destination port
    0xff,             //retries
    PROTOCOL_VERSION, //protocol ver
    DEVICE_TYPE,      //src type
    0xff,             //dest type
    0x00000000,        //src ID
    0xffffffff,       //dest ID
    0x00,             //subaddress
    1,                //length
    0x0000,           //MAC to be filled out
    {0x16},        //command/payload
  };


//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// ---------------------------GLOBAL FUNCTIONS ----------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppTestSendOPTESTResult
//------------------------------------------------------------------------------
//  
//==============================================================================
bool AppTestSendOPTESTResult(CM2Packet *ptr,uint8_t broadcast,OpTest *opTester)
{
  
  CM2Packet PINGPacket = 
  {
    PORT_INTERNAL,    //packet source
    PORT_SPIRIT,      //destination port
    0xff,             //retries
    PROTOCOL_VERSION, //protocol ver
    DEVICE_TYPE,      //src type
    0xff,             //dest type
    GetID(),        //src ID
    0xffffffff,       //dest ID
    0x00,             //subaddress
    15,                //length
    0x0000,           //MAC to be filled out
    {0x16},        //command/payload
  };
  if (broadcast == 0)
  {
    //direct 
    PINGPacket.destID = ptr->srcID;
    PINGPacket.destType = 0x00;
  }
  if (ptr == 0)
  {
      PINGPacket.deviceDest = opTest.Item.DeviceSource; 
  }
  else
  {
    PINGPacket.deviceDest = ptr->deviceSource;
  }
  
  PINGPacket.command[1] = opTester->Item.TestOperation;
  PINGPacket.command[2] = opTester->Item.SequenceNumber;  
  PINGPacket.command[3] = opTester->Item.TxPower;
  PINGPacket.command[4] = opTester->Item.Frequency;    
  PINGPacket.command[5] = opTester->Item.PingMasterPacketCount[0];
  PINGPacket.command[6] = opTester->Item.PingMasterPacketCount[1];    
  PINGPacket.command[7] = opTester->Item.PingTestResult[0];
  PINGPacket.command[8] = opTester->Item.PingTestResult[1];  
  
  PINGPacket.mac = CalcMAC(&PINGPacket, GetID());
  return RoutePacket(&PINGPacket);
}
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppTestSendPing
//------------------------------------------------------------------------------
//  
//==============================================================================
uint16_t AppTestSendPing(uint32_t ID,uint16_t offset,uint8_t destType)
{
  uint16_t temp;
  bool status; 
  CM2Packet PINGPacket = 
  {
    PORT_INTERNAL,    //packet source
    PORT_SPIRIT,      //destination port
    0x00,             //retries
    PROTOCOL_VERSION, //protocol ver
    DEVICE_TYPE,      //src type
    destType,             //dest type
    GetID(),        //src ID
    ID,       //dest ID
    0x00,             //subaddress
    3,                //length
    0x0000,           //MAC to be filled out
    {OP_PING},        //command/payload
  };
  PINGPacket.deviceDest = opTest.Item.DeviceSource;
  temp = offset>>8;
  PINGPacket.command[1] = temp;
  temp = offset & 0xff;
  PINGPacket.command[2] = temp;  
  PINGPacket.mac = CalcMAC(&PINGPacket, GetID());
  status = RoutePacket(&PINGPacket);
  return PINGPacket.mac;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppTestTask
//------------------------------------------------------------------------------
//  must be called every 1msec for timing requirements
//==============================================================================
void AppTestTask(void)
{
  uint16_t mac,itemp; 
  uint8_t i; 
  
  if (appTestEvent!= 0)
  {
    appTestEvent = 0;
    
    //------------------------
    // handle testing timers. 
    // 1. CW - second timer
    // 2. ping - msec timer 
    // 3. spirit driver timer 
    //=========================
    testMsTimer++;
    if (testMsTimer >= 1000)
    {
      testMsTimer = 0;
      if (testCWTimer > 0)
      {
        testCWTimer--;
      }
    }
    if (testPingTimer >0)
    {
      testPingTimer--;
    }
    //----------------------
    // Update DUP timers 
    for (i=0;i<MAX_TEST_DUP_ENTRIES;i++)
    {
      if (testDupTimers[i] > 0)
      {
        testDupTimers[i] = testDupTimers[i] - 1; 
      }
    }      
//    testSpiritTimer++;
//    if (testSpiritTimer >3)
//    {
      testSpiritTimer = 0;
      RunSpiritBackgroundTasks(); 
      RunCM2PortsTasks();
      RunCM2CoreBackgroundTasks();        
      RunTimerBackgroundTasks();  
//    }
    switch(testState)
    {
      case TESTSTATE_IDLE:
      {
        break;
      }
      case TESTSTATE_PINGMASTER:
      {
        //--------------------------
        // check if interval time done
        //--- if not, nothing to do 
        if (testPingTimer == 0)
        {
          //-------------------------------
          // if time up, then see if need to send another packet 
          // ... if not more packets ... DONE. 
          // stay in this state, so you are ready to respond 
          // to result message or user forces you to IDLE/ANOTHER TEST ... 
          // ..... IF they didn't get your result before swithcing you out, their loss 
          //----------------------------------
          if (testPingMasterOffset < testPingMasterCount)
          { 
            testPingTimer = testPingMasterInterval;
            //-------------- send ping 
            mac = AppTestSendPing(testPingMasterID,testPingMasterOffset,testPingMasterDestType);
            testPingMasterOffsetSent = testPingMasterOffset;
            testPingMasterOffset++;
            if (testPingMasterMACOffset >= MAX_PING_MAC_OFFSET)
            {
              testPingMasterMACOffset = 0;
            }
            testPingMasterMACQueue[testPingMasterMACOffset++] = mac;      
          }
          else
          {
            if (testPingMasterOffset == testPingMasterCount)
            {
              //---------------------
              // put in result 
              itemp = testPingMasterGoodAck;
              itemp = itemp>>8;
              opTest.Item.PingTestResult[0] = itemp;
              itemp = testPingMasterGoodAck & 0xff;
              opTest.Item.PingTestResult[1] = itemp;     
              GeneralPacket.deviceDest = opTest.Item.DeviceSource;
              AppTestSendOPTESTResult(0,1,&opTest);
              testPingMasterOffset++;  
              RegisterAckResultCallback(0);
            }    
          }
        }
        break;
      }
      case TESTSTATE_CW:
      {
        //--------------------------
        // check if CW time is up
        if ((testCWTimer == 0)&& (opTest.Item.CWTimeout != 0))
        {      
          AppSpiritInit(); 
          switch(myflashConfigSection.Item.appMode)
          {
            case APPTEST_EUCM2TASKSONLY_ID0:
            {
              SetSpiritFrequency(RADIO1_TYPE,SPIRIT_FREQ_EU_868_3);
              break;
            }
#if  (SPIRIT_VER >= 0x00000008)            
            case APPTEST_9275CM2TASKSONLY_ID0:
            {
              SetSpiritFrequency(RADIO1_TYPE,SPIRIT_FREQ_PLC_927_5);
              break;
            }         
#endif            
            case APPTEST_92275CM2TASKSONLY_ID0:
            {
              SetSpiritFrequency(RADIO1_TYPE,SPIRIT_FREQ_922_75);
              break;
            }            
            case APPTEST_92125CM2TASKSONLY_ID0:
            {
              SetSpiritFrequency(RADIO1_TYPE,SPIRIT_FREQ_921_25);
              break;
            }            
            case APPTEST_902CM2TASKSONLY_ID0:
            {
              SetSpiritFrequency(RADIO1_TYPE,SPIRIT_FREQ_PLC_902);
              break;
            }                        
            default:
            {
              SetSpiritFrequency(RADIO1_TYPE,RADIO1_FREQUENCY);    
              break;
            }  
          }  
          SetSpiritFrequency(RADIO2_TYPE,RADIO2_FREQUENCY);
          RegisterPacketRxCallback(AppTestPacketRX);
          testState = TESTSTATE_IDLE; 
        }
        break;
      }
    }
  }  
}  


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppTestInit
//------------------------------------------------------------------------------
//  
//==============================================================================
void AppTestInit(void)
{
  testState = TESTSTATE_IDLE;
  testMsTimer=0; 
  testPingTimer=0; 
  testCWTimer = 0; 
}  
 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION: TestingAckCallback 
//------------------------------------------------------------------------------
//  
//==============================================================================
void TestingAckCallback(uint16_t mac, bool success)
{
  uint8_t i,done; 
  
  if ((success == TRUE))
  {
    done = 0;
    i = 0;
    while (i<MAX_PING_MAC_OFFSET) 
    {
      if (testPingMasterMACQueue[i] == mac)
      {
        done = 1; 
        /// double UP FIX 
//        testPingMasterMACQueue[i] = 0; 
      }    
      i++;
    }  
  }
  if (done == 1)
  {
    testPingMasterGoodAck++;
  }
  ackCount++;
}  

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION: AppTestPacketRX 
//------------------------------------------------------------------------------
//  
//==============================================================================
RxPacketResult AppTestPacketRX(CM2Packet* packet)
{
  uint8_t newOne,i; 
  RxPacketResult result = RX_PACKET_UNHANDLED;

  //-------------------------
  // Check if the app cares about this packet, if so then handle it here
  if (packet != 0)
  {
    if ((packet->destType == 0)&& (packet->destID != GetID()))
    {
       result = RX_PACKET_HANDLED;
    }
    else
    {
      
      //---------------------------------------
      // Handle dups 
      for (i=0;i<MAX_TEST_DUP_ENTRIES;i++)
      {
        if (testDupTimers[i] == 0)
        {
          testDupMACs[i] = 0; 
        }
      }  
      newOne = TRUE; 
      for (i=0;i<MAX_TEST_DUP_ENTRIES;i++)
      {
        if (packet->mac == testDupMACs[i])
        { 
          newOne = FALSE;
          result = RX_PACKET_HANDLED;
        }
      }
      if (newOne == TRUE)
      {
        //---------------------
        // add to oldest spot in the list 
        if (testDupOffset >= MAX_TEST_DUP_ENTRIES)
        {
          testDupOffset=0;
        }
        testDupMACs[testDupOffset] = packet->mac; 
        testDupTimers[testDupOffset] = TESTDELAY_300MSEC;
        testDupOffset++;
      }  
      //==============================================
      if (newOne == TRUE)
      {      
        result = RX_PACKET_HANDLED; 
        switch(packet->command[0])
        {
          case 0x18:
          {
            result = RX_PACKET_HANDLED;  
            CM2SendACK(packet);
              //-----------------------
              // start test 
              //----------------------
            AppTestProcess(packet);
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
                  SendOPVERSION(packet);
                  result = RX_PACKET_HANDLED; 
                  break;
              }    
              case 0x16:
              {
                AppTestSendOPTESTResult(packet,0,&opTest);
                result = RX_PACKET_HANDLED; 
                break;
              }
            }
            break;
          }
        }  
      }  
    }
  }  
  return result;
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppTestProcessTypes
//------------------------------------------------------------------------------
//  
//==============================================================================
void AppTestProcessTypes(void)
{
    float setPower; 

    __enable_interrupt();
    ThreadingInit();
    TimerInit(1);  
    AppSpiritInit(); 
    SetSpiritFrequency(RADIO1_TYPE,RADIO1_FREQUENCY);
    SetSpiritFrequency(RADIO2_TYPE,RADIO2_FREQUENCY);
    RegisterPacketRxCallback(AppTestPacketRX);
    appTestEvent = 0;  
  
    switch(myflashConfigSection.Item.appMode)
    {
      case APPTEST_BRAINDEAD:
      {
        //-------------------------
        // 
        __disable_interrupt();
        while(1)
        {
        }
        break;
      }
      case APPTEST_CWDEFAULTFREQ_FULLPOWER:
      case APPTEST_CWDEFAULTFREQ_0DB:
      case APPTEST_CWDEFAULTFREQ_MINUS10DB:
      {
        //-------------------------
        // 
        SetSpiritFrequency(RADIO1_TYPE,RADIO1_FREQUENCY);
        setPower = 11.6;
        if (myflashConfigSection.Item.appMode == APPTEST_CWDEFAULTFREQ_0DB)
        {
          setPower = 0;
        }
        if (myflashConfigSection.Item.appMode == APPTEST_CWDEFAULTFREQ_MINUS10DB)
        {
          setPower = -10;
        }        
        SetSpiritTxPower(RADIO1_TYPE,setPower);               
        SetSpiritMode(RADIO1_TYPE,SPIRIT_MODE_CW_TX);        
        __disable_interrupt();
        while(1)
        {
        }
        break;
      }       
      case APPTEST_CWDEFAULTFREQRADIO2_FULLPOWER:
      case APPTEST_CWDEFAULTFREQRADIO2_0DB:
      case APPTEST_CWDEFAULTFREQRADIO2_MINUS10DB:
      {        
        //-------------------------
        // 
        SetSpiritFrequency(RADIO2_TYPE,RADIO2_FREQUENCY);
        setPower = 11.6;
        if (myflashConfigSection.Item.appMode == APPTEST_CWDEFAULTFREQRADIO2_0DB)
        {
          setPower = 0;
        }
        if (myflashConfigSection.Item.appMode == APPTEST_CWDEFAULTFREQRADIO2_MINUS10DB)
        {
          setPower = -10;
        }        
        SetSpiritTxPower(RADIO2_TYPE,setPower);               
        SetSpiritMode(RADIO2_TYPE,SPIRIT_MODE_CW_TX);  
        __disable_interrupt();
        while(1)
        {
        }
        break;       
      }  
      case APPTEST_EUCWDEFAULTFREQ_FULLPOWER:
      case APPTEST_EUCWDEFAULTFREQ_0DB:
      case APPTEST_EUCWDEFAULTFREQ_MINUS10DB:
      {        
        //-------------------------
        // 
        SetSpiritFrequency(RADIO1_TYPE,SPIRIT_FREQ_EU_868_3);
        setPower = 11.6;
        if (myflashConfigSection.Item.appMode == APPTEST_EUCWDEFAULTFREQ_0DB)
        {
          setPower = 0;
        }
        if (myflashConfigSection.Item.appMode == APPTEST_EUCWDEFAULTFREQ_MINUS10DB)
        {
          setPower = -10;
        }        
        SetSpiritTxPower(RADIO1_TYPE,setPower);               
        SetSpiritMode(RADIO1_TYPE,SPIRIT_MODE_CW_TX);  
        __disable_interrupt();
        while(1)
        {
        }
        break;       
      }    
      case APPTEST_EUCM2TASKSONLY_ID0:
      {
        SetSpiritFrequency(RADIO1_TYPE,SPIRIT_FREQ_EU_868_3);
        tableContents[0] = SPIRITBACKDOORID_ID0; 
        SetPairingTable(tableContents,1,1);         
        AppTestInit();
        while(1)
        {
          AppTestTask();
        }        
        break;
      }      
#if  (SPIRIT_VER >= 0x00000008)            
      case APPTEST_9275CM2TASKSONLY_ID0:
      {
        SetSpiritFrequency(RADIO1_TYPE,SPIRIT_FREQ_PLC_927_5);
        tableContents[0] = SPIRITBACKDOORID_ID0; 
        SetPairingTable(tableContents,1,1);         
        AppTestInit();
        while(1)
        {
          AppTestTask();
        }        
        break;
      }                   
#endif    
      case APPTEST_92275CM2TASKSONLY_ID0:
      {
        SetSpiritFrequency(RADIO1_TYPE,SPIRIT_FREQ_922_75);
        tableContents[0] = SPIRITBACKDOORID_ID0; 
        SetPairingTable(tableContents,1,1);         
        AppTestInit();
        while(1)
        {
          AppTestTask();
        }        
        break;
      }                     
      case APPTEST_92125CM2TASKSONLY_ID0:
      {
        SetSpiritFrequency(RADIO1_TYPE,SPIRIT_FREQ_921_25);
        tableContents[0] = SPIRITBACKDOORID_ID0; 
        SetPairingTable(tableContents,1,1);         
        AppTestInit();
        while(1)
        {
          AppTestTask();
        }        
        break;
      }                 
      case APPTEST_902CM2TASKSONLY_ID0:
      {
        SetSpiritFrequency(RADIO1_TYPE,SPIRIT_FREQ_PLC_902);
        tableContents[0] = SPIRITBACKDOORID_ID0; 
        SetPairingTable(tableContents,1,1);         
        AppTestInit();
        while(1)
        {
          AppTestTask();
        }        
        break;
      }                       
      case APPTEST_CM2TASKSONLY_ID0:
      {
        tableContents[0] = SPIRITBACKDOORID_ID0; 
        SetPairingTable(tableContents,1,1);         
        AppTestInit();
        while(1)
        {
          AppTestTask();
        }        
        break;
      }
      case APPTEST_CM2TASKSONLY_ID1:
      {
        tableContents[0] = SPIRITBACKDOORID_ID1; 
        SetPairingTable(tableContents,1,1);   
        AppTestInit();
        while(1)
        {
          AppTestTask();
        }        
        break;
      }  
      case APPTEST_TMBASE_RADIO2_OTA_IDO:
      {
        tableContents[0] = SPIRITBACKDOORID_ID0; 
        SetPairingTable(tableContents,1,1);    
        if ((myflashConfigSection.Item.model[1] == 0)&&( myflashConfigSection.Item.model[0] == 30 ))
        {
#if BASEBOARD         
           HAL_GPIO_WritePin(PLCD1_GPIO_Port,PLCD1_Pin, GPIO_PIN_SET);
#endif                    
        }  
        AppTestInit();
        while(1)
        {
          AppTestTask();
        }        
        break;
      }      
      case APPTEST_TMBASE_RADIO2_PLC_IDO:
      {
        tableContents[0] = SPIRITBACKDOORID_ID0; 
        SetPairingTable(tableContents,1,1);    
        if ((myflashConfigSection.Item.model[1] == 0)&&( myflashConfigSection.Item.model[0] == 30 ))
        {       
#if BASEBOARD         
           HAL_GPIO_WritePin(PLCD1_GPIO_Port,PLCD1_Pin, GPIO_PIN_RESET);
#endif            
        }
        AppTestInit();
        while(1)
        {
          AppTestTask();
        }        
        break;
      }    
      case APPTEST_TMBASE_RADIO2_OTA_CWFULLPOWER:
      case APPTEST_TMBASE_RADIO2_OTA_CW0DB:
      case APPTEST_TMBASE_RADIO2_OTA_CWMINUS10DB:    
      {        
        //-------------------------
        // 
        SetSpiritFrequency(RADIO2_TYPE,RADIO2_FREQUENCY);
        setPower = 11.6;
        if (myflashConfigSection.Item.appMode == APPTEST_TMBASE_RADIO2_OTA_CW0DB)
        {
          setPower = 0;
        }
        if (myflashConfigSection.Item.appMode == APPTEST_TMBASE_RADIO2_OTA_CWMINUS10DB)
        {
          setPower = -10;
        }                
        if ((myflashConfigSection.Item.model[0] == 0)&&( myflashConfigSection.Item.model[1] == 30 ))
        {
#if BASEBOARD         
           HAL_GPIO_WritePin(PLCD1_GPIO_Port,PLCD1_Pin, GPIO_PIN_SET);
#endif            
        }          
        SetSpiritTxPower(RADIO2_TYPE,setPower);               
        SetSpiritMode(RADIO2_TYPE,SPIRIT_MODE_CW_TX);  
        __disable_interrupt();
        while(1)
        {
        }
        break;       
      }    
      case APPTEST_TMBASE_RADIO2_PLC_CWFULLPOWER:
      case APPTEST_TMBASE_RADIO2_PLC_CW0DB:
      case APPTEST_TMBASE_RADIO2_PLC_CWMINUS10DB:        
      {        
        //-------------------------
        // 
        SetSpiritFrequency(RADIO2_TYPE,RADIO2_FREQUENCY);
        setPower = 11.6;    
        if (myflashConfigSection.Item.appMode == APPTEST_TMBASE_RADIO2_PLC_CW0DB)
        {
          setPower = 0;
        }
        if (myflashConfigSection.Item.appMode == APPTEST_TMBASE_RADIO2_PLC_CWMINUS10DB)
        {
          setPower = -10;
        }                
        if ((myflashConfigSection.Item.model[0] == 0)&&( myflashConfigSection.Item.model[1] == 30 ))
        {
#if BASEBOARD         
           HAL_GPIO_WritePin(PLCD1_GPIO_Port,PLCD1_Pin, GPIO_PIN_RESET);
#endif            
        }          
        SetSpiritTxPower(RADIO2_TYPE,setPower);               
        SetSpiritMode(RADIO2_TYPE,SPIRIT_MODE_CW_TX);  
        __disable_interrupt();
        while(1)
        {
        }
        break;       
      }              
    }
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppTestProcess
//------------------------------------------------------------------------------
//  
//==============================================================================
void AppTestProcess(CM2Packet *packet)
{
  uint8_t *ptr,i; 
  uint16_t itemp,itemp2; 
  uint32_t ltemp,ltemp2; 
  float setPower; 
      
  ptr = &(packet->command[1]);
  opTest.Item.TestOperation = *ptr++;
  opTest.Item.SequenceNumber = *ptr++;
  opTest.Item.TxPower = *ptr++;
  opTest.Item.Frequency = *ptr++;
  opTest.Item.PingMasterInterval = *ptr++;
  opTest.Item.PingMasterPacketCount[0] = *ptr++;
  opTest.Item.PingMasterPacketCount[1] = *ptr++;
  opTest.Item.PingTestDest[0] = *ptr++;
  opTest.Item.PingTestDest[1] = *ptr++;
  opTest.Item.PingTestDest[2] = *ptr++;
  opTest.Item.PingTestDest[3] = *ptr++;  
  opTest.Item.CWTimeout = *ptr++;
  opTest.Item.PingTestResult[0] =  0;
  opTest.Item.PingTestResult[1] = 0; 
  opTest.Item.PingTestDestType = *ptr++;
  opTest.Item.DeviceSource = packet->deviceSource; 

    //-------------------------------
  // check frequency if in range of this libray 
  if (opTest.Item.Frequency >= MAX_SPIRIT_FREQ)
  {
    //-----------force to default 
    opTest.Item.Frequency = 0xff;
  }
    switch(packet->command[1])
    {
      case 0:
      {
        //-------------------------
        // stop currently running test. 
        // if one not running. 
        testState  = TESTSTATE_IDLE;
        break;
      }
      case 1:
      {
        ackCount = 0; 
        //-------------------
        // start a ping master test 
        testPingMasterOffset = 0; 
        
        ltemp = opTest.Item.PingTestDest[0];
        ltemp = ltemp<<24; 
        ltemp2 = opTest.Item.PingTestDest[1];
        ltemp2 = ltemp2<<16;
        ltemp |= ltemp2; 
        ltemp2 = opTest.Item.PingTestDest[2];
        ltemp2 = ltemp2<<8;
        ltemp |= ltemp2;        
        ltemp2 = opTest.Item.PingTestDest[3];
        ltemp |= ltemp2;        
        testPingMasterID = ltemp;
        testPingMasterDestType = opTest.Item.PingTestDestType; 
        testPingMasterInterval = opTest.Item.PingMasterInterval;
        testPingMasterGoodAck = 0;
        
        itemp = opTest.Item.PingMasterPacketCount[0];
        itemp = itemp<<8;      
        itemp2 = opTest.Item.PingMasterPacketCount[1];
        itemp |= itemp2;                
        testPingMasterCount = itemp;
        testState = TESTSTATE_PINGMASTER;
        
        for (i=0;i<MAX_PING_MAC_OFFSET;i++)
        {
          testPingMasterMACQueue[i] = 0;
        }
        testPingMasterMACOffset=0;         
        RegisterAckResultCallback(TestingAckCallback);
        
        //----------------------------
        // handle power settings. 
        if (opTest.Item.TxPower >= 0x2E)
        {
          setPower = 11.6;
        }
        else
        {
          setPower = opTest.Item.TxPower; 
          setPower += -34;
        }
        SetSpiritTxPower(packet->deviceSource,setPower);    
        break; 
      } 
      case 2:
      {
        //-------------------
        // start a CW test 
        testState = TESTSTATE_CW;
        //----------------------------
        // handle power settings. 
        if (opTest.Item.TxPower >= 0x2E)
        {
          setPower = 11.6;
        }
        else
        {
          setPower = opTest.Item.TxPower; 
          setPower += -34;
        }
        SetSpiritTxPower(packet->deviceSource,setPower);   
        SetSpiritFrequency(packet->deviceSource,opTest.Item.Frequency);
        testCWTimer = opTest.Item.CWTimeout;     
        SetSpiritMode(packet->deviceSource,SPIRIT_MODE_CW_TX);          
        break; 
      }       
      case 3:
      {
        //-------------------------
        // stop currently running test. 
        // if one not running. 
        testState  = TESTSTATE_TXBACKTOBACK;
        break;
      }
      
    }
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppTestProcess
//------------------------------------------------------------------------------
//  
//==============================================================================
void AppTestProcessSpecial(CM2Packet *packet)
{
  uint8_t *ptr,i; 
  uint16_t itemp,itemp2; 
  uint32_t ltemp,ltemp2; 
  float setPower; 
      
  ptr = &(packet->command[1]);
  opTest.Item.TestOperation = *ptr++;
  opTest.Item.SequenceNumber = *ptr++;
  opTest.Item.TxPower = *ptr++;
  opTest.Item.Frequency = *ptr++;
  opTest.Item.PingMasterInterval = *ptr++;
  opTest.Item.PingMasterPacketCount[0] = *ptr++;
  opTest.Item.PingMasterPacketCount[1] = *ptr++;
  opTest.Item.PingTestDest[0] = *ptr++;
  opTest.Item.PingTestDest[1] = *ptr++;
  opTest.Item.PingTestDest[2] = *ptr++;
  opTest.Item.PingTestDest[3] = *ptr++;  
  opTest.Item.CWTimeout = *ptr++;
  opTest.Item.PingTestResult[0] =  0;
  opTest.Item.PingTestResult[1] = 0; 
  opTest.Item.PingTestDestType = *ptr++;
  opTest.Item.DeviceSource = packet->deviceSource; 

    //-------------------------------
  // check frequency if in range of this libray 
  if (opTest.Item.Frequency >= MAX_SPIRIT_FREQ)
  {
    //-----------force to default 
    opTest.Item.Frequency = 0xff;
  }
    switch(packet->command[1])
    {
      case 0:
      {
        //-------------------------
        // stop currently running test. 
        // if one not running. 
        testState  = TESTSTATE_IDLE;
        break;
      }
      case 1:
      {
        ackCount = 0; 
        //-------------------
        // start a ping master test 
        testPingMasterOffset = 0; 
        
        ltemp = opTest.Item.PingTestDest[0];
        ltemp = ltemp<<24; 
        ltemp2 = opTest.Item.PingTestDest[1];
        ltemp2 = ltemp2<<16;
        ltemp |= ltemp2; 
        ltemp2 = opTest.Item.PingTestDest[2];
        ltemp2 = ltemp2<<8;
        ltemp |= ltemp2;        
        ltemp2 = opTest.Item.PingTestDest[3];
        ltemp |= ltemp2;        
        testPingMasterID = ltemp;
        testPingMasterDestType = opTest.Item.PingTestDestType; 
        testPingMasterInterval = opTest.Item.PingMasterInterval;
        testPingMasterGoodAck = 0;
        
        itemp = opTest.Item.PingMasterPacketCount[0];
        itemp = itemp<<8;      
        itemp2 = opTest.Item.PingMasterPacketCount[1];
        itemp |= itemp2;                
        testPingMasterCount = itemp;
        testState = TESTSTATE_PINGMASTER;
        
        for (i=0;i<MAX_PING_MAC_OFFSET;i++)
        {
          testPingMasterMACQueue[i] = 0;
        }
        testPingMasterMACOffset=0;         
        RegisterAckResultCallback(TestingAckCallback);
        
        //----------------------------
        // handle power settings. 
        if (opTest.Item.TxPower >= 0x2E)
        {
          setPower = 11.6;
        }
        else
        {
          setPower = opTest.Item.TxPower; 
          setPower += -34;
        }
        SetSpiritTxPower(packet->deviceSource,setPower);    
        break; 
      } 
      case 2:
      {
        //-------------------
        // start a CW test 
        testState = TESTSTATE_CW;
        //----------------------------
        // handle power settings. 
        if (opTest.Item.TxPower >= 0x2E)
        {
          setPower = 11.6;
        }
        else
        {
          setPower = opTest.Item.TxPower; 
          setPower += -34;
        }
        SetSpiritTxPower(packet->deviceSource,setPower);   
        SetSpiritFrequency(packet->deviceSource,opTest.Item.Frequency);
        testCWTimer = opTest.Item.CWTimeout;     
        SetSpiritMode(packet->deviceSource,SPIRIT_MODE_CW_TX);          
        break; 
      }       
      case 3:
      {
        //-------------------------
        // stop currently running test. 
        // if one not running. 
        testState  = TESTSTATE_TXBACKTOBACK;
        break;
      }
      
    }
}


 

