//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: APPTESTSUPPORT_H
//------------------------------------------------------------------------------
// VERSION  DATE     PERSON   DESCRIPTION
//  00.01   03/28/19  EMH   FIRST release. integrated into EA103 and EA095
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#ifndef APPTESTSUPPORT_H
#define APPTESTSUPPORT_H

#include "version.h" 
#include "cm2CoreTypesHeader.h"
//---------------------GLOBAL DEFINITIONS-------------------------- 

//-------------------------------------
// update your radio type for each radio.
// if you have one radio ... it should be the same for both
//	PORT_SPIRIT,
//	PORT_SPIRIT_ALT,
//	PORT_SPIRIT_PLC,
#define BASEBOARD               0
#define RADIO1_TYPE             PORT_SPIRIT
#define RADIO2_TYPE             PORT_SPIRIT
#define RADIO1_FREQUENCY        SPIRIT_FREQ_921_25 //SPIRIT_FREQ_921_25  
#define RADIO2_FREQUENCY        SPIRIT_FREQ_921_25  //SPIRIT_FREQ_PLC_927_5  
//-----------------------------------------


#if  (SPIRIT_VER == 0x00000006)
#define MAX_SPIRIT_FREQ 6  //version 6 of sw048 the max is 6 
                           //   SPIRIT_FREQ_921_25,
                           //   SPIRIT_FREQ_922_75,
                           //   SPIRIT_FREQ_926_75,
                           //   SPIRIT_FREQ_PLC_902,
                           //   SPIRIT_FREQ_EU_868_3, //1%
                           //   SPIRIT_FREQ_EU_868_95 //0.1%
#endif 
#if  (SPIRIT_VER >= 0x00000008)
#define MAX_SPIRIT_FREQ 7 
                        //  SPIRIT_FREQ_921_25,
                        //  SPIRIT_FREQ_922_75,
                        //  SPIRIT_FREQ_926_75,
                        //  SPIRIT_FREQ_PLC_902,
                        //  SPIRIT_FREQ_PLC_927_5,
                        //  SPIRIT_FREQ_EU_868_3, //1%
                        //  SPIRIT_FREQ_EU_868_95 //0.1%
#endif 

typedef union
{
  uint8_t Array[12];
  __packed struct
  {
    uint8_t TestOperation;
    uint8_t SequenceNumber;
    uint8_t TxPower;
    uint8_t Frequency; 
    uint8_t PingMasterInterval;
    uint8_t PingMasterPacketCount[2];
    uint8_t PingTestDest[4];
    uint8_t CWTimeout;
    uint8_t PingTestResult[2];
    uint8_t DeviceSource; 
    uint8_t PingTestDestType; 
  }Item;
}OpTest;

#define SPIRITBACKDOORID_ID0 0x33334444
#define SPIRITBACKDOORID_ID1 0x11112222

//====================================
// 0 = normal application mode
// 1 = brain dead board I/O initialized (blinking LED only if available). 
// 2 =  cm2 radio tasks only (supports OP_TEST and any other commands normal for board) – only paired ID is 0x33334444
// 3 = cm2 radio tasks only (supports OP_TEST and any other commands normal for board) – only paired ID is 0x11112222
// 4 – CW at board default frequency at FULL power 
// 5 = CW at board default frequency at 0db
// 6 = CW at board default frequency at -10db
// 7 – CW secondary radio at board default frequency at FULL power 
// 8 = CW secondary radio at board default frequency at 0db
// 9 = CW secondary radio at board default frequency at -10db
// 10 = EU cm2 radio tasks only (supports OP_TEST and any other commands normal for board) – only paired ID is 0x33334444
// 11 = EU CW at board default frequency at FULL power 
// 12 = EU CW at board default frequency at 0db
// 13 = EU CW at board default frequency at -10db
// 14 = 927.5 SINGLE radio tasks only (supports OP_TEST and OP_REQUEST/OP_VERSION) – only paired ID is 0x33334444
// 15 = 922.75 SINGLE radio tasks only (supports OP_TEST and OP_REQUEST/OP_VERSION) – only paired ID is 0x33334444
// 16 = 921.25 SINGLE radio tasks only (supports OP_TEST and OP_REQUEST/OP_VERSION) – only paired ID is 0x33334444
// 17 = 902 SINGLE radio tasks only (supports OP_TEST and OP_REQUEST/OP_VERSION) – only paired ID is 0x33334444
// 18 = Radio 2 TM Base Force OTA ID0 
// 19 = Radio 2 TM Base Force PLC ID0
// 20 = Radio 2 TM Base OTA CW FULL POWER
// 21 = Radio 2 TM Base OTA CW 0 db
// 22 = Radio 2 TM Base OTA CW -10db
// 23 = Radio 2 TM Base PLC CW FULL POWER
// 24 = Radio 2 TM Base PLC CW 0 db
// 25 = Radio 2 TM Base PLC CW -10db
#define APPTEST_NORMAL                          0
#define APPTEST_BRAINDEAD                       1
#define APPTEST_CM2TASKSONLY_ID0                2
#define APPTEST_CM2TASKSONLY_ID1                3
#define APPTEST_CWDEFAULTFREQ_FULLPOWER         4
#define APPTEST_CWDEFAULTFREQ_0DB               5
#define APPTEST_CWDEFAULTFREQ_MINUS10DB         6
#define APPTEST_CWDEFAULTFREQRADIO2_FULLPOWER   7
#define APPTEST_CWDEFAULTFREQRADIO2_0DB         8
#define APPTEST_CWDEFAULTFREQRADIO2_MINUS10DB   9
#define APPTEST_EUCM2TASKSONLY_ID0              10
#define APPTEST_EUCWDEFAULTFREQ_FULLPOWER       11
#define APPTEST_EUCWDEFAULTFREQ_0DB             12
#define APPTEST_EUCWDEFAULTFREQ_MINUS10DB       13
#define APPTEST_9275CM2TASKSONLY_ID0            14
#define APPTEST_92275CM2TASKSONLY_ID0           15
#define APPTEST_92125CM2TASKSONLY_ID0            16
#define APPTEST_902CM2TASKSONLY_ID0             17
#define APPTEST_TMBASE_RADIO2_OTA_IDO           18 
#define APPTEST_TMBASE_RADIO2_PLC_IDO           19  
#define APPTEST_TMBASE_RADIO2_OTA_CWFULLPOWER   20
#define APPTEST_TMBASE_RADIO2_OTA_CW0DB         21 
#define APPTEST_TMBASE_RADIO2_OTA_CWMINUS10DB   22
#define APPTEST_TMBASE_RADIO2_PLC_CWFULLPOWER   23
#define APPTEST_TMBASE_RADIO2_PLC_CW0DB         24 
#define APPTEST_TMBASE_RADIO2_PLC_CWMINUS10DB   25
#define APPTEST_NORMAL_PLCFULLPOWER             26
#define APPTEST_NORMAL_PLC0DB                   27
#define APPTEST_NORMAL_PLCMINUS10DB             28

//---------------------GLOBAL VARIABLES-------------------------- 
extern OpTest opTest; 
extern uint8_t appTestEvent;

#define TESTSTATE_IDLE          0
#define TESTSTATE_PINGMASTER    1
#define TESTSTATE_CW            2
#define TESTSTATE_TXBACKTOBACK  3
extern uint8_t testState; 
//---------------------GLOBAL PROTOTYPES--------------------------
void AppTestProcessTypes(void);
void AppTestProcess(CM2Packet *packet); 
void AppTestProcessSpecial(CM2Packet *packet); 

#endif