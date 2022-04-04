//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: DEFS_POWERPOLE.H
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// version 
// 00.01   emh 08/22/17  
//      1. Created versioned baseline. 
//
//==============================================================================

#ifndef __DEFS_H__
#define __DEFS_H__
 
#define false 0
#define true 1
//#include "version.h"
#include "mc_type.h"
#include "stm32f10x.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_adc.h"
#include "drive parameters.h"
#include "Timebase.h"
#include "UITask.h"
#include <stdlib.h>

#define MC_NUM 1

#define TEST_MOTORSTART  1

#define TEST_MOTORSTOP                  0   //PIN1
#define TEST_MOTORSTATEMACHINE          0   //PIN1 
#define TEST_SYSMONITORTASK             0   //PIN2
#define TEST_RTCTICKLETASK              0   //PIN2
#define TEST_RUNCM2COREBACKGROUNDTASKS  0   //PIN1
#define TEST_RUNCM2PORTSTASK            0   //PIN1
#define TEST_SPIRITBACKGROUNDTASKS      0   //PIN1
#define TEST_RSSISTOP                   0   //PIN2
#define TEST_RUNTIMERBACKGROUNDTASKS    0   //PIN2
#define TEST_RSSISTREAMTASK             0   //PIN2
#define TEST_PAIRTASK                   0   //PIN2
#define TEST_RUNSIMPLECOMMANDBLUETOOTH  0   //PIN2
#define TEST_BTSTREAMCURRENT            0   //PIN2


#define TESTWDOG 1
#define USEFLASH 0
#define FCCVERSION 0
#define MOTORCMD 1
#define JOHNSMOTORTEST 0
#define SPIRITMASTER 1
#define TESTINGPORTON 1
#define WDOGON 1 



#define TIME_5SECONDS 5000/12 

  //-----------------GPIO ON THE AVENGER BOARD 
#define LED_RED_PIN     GPIO_Pin_13
#define LED_GREEN_PIN   GPIO_Pin_14

  //-----------------DBOARD-------------
  // will remove the 433 radio 
  // will move the accelerometer CS  from PC9 to PD2 
  // will move the Bluetooth reset from PA4 to PC3
  // eeprom still on I2C but the part changed. 
#define TEST_BITS 2

#ifdef TEST_ARRAYS
 #define MAX_CURRENTA_COUNTS 10
  extern uint16_t   CurrentA_counts[MAX_CURRENTA_COUNTS+TEST_BITS];
#define MAX_DECODE_BUFFERS 8
#define NUM_DATA_BYTES  8
  extern uint8_t decodeDataBuffer[MAX_DECODE_BUFFERS+TEST_BITS][NUM_DATA_BYTES];
  extern uint8_t decodeGood[MAX_DECODE_BUFFERS+TEST_BITS]; 
  extern uint8_t decodeStamp[MAX_DECODE_BUFFERS+TEST_BITS];
#define PREAMBLE_DETECT_BITS      24         
  extern uint16_t preamble_counts[PREAMBLE_DETECT_BITS+TEST_BITS]; 
  extern uint8_t decoded_data[NUM_DATA_BYTES+TEST_BITS]; 
#define RF_MAX_BUFFERS  2 
#define DATA_BIT_CAPTURE_LENGTH   17  
  extern uint16_t dataCounts[RF_MAX_BUFFERS+TEST_BITS][(NUM_DATA_BYTES * DATA_BIT_CAPTURE_LENGTH)];    
  extern uint8_t firstDataValue[RF_MAX_BUFFERS+TEST_BITS][(NUM_DATA_BYTES  *DATA_BIT_CAPTURE_LENGTH)];
  extern uint16_t rfDataCounts[(NUM_DATA_BYTES * DATA_BIT_CAPTURE_LENGTH)+TEST_BITS];   
  extern uint8_t rfFirstDataValue[(NUM_DATA_BYTES  *DATA_BIT_CAPTURE_LENGTH)+TEST_BITS];
  extern short int messageGood[RF_MAX_BUFFERS+TEST_BITS]; 
//  extern uint8_t echoData[NUM_DATA_BYTES+1+TEST_BITS];

//#define MAX_GENERAL_BUFFER 128
//  extern uint8_t general_buffer[MAX_GENERAL_BUFFER+TEST_BITS];  
#define MAX_BLUETOOTH_RESP_BUFFER 128
  extern uint8_t module_resp_buffer[MAX_BLUETOOTH_RESP_BUFFER+TEST_BITS];  
#define MAX_BLUETOOTH_TX_BUFFER  150
  extern uint8_t tx_buffer[MAX_BLUETOOTH_TX_BUFFER+TEST_BITS];  
#endif
 
//---------------------GLOBAL DEFINITIONS--------------------------

#define BLUETOOTH_ST            0
#define BLUETOOTH_MICROCHIP     1
#define BLUETOOTH_EITHER        0
  
#define BLUETOOTH_ST_SETTING       0x71      
#define BLUETOOTH_UCHIP_SETTING    0x33


  
#define DISABLE_TEST_PINS       1    //set to 0, if using tes pins 
  //-------------------------------
  // TESTING - PIN OUTPUTS
  // TEST_RESET - this uses pin C.6 
  // TEST_PREAMBLE uses pin C.6 J9 pin #2
  // TEST_RFGOODPACKET uses pin C.5 = J8 pin #9
  // TEST_MOTOR_RUN uses pin C.7 J9 pin #3 
  // TEST_RTC uses pin C.5
  // TEST_PACKET_COPY uses pin C.7 J9 pin #3   

#define LOGFAULT_ENABLE 0
//#define TEST_RESET    
//#define TEST_PREAMBLE
//#define TEST_SHORT  
//#define TEST_RFGOODPACKET
//#define TEST_MOTOR_RUN
//#define TEST_PACKET_COPY 
//#define TEST_RTC

  //--------------------------------
  // MOTOR TYPE SETTINGS 
  // Brushed Dual motor selected.
  //--------------------------------
#define DUAL    //EMH - is this really used??? ask Jeff 
// Define BLDC or Brushed
#define BLDC    // UnComment for BLDC 
//#define SINGLE  // Uncomment for Brushed Single Motor
//#define DUAL    // Uncomment for Brushed Dual Motor

  
  //--------------------------------
  // ACCELEROMETER SETTING - Used to disable accelerometer when 
  // the DAC is used for testing 
  //--------------------------------
//uncomment to use the Accelerometer feature
#define ACCEL_ACTIVE

  //----------------------------------
  // MOTOR SETTING - Resides in FOCDriveClass.c
  // (no longer used here - updated in the motor file)
  // filter intergral variable
//#define filtertc 512 //4096

  //-------------------------------------
  // FIRMWARE VERSION - update with new builds.
  //--------------------------------------
#define CONFIGVERSION        0x31
    //CONFIGVERSION changed 10-18-2016 to force remote speed range from 0 to 100 to 0 to 70 
  
#define SW_VERSION          "00.05 "  //SW version
#define SW_VER0              0         //SW version
#define SW_VER1              0
#define SW_VER2              '.'
#define SW_VER3              0
#define SW_VER4              5
#define SW_VER5              ' ' 
//Define SW_VER               0x12
  
#define SWAPPMSB            '0'
#define SWAPPMID             '5'
#define SWAPPLSB             '3'
#define SWBOOTMSB            '0'
#define SWBOOTMID             '6'
#define SWBOOTLSB             '8'  
 
#define SWAPP_LOCATION_S      (uint8_t)(*((uint16_t*)0x0800314C)) 
#define SWAPP_LOCATION_W      (uint8_t)(*((uint16_t*)0x08003150))   
#define SWAPPMSB_LOCATION     (uint8_t)(*((uint16_t*)0x08003154)) 
#define SWAPPMID_LOCATION     (uint8_t)(*((uint16_t*)0x08003158))     
#define SWAPPLSB_LOCATION     (uint8_t)(*((uint16_t*)0x0800315C))     
  
#define SWBOOT_LOCATION_S      (uint8_t)(*((uint16_t*)0x08000140)) 
#define SWBOOT_LOCATION_W      (uint8_t)(*((uint16_t*)0x08000144))   
#define SWBOOTMSB_LOCATION     (uint8_t)(*((uint16_t*)0x08000148)) 
#define SWBOOTMID_LOCATION     (uint8_t)(*((uint16_t*)0x0800014C))     
#define SWBOOTLSB_LOCATION     (uint8_t)(*((uint16_t*)0x08000150))       
  
  
#define ID_H_0              (uint8_t)(*((uint16_t*)0x1FFFF7E8))
#define ID_H_1              (uint8_t)(*((uint16_t*)0x1FFFF7EC))
#define ID_H_2              (uint8_t)(*((uint16_t*)0x1FFFF7ED))
#define ID_H_3              (uint8_t)(*((uint16_t*)0x1FFFF7EE))
#define ID_H_4              (uint8_t)(*((uint16_t*)0x1FFFF7EF))
#define ID_H_5              (uint8_t)(*((uint16_t*)0x1FFFF7F2))
#define ID_H_6              (uint8_t)(*((uint16_t*)0x1FFFF7F3))
#define ID_M                (uint8_t)(*((uint16_t*)0x1FFFF7F1))
#define ID_L                (uint8_t)(*((uint16_t*)0x1FFFF7F0))


#ifdef BLDC
  #define M_CONFIG          1  // BLDC
#elif defined SINGLE
  #define M_CONFIG          2  // Single Brushed  
#elif defined DUAL
  #define M_CONFIG          3  // Dual Brushed   
#endif

#define MOTOR_RUN_TIME      250 //ms
#define MOTOR_REVERSE_DELAY 250 //ms
#define MOTOR_DEAD_TIME     200 //ns
#define MOTOR_ON_DELAY      30  //ms
#define BT_TX_TIMEOUT       100 //100ms
#define BT_RX_TIMEOUT       300 //300ms
#define OVERTEMP            95  //deg C

#define REFSPD              200 //.1 Hz
 
// Bootloader Version
#define BT_VER_4              (uint8_t)(*((uint16_t*)0x08000158))      
#define BT_VER_3              (uint8_t)(*((uint16_t*)0x0800015C))    
#define BT_VER_2              (uint8_t)(*((uint16_t*)0x08000160))    
#define BT_VER_1              (uint8_t)(*((uint16_t*)0x08000164))    
#define BT_VER_0              (uint8_t)(*((uint16_t*)0x08000168))    
  
  //-------------------------if #0 then uses REAL JLM DATA 
  // --------------------------otherwise #1 uses HARDCODED JLM DATA
// JLM Data
#if 0
#define DC_MIN_L 20
#define DC_MAX_L 100  
#define OPTION_BYTE 1
#define PSI_UP       0x78         
#define PSI_DN       0x50         
#define MODEL        0x10  //bld-10         
#define MODEL_REV    0x61   
#define MODEL_REV1   0x62
#define MODELMSB     0x00           
#define LOT_CODE_4   0x31          
#define LOT_CODE_3   0x32         
#define LOT_CODE_2   0x33       
#define LOT_CODE_1   0x34       
#define LOT_CODE_0   0x35         
#else  
#define MODEL_REV1            (uint8_t)(*((uint16_t*)0x08002FF3))  
#define DC_MIN_L              (uint8_t)(*((uint16_t*)0x08002FF4))   
#define DC_MAX_L              (uint8_t)(*((uint16_t*)0x08002FF5))   
#define OPTION_BYTE           (uint8_t)(*((uint16_t*)0x08002FF6)) 
#define PSI_UP                (uint8_t)(*((uint16_t*)0x08002FF7)) 
#define PSI_DN                (uint8_t)(*((uint16_t*)0x08002FF8)) 
#define MODEL                 (uint8_t)(*((uint16_t*)0x08002FF9)) 
#define MODEL_REV             (uint8_t)(*((uint16_t*)0x08002FFA)) 
#define MODELMSB              (uint8_t)(*((uint16_t*)0x08002FEE))   
  // Lot Code
#define LOT_CODE_4            (uint8_t)(*((uint16_t*)0x08002FFB)) 
#define LOT_CODE_3            (uint8_t)(*((uint16_t*)0x08002FFC)) 
#define LOT_CODE_2            (uint8_t)(*((uint16_t*)0x08002FFD)) 
#define LOT_CODE_1            (uint8_t)(*((uint16_t*)0x08002FFE)) 
#define LOT_CODE_0            (uint8_t)(*((uint16_t*)0x08002FFF)) 
#endif 
  
//Main states
#define COMSTATE_IDLE		1
#define COMSTATE_MANUALSTART	2
#define COMSTATE_MANUALACTIVE	3
#define COMSTATE_MANUALSTOP     4
#define PAIRING		        8

// Power modes
#define IDLE_PWR          0
#define OP_PWR            1
#define STANDBY_PWR       2

//RX modes
#define RX_OFF          0
#define PREAMBLE_DETECT 1
#define DATA_DECODE     2
#define RX_RESET        3
#define RX_CRC          4

//instances
#define LEFT  '2' //0x0032
#define RIGHT '1' //0x0031
#define BOTH  '0' //0x0030

//directions and RX Commands
#define UP          0x01
#define DOWN        0x02
#define SLP_NOTE    0x03
#define UP_AUTO     0x11
#define DOWN_AUTO   0x12
#define STOP_MOTOR  0xFB

//LED colors
#define RED     0
#define GREEN   1
#define YELLOW  2

//bottom settings
#define AUTO    0
#define SOFT    1
#define HARD    2
#define SOFT_THRESHOLD  40
#define HARD_THRESHOLD  50


  //-------------------------------
  // status values
#define SUCCESSFUL 1
#define UNSUCCESSFUL    0

//delta I state
typedef enum
{ POS, NEG, FLAT } DeltaA_State;



//programming state
typedef enum
{   INST = 1, 
    R2_MAX, 
    SENSE_UP,
    SENSE_DN,
    ROLE} Prog_State;

//auto bottom readings
typedef enum
{   AUTO_HARD, 
    AUTO_SOFT} Hard_Factor;
/*
//internal error defines
#define MC_OVER_VOLT    0x0002
#define MC_UNDER_VOLT   0x0004
#define MC_OVER_TEMP    0x0008
#define MC_START_UP     0x0010
#define MC_SPEED_FDBK   0x0020
*/
    
//LED defines
#define RED_LED_ON    GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_SET) //active high
#define RED_LED_OFF   GPIO_WriteBit(GPIOB,GPIO_Pin_6,Bit_RESET)
#define GREEN_LED_ON  GPIO_WriteBit(GPIOB,GPIO_Pin_7,Bit_SET)
#define GREEN_LED_OFF GPIO_WriteBit(GPIOB,GPIO_Pin_7,Bit_RESET)

//short delay
#define DEADTIME_DELAY asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop")

//array length macro
#define ARRAY_LEN(x) sizeof(x)/sizeof(x[0])

//---------------------GLOBAL VARIABLES--------------------------
 
  
//---------------------GLOBAL PROTOTYPES--------------------------
typedef union
{
  uint8_t Index[28];
  struct
  {
    uint8_t MotorTempMSB;
    uint8_t MotorTempLSB;
    uint8_t BusVoltageMSB;
    uint8_t BusVoltageLSB;
    uint8_t RPMMSB;
    uint8_t RPMLSB;
    uint8_t FirmwareVersion[6];   
    uint8_t spare[16];
  }Item;
}STATUSTABLE; 
extern STATUSTABLE statusTable;

//--------------------------------------
// scheduling of tasks
extern unsigned int schedByte;
#define SCHEDBYTE_MOTOROFF              0x0001
#define SCHEDBYTE_MOTORSTATEMACHINE     0x0002
 
#define SCHEDBYTE_RTC_TICKLE            0x0008
#define SCHEDBYTE_CM2COREBACKGROUND     0x0010
#define SCHEDBYTE_CM2PORTS              0x0020
#define SCHEDBYTE_SPIRITBACKGROUND      0x0040

#define SCHEDBYTE_RSSI_DONE             0x0080
#define SCHEDBYTE_TIMERBACKGROUND       0x0100
//#define SCHEDBYTE_SPIRIT_ISR            0x0100
#define SCHEDBYTE_SYSMONITOR           0x0200
#define SCHEDBYTE_PAIRTASK              0x0400
#define SCHEDBYTE_RUN_SIMPLE_BLUETOOTH  0x0800
#define SCHEDBYTE_BT_STREAMCURRENT      0x1000
#define SCHEDBYTE_BLUETOOTHEXITBOND     0x2000 
#define SCHEDBYTE_YMFIRMWARE            0x4000


#define MAX_RSSI_READING 10
extern uint16_t rssiReading[MAX_RSSI_READING];
extern uint8_t rssiReadingOffset; 

  //-------------------------------
  // BLUETOOTH PIN SETTINGS 
 
  #define BLUETOOTH_RESET_PORT    GPIOA
  #define BLUETOOTH_RESET_PIN     GPIO_Pin_4

#if FCCVERSION
extern uint8_t fccOffset; 
#endif 

extern CVBS BusVoltSense1;  
extern CMCI oMCI[MC_NUM];
extern uint16_t  busV, busV_idle;

  //----------12msec timers 
#define TIMERSYSMONITORTIME     500/12 
extern uint8_t timerSysMonitor;

extern uint16_t timerPairingSession;
#define TIMEPAIRSESSION (10*1000)/12

extern uint8_t motorOn;

#endif
//end of defs.h
