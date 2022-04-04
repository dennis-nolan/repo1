//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: DEFS_POWERPOLE.H
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// HISTORY: CM2 specific release updates
//  DATE    Version  whom Details
// 08/22/2017 00.01  EMH  Baseline for REAL application. Framework added for application
//                        remote downloads.
// 12/09/2017 00.06  EMH  Added config stuff and storing to flash
//==============================================================================

#ifndef __DEFS_H__
#define __DEFS_H__

#define false 0
#define true 1

#include "intrinsics.h"
#include "mc_type.h"
#include "stm32f10x.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_adc.h"
#include "drive parameters.h"
#include "Timebase.h"
#include "UITask.h"
#include "driverFlash.h"
#include "avengersCoreheader.h"
#include <stdlib.h>

#define TEST_MEASURE_CLOCK 0

#define MC_NUM 1
#define WDOGON 1
#define MY_MAX_APPLICATION_SPEED 6500

//----- Address defines
#define DEVICE_TYPE		DEV_TYPE_PROP

#define TIME_5SECONDS 5000/12

  //-----------------GPIO ON THE AVENGER BOARD
#define LED_RED_PIN     GPIO_Pin_13
#define LED_GREEN_PIN   GPIO_Pin_14

//---------------------GLOBAL DEFINITIONS--------------------------
  //--------------------------------
  // MOTOR TYPE SETTINGS
  // Brushed Dual motor selected.
  //--------------------------------
#define DUAL    //EMH - is this really used??? ask Jeff
// Define BLDC or Brushed
#define BLDC    // UnComment for BLDC
//#define SINGLE  // Uncomment for Brushed Single Motor
//#define DUAL    // Uncomment for Brushed Dual Motor

  //-------------------------------------
  // FIRMWARE VERSION - update with new builds.
  //--------------------------------------
#define SW_VERSION          "00.40 "  //SW version
#define SW_VER0              0         //SW version
#define SW_VER1              0
#define SW_VER2              '.'
#define SW_VER3              4
#define SW_VER4              0
#define SW_VER5              ' '

#ifdef BLDC
  #define M_CONFIG          1  // BLDC
#elif defined SINGLE
  #define M_CONFIG          2  // Single Brushed
#elif defined DUAL
  #define M_CONFIG          3  // Dual Brushed
#endif


//LED defines
#define RED_LED_ON    GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET) //active high
#define RED_LED_OFF   GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_RESET)
#define GREEN_LED_ON  GPIO_WriteBit(GPIOC,GPIO_Pin_14,Bit_SET)
#define GREEN_LED_OFF GPIO_WriteBit(GPIOC,GPIO_Pin_14,Bit_RESET)

//---------------------GLOBAL VARIABLES--------------------------


//---------------------GLOBAL PROTOTYPES--------------------------

//--------------------------------------
// scheduling of tasks
extern unsigned int schedByte;
#define SCHEDBYTE_APPMODETIMEOUT        0x0004
#define SCHEDBYTE_DOWNLOADTIMEOUT       0x0008

#define SCHEDBYTE_SPIRITBACKGROUND      0x0010
#define SCHEDBYTE_CM2PORTS              0x0020
#define SCHEDBYTE_CM2COREBACKGROUND     0x0040
#define SCHEDBYTE_SAVETIMNGADJ          0x0080

#define SCHEDBYTE_TIMERBACKGROUND       0x0100
#define SCHEDBYTE_SYSMONITOR            0x0200
#define SCHEDBYTE_SUPERVISOR            0x0400
#define SCHEDBYTE_STOPMOTOR             0x0800

#define SCHEDBYTE_RPMMOTORSTOP          0x1000



extern CVBS BusVoltSense1;
extern CMCI oMCI[MC_NUM];
extern uint16_t  busV, busV_idle;

  //----------12msec timers
#define TIMERSYSMONITORTIME     500/12
extern uint8_t timerSysMonitor;

  //----------10msec timers
#define TIMERSUPMONITORTIME     10
extern uint8_t timerSupMonitor;

#define SPIRIT_SUP_TIME         450    /*!< Timeout before auto-offing the motor. Measured in 10 ms increments (e.g. 450 == 4500 ms)*/
extern uint16_t spiritSupTimer;
extern uint16_t spiritSupTimer_expire_count;
extern int16_t averageMotorRPMReported;
extern int16_t newRPM;
extern int16_t executeRPM;
extern uint8_t lastParameter;
extern uint8_t motorOn;
extern uint8_t motorStartFailureCount;
extern uint8_t motorStopFailureCount;
extern AppInfo app;
extern AppInfo boot;
extern uint32_t freeRunningTimer;
void MyStopMotor(void);
#endif
//end of defs.h
