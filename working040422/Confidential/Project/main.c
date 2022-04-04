// Processor: STM32F103R
// TOOLS: IAR Workbench
// DATE:
// CONTENTS: This file contains Main program body
//------------------------------------------------------------------------------
// HISTORY: CM2 specific release updates
//  DATE    Version  whom Details
// HISTORY NOW RESIDES IN DEFS.H
//==============================================================================
#include "defs.h"
#include "config.h"
#include "JL_SpiritDriver.h"
#define PROJECT_CHK
#include "CrossCheck.h"
#undef PROJECT_CHK
#include "spiritInterfaceHeader.h"
#include "driverButtonPsuedo.h"
#include "driverFlash.h"
#include "appTestSupport.h"
#include "ProductionInfo.h"
#include "ThreadingHeader.h"
#include "TimersHeader.h"
#include "CM2CoreHeader.h"
#include "CM2SecurityHeader.h"
#include "beep.h"
#include "appMotorRampHeader.h"
#include "new_beep_header.h"
#include "motor_braking_header.h"

static bool appThreadRegistered = FALSE;
#if defined(PFC_ENABLED)
  #include "PIRegulatorClass.h"
#endif

#include "MCTuningClass.h"
#include "MCInterfaceClass.h"
#include "jlmarine.h"

#if defined(PFC_ENABLED)
  #include "PFCInit.h"
  #include "PFCApplication.h"
#endif

#include "MCTasks.h"
#include "Parameters conversion.h"
#ifdef DUALDRIVE
#include "Parameters conversion motor 2.h"
#endif
#include "Timebase.h"
#include "UITask.h"
#include "MCLibraryISRPriorityConf.h"


#include <stdio.h>

#if (defined(USE_STM32303C_EVAL))
#include "stm32303c_eval.h"
#else
#include "stm32_eval.h"
#endif

#include "driverMagSensor.h"

#define FIRMWARE_VERS "STM32 FOC SDK\0Ver.4.2.0"
const char s_fwVer[32] = FIRMWARE_VERS;

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */



/* Uncomment the following line to enable the demo mode */
/* #define DEMOMODE */

/* DEMO MODE prototypes, variables, macros */
#if defined(DEMOMODE)

#define MANUAL_MODE 0x00
#define DEMO_MODE   0x01

static volatile uint8_t Mode = DEMO_MODE;

void Demo(void);
void TqSpeedMode_start(void);
#endif

#if defined(EXAMPLE_SPEEDMONITOR)
  void speedmonitor_start(void);
#endif
#if defined(EXAMPLE_POTENTIOMETER)
 void potentiometer_start(void);
#endif
#if defined(EXAMPLE_RAMP)
  void ramp_start(void);
#endif
#if defined(EXAMPLE_PI)
  void NewPIval_start(void);
#endif
#if defined(EXAMPLE_CONTROLMODE)
 void TqSpeedMode_start(void);
#endif

/* Private function prototypes -----------------------------------------------*/

void SysTick_Configuration(void);
void SysMonitorTask(void);
static void MonitorElectrolysisInit(void);
static void CheckForElectrolysis(void);
void GndCaseEnable(void);
void GndCaseDisable(void);

//---------------------GLOBAL DEFINITIONS--------------------------------
#pragma location = 0x08001C00
__no_init ToolLoad myflashConfigSection;


//---------------------LOCAL VARIABLES------------------------------------
uint16_t motorOccurredFaults;
uint16_t motorCurrentFaults;
int16_t motorRPMReported;

 //--------------------------------------
  // scheduling of tasks
unsigned char schedCount;
unsigned char schedDone;
  //--------------------------------------
  // scheduling of tasks
unsigned int schedByte;

uint8_t motorOn = 0;
uint8_t motorStartFailureCount = 0;
uint8_t motorStopFailureCount = 0;
  //------------------------------------
  // following is updated in FOCDriveClass.c
int16_t   CurrentA, CurrentAdac;      //Motor Current
int16_t   Currents16;                 //Motor Current in s16
int16_t   Currents16LPF;              //Motor Current in s16 filtered
int32_t   slong0;          // temporary variable used for calculations, can be local

  //--------------------------
  // MOTOR state machine object to check fault state
  //--------------------------
CTSNS TempSensr1;
CSTM STM1;
CVBS BusVoltSense1;


CMCI oMCI[MC_NUM];
CMCT oMCT[MC_NUM];
uint32_t wConfig[MC_NUM] = {UI_CONFIG_M1,UI_CONFIG_M2};

  //---------------------------
  // System voltage readings
  // - from the motor library
uint16_t busV;
uint16_t busV_idle;
  //-----------------------------
  // System Temperagture readings
  // - from motor library
int16_t   motorTemp;
  //----------------------------
  // System motor reading - status from the motor library
  //----------------------------
State_t   motorLibState;



//---------------------GLOBAL VARIABLES-----------------------------------

uint16_t timerPairingSession=0;


//---------------------LOCAL FUNCTION PROTOTYPES--------------------------
void RunTimerBackgroundTasks(void);
void RunCM2CoreBackgroundTasks(void);
void RunSpiritBackgroundTasks(void);
void RunCM2PortsTasks(void);
uint8_t ButtonChanged(void);
void ButtonInit(void);
void ButtonSample(void);
void TestPropulsionRampTask(void);
bool OCL_GetStatus(void);
void OCL_Reset(void);
void OCL_UserReset(void);

void EMPTY_TASK(void)
{

}

typedef void swTask(void);
swTask *const SwTaskList[16] =
{
  EMPTY_TASK,
  EMPTY_TASK,
  AppModeHandleTask,
  DownloadSupTask,
  RunSpiritBackgroundTasks,
  RunCM2PortsTasks,
  RunCM2CoreBackgroundTasks,
  SaveTimingAdjTask,
  RunTimerBackgroundTasks,
  SysMonitorTask,
  CM2SupervisorTask,
  TimingStopMotorTask,
  TimingStopRPMTask,
  EMPTY_TASK,
  EMPTY_TASK,
  TestPropulsionRampTask
};

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
void InitIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable GPIO clock
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOC |
                         RCC_APB2Periph_GPIOD,
                         ENABLE);
  //speed is common to all pins
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  //------------- set MOT-EN as an otutput
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOA,GPIO_Pin_11,Bit_RESET);
  //--------------- set OCR interface OCL is input OCLR is output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //---------------------------
  //
  //---------------------------
  GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_RESET);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#if TEST_MEASURE_CLOCK
  //---------------------------
  //
  //---------------------------
  GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET);

#endif
#ifdef SERIAL_COMMUNICATION
#else
 //------------- SET PA3 as input pulled up
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

#endif

// --------------Set unused pins as input pulled up JDO
  // ---  PD2 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  // ---  PC12 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ---  PC11 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

   // ---  PC10 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // ---  PC12 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // ---  PA12 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // ---  PC9 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // ---  PC8 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // ---  PC7 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // ---  PC6 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ---  PB11 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

      // ---  PA0 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

      // ---  PC3 Pulled UP ---
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

AppInfo newImage;
uint8_t goodDownload;
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
void ResetDevice(void)
{
    /*!< Commented to stop resets post download */
//  goodDownload = 0;
//	///////SpiritShutdown(1);
//  if (GetAppInfo((uint32_t)Temp_ApplicationAddress, &newImage)!= 0)
//  {
//    if (newImage.checksum == ComputeChecksum(Temp_ApplicationAddress, newImage.appLength, newImage.checksumStartOffset, newImage.secondaryChecksumAddr))
//    {
//      goodDownload = 1;
//    }
//  }
//	NVIC->ICER[0] = 0xFFFFFFFF; //disable all interrupts
//	NVIC->ICPR[0] = 0xFFFFFFFF; //clear pending interrupts
//	EXTI->PR = 0xFFFF; //__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_All); //clear pending EXTI
//
//
//	// reset the chip.
//	NVIC_SystemReset();
}

void DeviceReset(void)
{
    NVIC->ICER[0] = 0xFFFFFFFF; //disable all interrupts
    NVIC->ICPR[0] = 0xFFFFFFFF; //clear pending interrupts
    EXTI->PR = 0xFFFF; //__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_All); //clear pending EXTI
//
    // reset the chip.
    NVIC_SystemReset();
}

uint8_t CheckDevice(void)
{
  //----------------------
  // Just completed a download ....
  // 1. check integrity - if good,
  //    a. see which board it is for and copy to serial flash
  //    b. update flash header information that is in EEPROM
  if (GetAppInfo((uint32_t)Temp_ApplicationAddress, &newImage)!= 0)
  {
    if (newImage.checksum == ComputeChecksum(Temp_ApplicationAddress, newImage.appLength, newImage.checksumStartOffset, newImage.secondaryChecksumAddr))
    {
      goodDownload = 1;
    }
  }
  return goodDownload;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
// BOC SISKO 06/01/2020
static bool test_new_beep = FALSE;
static motif test_tune = TEST_TONE;
static bool test_blocking = FALSE;
extern bool brake_initialized;
extern int32_t myspeed;
static bool test_ocl_reset = FALSE;
static bool spirit_init = FALSE;
static bool check_download_integrity = FALSE;
static bool is_download_good = FALSE;
// EOC SISKO

unsigned short testmyangle,testmyangle2;  //JDO 7221 for watching encoder when not running


uint8_t regimage[0xb0];
unsigned short myangle1;
uint8_t powerUpAdjust = 0;
int main(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  uint32_t myi;
  uint8_t newKey;
  uint32_t start = 0;
  uint8_t val;
  /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */

   __disable_interrupt();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

  //---------------------------------------
  // Set the Vector Table base location at 0x3000
  NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x3000);  //was 0x3000

  //============================================================================
  /* Brownout startup prevention */
  __enable_interrupt();
  SysTick_Configuration();
  BrownoutStartup();    /* Blocks until board voltage > BROWNOUT_VOLTAGE */
  //===========================================================================
  __disable_interrupt();

  /*MCInterface and MCTuning boot*/
  MCboot(oMCI,oMCT);
  //-------------------------------
  // Get state machine object to check fault state
  BusVoltSense1 = MCT_GetBusVoltageSensor(oMCT[0]);
  TempSensr1 = MCT_GetTemperatureSensor(oMCT[0]);
  STM1 = MCT_GetStateMachine(oMCT[0]);



  #if defined(PFC_ENABLED)
    PFC_Boot(oMCT[0],(CMCT)MC_NULL, (int16_t *)MC_NULL);
  #endif

  /*Systick configuration.*/
  SysTick_Configuration();

//DENNIS  jlminit();

 //---------------- START OF BETHS CHANGES
 // board intializaton
  InitIO();
  //--------------------------
  // get boot and app versions
   //---------------------
    // check bootloader veresion
  GetAppInfo((uint32_t)0x08000000, &boot);
  GetAppInfo((uint32_t)0x08003000, &app);



 //-------------MAG302 SUPPORT

  GPIO_WriteBit(GPIOA,GPIO_Pin_11,Bit_RESET);
  for (myi=0; myi<0x777;myi++);

  GPIO_WriteBit(GPIOA,GPIO_Pin_11,Bit_SET);
   for (myi=0; myi<0x777;myi++);
  MagInit();
   //-----------following writes the value 0x60 to register 0x07 and
  // then reads it back.
//  MagChangeRegister(0x07,0x60);
 // MagChangeRegister(0x02,0x35);
 //-------------------------------------

      //-------------------------------
      //  Arm the OCL circuit
  GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_SET);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
  // Enable GPIO clock
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,
                         ENABLE);
  //speed is common to all pins
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //---------------------------
  GPIO_WriteBit(GPIOB,GPIO_Pin_3,Bit_RESET);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 // Enable GPIO clock
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,
                         ENABLE);   //---------------------------
  GPIO_WriteBit(GPIOC,LED_RED_PIN,Bit_RESET);
  GPIO_WriteBit(GPIOC,LED_GREEN_PIN,Bit_RESET);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // BETH SPIRIT STUFF
  //----------------------------------
  // SPIRIT RADIO INTERFACE SETUP - BRAD'S
  //---------------------------------
  __disable_interrupt();
  ThreadingInit();
  TimerInit(1);
  //--------------------------
  // Initialize all application state
  // Register the application thread if needed
  if (!appThreadRegistered)
  {
    appThreadRegistered = TRUE;
  }
  __enable_interrupt();
  ButtonInit();
  ReadSettings();

  //AppSpiritInit();
  //-----------------------------
  // check if full app should run
  //----- if not just to mini schedule
  if ((myflashConfigSection.Item.appMode!= 0)&&(AppProductionVerifyInfo()!=0))
  {
     AppTestProcessTypes();
  }


  __enable_interrupt();

   /* Start here ***************************************************************/
  /* GUI, this section is present only if LCD, DAC or serial communication is */
  /* enabled.                                                                 */
#if (defined(LCD_FUNCTIONALITY) | defined(DAC_FUNCTIONALITY) | defined(SERIAL_COMMUNICATION))
  UI_TaskInit(UI_INIT_CFG,wConfig,MC_NUM,oMCI,oMCT,s_fwVer);
#endif
  /* End here******************************************************************/
  MCI_ExecSpeedRamp(oMCI[0],0,0);
//              MCI_StartMotor(oMCI[0]);

//  StartTimingAdjust(0);
 // PlayTune(CHARGE);
  MonitorElectrolysisInit();
  while(1)
  {
    schedDone = 0;
    schedCount = 0;
    
    //----------------------------------------
    //  JDO for watching myangle on livewatch when motor isn't running
    //---------------------------------------------------------------------
    //----------   JDO 7221 uncomment to watch angle on livewatch when motor isn't running
    //----------   reading the angle here will cause a disturbance in the motor while running.  only use to look at the angle when not running
//    testmyangle = MagRead16(0); 
//    testmyangle2 = testmyangle * 3;

    // =========================================================================
    // BOC SISKO 06/01/2020
    if(check_download_integrity != FALSE)
    {
        is_download_good = CheckDevice();
    }

    if(test_new_beep != FALSE)
    {
        PlayTuneNew(test_tune, test_blocking);
        test_new_beep = FALSE;
    }

    if((GetID() == 0xFFFFFFFF) && (brake_initialized == FALSE))
    {
        BrakeInit();
        MeasureEncoderRpmInit();

    }
    else if((GetID() != 0xFFFFFFFF) && (brake_initialized != FALSE))
    {
        MeasureEncoderRpmDeinit();
        BrakeDeinit();
    }

    if(test_ocl_reset != FALSE)
    {
        test_ocl_reset = FALSE;
        OCL_Reset();
    }

    if((spirit_init == FALSE) &&
       (GetMsSinceStart() > 1000) &&
       (GetEncoderRpm() < 200) &&
       (busV > 11))
    {
        spirit_init = TRUE;
        AppSpiritInit();
    }
    // EOC SISKO
    // =========================================================================

    while ((schedCount <16) && (schedDone ==0))
    {
      if ((schedByte & (1<<schedCount))!= 0)
      {
        //-------------------
        // disable interrupt
        //-------------------
        __disable_interrupt();
        schedByte &= (~(1<<schedCount));
        __enable_interrupt();
        SwTaskList[schedCount]();
      }
      schedCount++;
    }
    RunAllThreads();
    ButtonSample();
    if((AppModeGet() == MODE_NORMAL) && (brake_initialized == FALSE))
    {
      if (powerUpAdjust == 0)
      {
//        PlayTune(CHARGE);
        StartTimingAdjust(0);
        powerUpAdjust = 1;
      }
      else
      {
        TimingAdjustMotorStart();
      }

      if (spiritSupTimer == 0)
      {
          //-------------------------------
          // turn off motor
        // added code to count how many times we pass through this function with the timer expiered

        spiritSupTimer_expire_count ++;


        if ((motorOn != 0)&&(turnMotorOff== 0))
        {
          executeRPM = 0;  //JDO commented out for test only
          PropulsionRPMRamp(0);
          turnMotorOff = 1;
        }

      }
    }

    if (GetID() == 0xFFFFFFFF && GetMsElapsed(start) > (DEVICE_TYPE * 100))
    {
        start = GetMsSinceStart();
	RequestProductID();
    }

 //   motorTemp = TSNS_GetAvTemp_C(TempSensr1);
 //   Tempdac = motorTemp * 345;     //TT scale temp for DAC output, -20 to 95 C
 //   motorLibState = MCI_GetSTMState(oMCI[0]);

#ifdef SERIAL_COMMUNICATION

#else
#if 0
    //-----------------------------
    // process the C-monster key
    //----------------------------
    newKey = ButtonChanged();
    if (newKey != 0)
    {
      if ((newKey & KEY_CMONSTER)!= 0)
      {
          SetPromiscuousMode(TRUE);
          CM2SendPairRequest();
          timerPairingSession = TIMEPAIRSESSION;
      }
    }
#endif
#endif


#ifdef SERIAL_COMMUNICATION
    /* Start here ***********************************************************/
    /* GUI, this section is present only if serial communication is enabled.*/
    if (UI_SerialCommunicationTimeOutHasElapsed())
    {
      // Send timeout message
      Exec_UI_IRQ_Handler(UI_IRQ_USART,3,0); // Flag 3 = Send timeout error*/
    }
    /* End here**************************************************************/
#endif

#ifdef LCD_FUNCTIONALITY
    if (UI_IdleTimeHasElapsed())
    {
      UI_SetIdleTime(UI_TASK_OCCURENCE_TICKS);
      UI_LCDRefresh();
    }
#endif

#if defined(EXAMPLE_POTENTIOMETER)
   potentiometer_start();
#endif
#if defined(EXAMPLE_RAMP)
   ramp_start();
#endif
#if defined(EXAMPLE_PI)
   NewPIval_start();
#endif
#if defined(EXAMPLE_CONTROLMODE)
   TqSpeedMode_start();
#endif
#if defined(EXAMPLE_SPEEDMONITOR)
   speedmonitor_start();
#endif

  }
}

extern int16_t hMeasuredSpeedReported;
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
void SysMonitorTask(void)
{

  int16_t stemp;


  //---------------------------------
  // Read Bus Voltage and Temp
  //---------------------------------
  busV = VBS_GetAvBusVoltage_V(BusVoltSense1);
  motorTemp = TSNS_GetAvTemp_C(TempSensr1);
  motorLibState = MCI_GetSTMState(oMCI[0]);
  motorOccurredFaults = MCI_GetOccurredFaults(oMCI[0]);
  motorCurrentFaults = MCI_GetCurrentFaults(oMCI[0]);
 // motorRPMReported = hMeasuredSpeedReported;;  //MCI_GetAvrgMecSpeed01Hz(oMCI[0]);

  stemp = motorOccurredFaults>>8;
  tablex41.OccurredFaultsMSB = stemp;
  stemp = motorOccurredFaults & 0xFF;
  tablex41.OccurredFaultsLSB = stemp;

  stemp = motorCurrentFaults>>8;
  tablex41.CurrentFaultsMSB = stemp;
  stemp = motorCurrentFaults & 0xFF;
  tablex41.CurrentFaultsLSB = stemp;

  tablex41.FaultStateOverVoltage        = (motorCurrentFaults & MC_OVER_VOLT) >> 1;
  tablex41.FaultStateUnderVoltage       = (motorCurrentFaults & MC_UNDER_VOLT) >> 2;
  tablex41.FaultStateOverTemp           = (motorCurrentFaults & MC_OVER_TEMP) >> 3;
  tablex41.FaultStateOverCurrentLimit   = (motorCurrentFaults & MC_BREAK_IN) >> 6;


  tablex41.MotorLibState = motorLibState;

 //--------------------------------------------------
  tablex41.MotorTemp = motorTemp;
 //----------------------------------------------------
  tablex41.BusVoltage = busV;
 //----------------------------------------------
  tablex41.RPMSetting = targetSpeedRPM;
//  tablex41.RPMMeasured = motorRPMReported;
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


  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)==0)
  {
      tablex41.OCLReading = 0;
  }
  else
  {
      tablex41.OCLReading = 1;
  }

  //BOC SISKO 10-27-2020
  tablex41.OCLAutoResetOccured = OCL_GetStatus();
  // EOC SISKO 10-27-2020

  CheckForElectrolysis();

}

// BOC SISKO 10-27-2020: OCL Handling
// =============================================================================
#if 1
#define TIMER_SCALER            ((uint8_t)2)            /* Testing has shown that GetMsElapsed()
                                                            and GetMsSinceStart() are being
                                                            handled twice a millisecond */

#define OCL_TOGGLE_DELAY        ((uint32_t)50)          /*!< time (milliseconds) between
                                                                toggling OCLR state */

#define OCL_DEBOUNCE_TIME       ((uint32_t)500)         /*!< time (milliseconds) */

#define OCL_RESET_DELAY         ((uint32_t)3000)        /*!< time to elaspse before
                                                                resetting the motor */

#define OCL_STATUS_RESET        ((uint32_t)5000)        /*!< time (milliseconds) before
                                                                reseting OCL Auto reset status */

#define OCL_PERSIST_TIME        ((uint32_t)30000)       /*!< time in milliseconds */

#define OCL_CNTR_LIMIT          ((uint8_t)3)            /*!< # of times motor can auto reset
                                                                before requiring user reset */

#define OCL_RESET_RPM           ((int16_t)200)          /*!< rpm setting if OCL
                                                                occurs 3 or more times*/

static uint32_t ocl_reset_time = 0;

static int16_t prev_rpm = 0;

static uint8_t ocl_cntr = 0;            /*!< Num of times the OCL has been tripped */

bool wait_for_user_reset = FALSE;
/**
  * @brief      Container for handling overcurrent reset.
  * @retval     bool FALSE if OCL is set; TRUE if OCL is not set
  * @note
        Overcurrent occured:
        1.) Shut off motor
        2.) Set Status flag (persist for 5 seconds)
        3.) Reset OCL
        4.) Turn on motor (ramp to previous speed)

        Repeat Occurance:
        d.) if OCL 3 or more times in < 30 seconds
        1.) Turn off motor
        2.) Set speed to 1
        3.) User must turn on motor
  */
bool OCL_GetStatus(void)
{
  static uint32_t ocl_debounce = 0;
  static uint32_t ocl_timer = 0;
  static uint32_t motor_rest_time = 0;
  static bool status = FALSE;
  static bool update_cntr = TRUE;
  static bool new_motor_cmd = TRUE;

  /*!< Determine return status
   * =============================================*/
  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)==0)
  {
    if((GetMsElapsed(ocl_timer) >= (OCL_PERSIST_TIME * TIMER_SCALER)) &&
       (wait_for_user_reset != TRUE))
    {
        ocl_cntr = 0;
    }

    if(GetMsElapsed(ocl_timer) >= (OCL_STATUS_RESET * TIMER_SCALER))
    {
        status = FALSE;         /*!< OCL status == GOOD */
    }

    ocl_debounce = GetMsSinceStart();
    update_cntr = TRUE;
  }
  else
  {
      status = TRUE;            /*!< OCL status == BAD */
      ocl_timer = GetMsSinceStart();
  }

  /*!< Handle OCL occurances
   * =============================================*/
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)!=0)
  {
      /*!< Debounce and update flag(s)/counter(s) */
      if(GetMsElapsed(ocl_debounce) >= (OCL_DEBOUNCE_TIME * TIMER_SCALER))
      {
          ocl_debounce = GetMsSinceStart();

          if((ocl_cntr < 255) && (update_cntr != FALSE))
          {
              ocl_cntr++;
              update_cntr = FALSE;
              if(ocl_cntr >= OCL_CNTR_LIMIT)
              {
                  wait_for_user_reset = TRUE;
              }
          }
      }

      /*!< Motor Control */
      /* Motor OFF */
      if ((motorOn != 0)&&(turnMotorOff== 0))
      {
          if(new_motor_cmd == TRUE)
          {
              new_motor_cmd = FALSE;
              motor_rest_time = GetMsSinceStart();
              prev_rpm = executeRPM;
          }

          executeRPM = 0;
          PropulsionRPMRamp(0);
          turnMotorOff = 1;
      }

      if((GetMsElapsed(motor_rest_time) >= OCL_RESET_DELAY) &&
         (wait_for_user_reset == FALSE))
      {
          /* motor ON */
          if ((motorOn == 0)||(turnMotorOff!=0))
          {
              new_motor_cmd = TRUE;
              OCL_Reset();
              executeRPM = prev_rpm;
              PropulsionRPMRamp(executeRPM);
              MotorStartPropulsion();
              turnMotorOff = 0;
          }
      }
  }

//  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)==1)
//  {
//    /*!< Ramp to 0 */
//    prev_rpm = executeRPM;
////    PropCmdTable tmp = {0};
////    tmp.cmd_action          = 0x01;
////    tmp.cmd_goToRPM         = 0x01;
////    tmp.action              = 206;
////    tmp.goToRPM             = 0;
////    tmp.systemMode          = 0x05;
////    HandlePropCmd(&tmp);
//
//    if(GetMsElapsed(ocl_debounce) >= (OCL_RESET_DELAY * TIMER_SCALER))
//    {
//        ocl_debounce = GetMsSinceStart();
//
//        if(ocl_cntr > OCL_CNTR_LIMIT)
//        {
//            if(wait_for_user_reset == FALSE)
//            {
//                OCL_Reset();
//                executeRPM = 200;
//                ocl_cntr = 0;
////                    PropCmdTable tmp        = {0};
////                    tmp.cmd_action          = 0x01;
////                    tmp.cmd_goToRPM         = 0x01;
////                    tmp.action              = 206;
////                    tmp.goToRPM             = 200;
////                    tmp.systemMode          = 0x05;
////                    HandlePropCmd(&tmp);
//            }
//        }
//        else if(ocl_cntr <= OCL_CNTR_LIMIT)
//        {
//            OCL_Reset();
//            PropCmdTable tmp = {0};
//            tmp.cmd_action          = 0x01;
//            tmp.cmd_goToRPM         = 0x01;
//            tmp.action              = 204;
//            tmp.goToRPM             = prev_rpm;
//            tmp.systemMode          = 0x05;
//            HandlePropCmd(&tmp);
//
//            ocl_cntr++;
//            if(ocl_cntr > OCL_CNTR_LIMIT)
//            {
//                wait_for_user_reset = TRUE;
//            }
//        }
//    }
//  }

  return status;
}

/**
  * @brief      Toggle OCLR to reset OCL
  */
void OCL_Reset(void)
{
    GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_RESET);
    uint32_t tmp_timer = 0;
    tmp_timer = GetMsSinceStart();
    while(GetMsElapsed(tmp_timer) < (OCL_TOGGLE_DELAY * TIMER_SCALER))
    {
        __NOP();
    }

    ocl_reset_time = GetMsSinceStart();

    GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_SET);
    tmp_timer = GetMsSinceStart();
    while(GetMsElapsed(tmp_timer) < (OCL_TOGGLE_DELAY * TIMER_SCALER))
    {
        __NOP();
    }
}

/**
  * @brief      Set function for satisfying OCL reset condition
  * @see        void OCL_Handler(void)
  */
void OCL_UserReset(void)
{
    prev_rpm = OCL_RESET_RPM;
    ocl_cntr = 0;
    wait_for_user_reset = FALSE;
    return;
}

/**
  * @brief      return the number of times the ocl line has been tripped
  */
uint8_t GetOclCount(void)
{
    return ocl_cntr;
}

/**
  * @brief      return time in milliseconds since OCL was reset
  */
uint32_t GetMsSinceOclReset(void)
{
    return ocl_reset_time;
}
#endif
// EOC SISKO 10-27-2020: OCL Handling
// =============================================================================
// BOC SISKO 12-10-2020: Electrolysis Monitoring
// =============================================================================
#if 1
#include "ThreadingHeader.h"
#define ELECTROLYSIS_DELAY              ((uint32_t)(5000))      /*!< milliseconds */
#define ADC_UPDATE_DELAY                ((uint32_t)(250))       /*!< time required for dma to update adc (milliseconds) */
#define MAX_ELECTROLYSIS                ((int32_t)(500))        /*!< millivolts */
#define ELECTROLYSIS_SCALAR             ((float)(2.442))        /*!< millivolts per bit*/
//#define ELECTROLYSIS_OFFSET             ((int32_t)(2740))       /*!< raw adc value denoting 0V */
#define GNDING_VOLTAGE_SAMPLES          ((uint8_t)(16))         /*!< num samples used for averaging */

static uint16_t ELECTROLYSIS_OFFSET = 2740;
static uint16_t adc_dma_electrolysis[GNDING_VOLTAGE_SAMPLES] = {0};
static bool auto_grounding_enabled = FALSE;

static int16_t avg_electrolysis_mV = 0; /*!< live watch aid */
static bool case_grounded = FALSE;      /*!< live watch aid */

/**
  * @brief      Ground external case
  */
void GndCaseEnable(void)
{
    GPIO_WriteBit(GPIOC, GPIO_Pin_2, Bit_RESET);
    case_grounded = TRUE;
    return;
}

/**
  * @brief      Un-ground external case
  */
void GndCaseDisable(void)
{
    GPIO_WriteBit(GPIOC, GPIO_Pin_2, Bit_SET);
    case_grounded = FALSE;
    return;
}

void SetElectorylsisHandling(bool enable)
{
    auto_grounding_enabled = enable;
    return;
}

/**
  * @brief      Initialize adc and schedule sampling of case voltage
  */
void MonitorElectrolysisInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC->CFGR           |= RCC_CFGR_ADCPRE_DIV6;        /*!< Set AD prescaler */
    RCC->APB2ENR        |= RCC_APB2ENR_IOPCEN;          /*!< Enable port C*/
    RCC->APB2ENR        |= RCC_APB2ENR_ADC3EN;          /*!< Enable ADC 3 */
    RCC->APB2ENR        |= RCC_APB2ENR_AFIOEN;          /*!< Enable ADC ALT Function*/
    RCC->AHBENR         |= RCC_AHBENR_DMA2EN;           /*!< Enable DMA2*/

    /*!< Config ADC */
    RCC->CFGR           |= RCC_CFGR_ADCPRE_DIV6;        /*!< Set AD prescaler */
    RCC->APB2ENR        |= RCC_APB2ENR_ADC3EN;          /*!< Enable ADC 3 DMA clock*/
    RCC->APB2ENR        |= RCC_APB2ENR_AFIOEN;          /*!< Enable ADC ALT Function*/

    /*!< GPIO Configuration */
    GndCaseDisable(); /*!< DONT GND CASE UNTIL AFTER ADC CHECK! */

    /*!< Init pin used for grounding case*/
    GPIO_InitStructure.GPIO_Pin         = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed       = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode        = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*!< Init pin used for monitoring case voltage */
    GPIO_InitStructure.GPIO_Pin         = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode        = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    ADC3->SMPR1         |= ADC_SMPR1_SMP10_2 | ADC_SMPR1_SMP10_1 | ADC_SMPR1_SMP10_0;      /*!< Set sample rate: 239.5 cycles */
    ADC3->SQR3          |= 0x0A;        /*!< Assign channel to monitor*/
    ADC3->CR2           |= ADC_CR2_DMA;

    /*Configure DMA RX */
    DMA2_Channel5->CPAR = (uint32_t)(&(ADC3->DR));              /*<! where to read data from*/
    DMA2_Channel5->CMAR = (uint32_t)adc_dma_electrolysis;       /*<! where to store data */
    DMA2_Channel5->CNDTR = GNDING_VOLTAGE_SAMPLES;      /*<! read length */

    DMA2_Channel5->CCR |= DMA_CCR5_MSIZE_0;             /*!< Set Mem size */
    DMA2_Channel5->CCR |= DMA_CCR5_PSIZE_0;             /*!< Set Periph size */
    DMA2_Channel5->CCR |= DMA_CCR5_CIRC;                /*!< Circular mode */
    DMA2_Channel5->CCR |= DMA_CCR5_MINC;                /*!< Memory increment enable */
    DMA2_Channel5->CCR |= DMA_CCR5_EN;                  /*!< Begin DMA transfer */

    ADC3->CR2           |= ADC_CR2_ADON;                /*!< Turn the converter ON */
    ADC3->CR2           |= ADC_CR2_CONT;                /*!< Set mode to continuous */

    for(uint16_t delay=0; delay<=2000; delay++)
    {
        /* adc power up time: "tSTAB" == 2 ADC clock cycles */
    }

    ADC3->CR2           |= ADC_CR2_ADON;
    ADC3->CR2           |= ADC_CR2_CAL;

    while((ADC3->CR2 & ADC_CR2_CAL) == ADC_CR2_CAL)
    {
        /* wait for ADC calibration to complete */
    }

    if(tablex40.Item.configBitGroundingOn == TRUE)
    {
        SetElectorylsisHandling(TRUE);
    }

    return;
}

/**
  * @brief      Check for / Handles electroysis
  * @note       Does nothing if propulsion motor is ON
  */
void CheckForElectrolysis(void)
{
    static uint32_t stowed_timestamp = 0;

    if(tablex41.MotorOn == FALSE)
    {
        /*!< Inaccurate adc reading when case is grounded */
        if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == Bit_RESET)
        {
            GndCaseDisable();
            uint32_t temp_timestamp = 0;
            temp_timestamp = GetMsSinceStart();
            while(GetMsElapsed(temp_timestamp) < ADC_UPDATE_DELAY)
            {
                /*!< Spin and wait for adc to update */
            }
        }

        /*!< Calculate average adc value*/
        int32_t avg_electrolysis = 0;
        for(uint8_t i=0; i<GNDING_VOLTAGE_SAMPLES; i++)
        {
            avg_electrolysis += adc_dma_electrolysis[i];
        }
        avg_electrolysis = avg_electrolysis / GNDING_VOLTAGE_SAMPLES;

        if(tablex41.stow == TRUE)
        {
            /*!< Delay to ensure we're completely out of the water */
            if(GetMsElapsed(stowed_timestamp) > ELECTROLYSIS_DELAY)
            {
                stowed_timestamp = GetMsSinceStart();
                ELECTROLYSIS_OFFSET = avg_electrolysis; /*!< Update offset */
            }
        }
        else
        {
            stowed_timestamp = GetMsSinceStart();       /*!< Not stowed, so reset the timestamp */
            avg_electrolysis = avg_electrolysis - ELECTROLYSIS_OFFSET;
            avg_electrolysis = ((int32_t)((float)avg_electrolysis * ELECTROLYSIS_SCALAR)); /*!< total electrolysis (millivolts) */
            avg_electrolysis_mV = avg_electrolysis;

            if((((-1) * MAX_ELECTROLYSIS) < avg_electrolysis) && (avg_electrolysis < MAX_ELECTROLYSIS))
            {
                tablex41.FaultElectrolysis = FALSE;

                if(auto_grounding_enabled == TRUE)
                {
                    GndCaseEnable(); /*!< Should be safe to ground the case */
                }
            }
            else
            {
                /*!< Error not avaiable in Avenger's core version 0x86 */
                tablex41.FaultElectrolysis = TRUE;

                if(auto_grounding_enabled == TRUE)
                {
                    GndCaseDisable(); /*!< DO NOT GROUND THE CASE! ELECTROLYSIS DAMAGE WILL OCCUR*/
                }
            }
        }

        ////////////////////////////////////////////////////////////////////////////
//        if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2))
//        {
//            /*!< Case cannot be grounded for accurate adc reading */
//            GndCaseDisable();
//            uint32_t temp_timestamp = 0;
//            temp_timestamp = GetMsSinceStart();
//            while(GetMsElapsed(temp_timestamp) < ADC_UPDATE_DELAY)
//            {
//                /*!< Spin and wait for adc to update */
//            }
//        }
//
//
//        /*!< Calculate average adc value*/
//        for(uint8_t i=0; i<GNDING_VOLTAGE_SAMPLES; i++)
//        {
//            avg_electrolysis += adc_dma_electrolysis[i];
//        }
//        avg_electrolysis = avg_electrolysis / GNDING_VOLTAGE_SAMPLES;
//
//        /*!< Scale to match OP_AMP circuit */
//        avg_electrolysis = avg_electrolysis - ELECTROLYSIS_OFFSET;
//        avg_electrolysis = ((int32_t)((float)avg_electrolysis * ELECTROLYSIS_SCALAR)); /*!< total electrolysis (millivolts) */
//
//        if(((-1 * MAX_ELECTROLYSIS) < avg_electrolysis) && (avg_electrolysis < MAX_ELECTROLYSIS))
//        {
//            GndCaseEnable(); /*!< Should be safe to ground the case */
//        }
//        else
//        {
//            GndCaseDisable(); /*!< DO NOT GROUND THE CASE! ELECTROLYSIS DAMAGE WILL OCCUR*/
//        }
    }

    return;
}

#endif
// EOC SISKO xx-xx-xxxx: Electrolysis Monitoring
// =============================================================================


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
void SysTick_Configuration(void)
{
  /* Setup SysTick Timer for 500 usec interrupts  */
  if (SysTick_Config((SystemCoreClock) / SYS_TICK_FREQUENCY))
  {
    /* Capture error */
    while (1);
  }

  NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIORITY);
  NVIC_SetPriority(PendSV_IRQn, PENDSV_PRIORITY);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
