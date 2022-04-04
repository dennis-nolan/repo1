// Processor: STM32F103R
// TOOLS: IAR Workbench 
// DATE:
// CONTENTS: This file contains Main program body 
//------------------------------------------------------------------------------
// HISTORY: CM2 specific release updates
//  DATE    Version  whom Details
// 08/22/2017 00.01  EMH  Baseline for REAL application. Framework added for application
//                        remote downloads. 


#include "defs.h"  
 
#define PROJECT_CHK
#include "CrossCheck.h" 
#undef PROJECT_CHK
#include "spiritInterfaceHeader.h"
#include "driverButtonPsuedo.h"

//------------BRAD CM2 ADDITIONS 
  //--------brads headers
#include "ThreadingHeader.h"
#include "TimersHeader.h"
#include "CM2CoreHeader.h"
#include "CM2SecurityHeader.h"
  
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
//---------------------GLOBAL DEFINITIONS--------------------------------

//---------------------LOCAL VARIABLES------------------------------------

 //--------------------------------------
  // scheduling of tasks
unsigned char schedCount; 
unsigned char schedDone; 
  //--------------------------------------
  // scheduling of tasks
unsigned int schedByte; 

uint8_t motorOn = 0; 

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

  //----timerSysMonitor - 12msec timer 
uint8_t timerSysMonitor;

uint16_t timerPairingSession=0;

STATUSTABLE statusTable;
//---------------------LOCAL FUNCTION PROTOTYPES--------------------------  

void RunTimerBackgroundTasks(void);
void RunCM2CoreBackgroundTasks(void);
void SpiritBackgroundTasks(void);
void RunCM2PortsTasks(void);

uint8_t ButtonChanged(void);
void ButtonInit(void);
void ButtonSample(void);


void EMPTY_TASK(void)
{
  
}  

typedef void swTask(void);
swTask *const SwTaskList[16] =
{
  EMPTY_TASK,
  EMPTY_TASK, 
  EMPTY_TASK, 
  EMPTY_TASK,
  RunCM2CoreBackgroundTasks, 
  RunCM2PortsTasks,
  SpiritBackgroundTasks,
  EMPTY_TASK,
  RunTimerBackgroundTasks,  
  SysMonitorTask,  
  EMPTY_TASK, 
  EMPTY_TASK,
  EMPTY_TASK,
  EMPTY_TASK,  
  EMPTY_TASK,  
  EMPTY_TASK
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
                         RCC_APB2Periph_GPIOA,
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
  
#ifdef SERIAL_COMMUNICATION 
#else
 //------------- SET PA3 as input pulled up 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
#endif   
 
  
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  
//------------------------------------------------------------------------------
// This function 
//==============================================================================
void ResetDevice(void)
{
	///////SpiritShutdown(1);
	
	NVIC->ICER[0] = 0xFFFFFFFF; //disable all interrupts
	NVIC->ICPR[0] = 0xFFFFFFFF; //clear pending interrupts
	EXTI->PR = 0xFFFF; //__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_All); //clear pending EXTI

 
	// reset the chip.
	NVIC_SystemReset();
}

#define ApplicationAddress        0x8003000
#define Temp_ApplicationAddress   0x8023000 
#define Temp_ApplicationAddressRemote 0x8043000
#define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
#define FLASH_SIZE                        (0x60000)  /* 256 KBytes */
#define APPSIZEALLOWED             0x1FFFF //0x18fff  //used to be 0xcfff

uint32_t FlashDestinationRemote = Temp_ApplicationAddressRemote;
uint32_t FlashDestination = Temp_ApplicationAddress; /* Flash user program offset */
uint32_t FlashSource = ApplicationAddress;
uint16_t PageSize = PAGE_SIZE;
uint32_t EraseCounter = 0x0;
uint32_t NbrOfPage = 0;
FLASH_Status FLASHStatus = FLASH_COMPLETE; // added when adding FLASH protect
uint8_t *RamSource;

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:    
//------------------------------------------------------------------------------
// This function   
//==============================================================================
uint32_t FLASH_PagesMask(__IO uint32_t Size)
{
  uint32_t pagenumber = 0x0;
  uint32_t size = Size;

  if ((size % PAGE_SIZE) != 0)
  {
    pagenumber = (size / PAGE_SIZE) + 1;
  }
  else
  {
    pagenumber = size / PAGE_SIZE;
  }
  return pagenumber;

}

bool FlashWriteScratch(uint32_t offset, uint32_t data)
{

  /*
	//unlock flash
	if (HAL_FLASH_Unlock() != HAL_OK)
	{
 		return FALSE;
	}
	
	//write flash
	if (*((uint32_t*)(Temp_ApplicationAddress+offset)) != data && 
		(HAL_FLASH_Program(TYPEPROGRAM_WORD, Temp_ApplicationAddress+offset, data) != HAL_OK))
	{
		HAL_FLASH_Lock();
		 
		return FALSE;
	}
	
	//wrap up
	HAL_FLASH_Lock();
*/
 FLASH_ProgramWord(Temp_ApplicationAddress+offset,data);  
	return TRUE;
} 

bool FlashEraseScratch(void)
{
  /*
    uint32_t size;
          HAL_FLASH_Unlock();     
          size = APPSIZEALLOWED;
          EraseInitStruct.PageAddress = FlashDestination;
          EraseInitStruct.NbPages = size / FLASH_PAGE_SIZE + ((size % FLASH_PAGE_SIZE) > 0);
          HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);  
          HAL_FLASH_Lock();
    */      
         NbrOfPage = FLASH_PagesMask(APPSIZEALLOWED); //ymodemSize);
         FLASH_Unlock();          
          //--------------------------------------
          // Erase the FLASH pages 
          for (EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
          {
            FLASHStatus = FLASH_ErasePage(FlashDestination + (PageSize * EraseCounter));
            if (FLASHStatus != FLASH_COMPLETE)
            {
             FLASHStatus = FLASH_ErasePage(FlashDestination + (PageSize * EraseCounter)); 
            }
          }          
          
          
          
  return TRUE;
  
  
  
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  
//------------------------------------------------------------------------------
// This function 
//============================================================================== 
uint16_t newRPM; 
int main(void)
{    
  GPIO_InitTypeDef GPIO_InitStructure;
  uint32_t myi; 
  uint8_t newKey; 
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
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x3000);  //was 0x3000
  
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

 //-------------MAG302 SUPPORT 
 
  GPIO_WriteBit(GPIOA,GPIO_Pin_11,Bit_RESET);
  for (myi=0; myi<0x777;myi++); 
  
  GPIO_WriteBit(GPIOA,GPIO_Pin_11,Bit_SET); 
   for (myi=0; myi<0x777;myi++);   
  MagInit();
   //-----------following writes the value 0x60 to register 0x07 and
  // then reads it back. 
  MagChangeRegister(0x07,0x60);     
  MagChangeRegister(0x02,0x35);     
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
  GPIO_WriteBit(GPIOC,LED_GREEN_PIN,Bit_SET);   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_Init(GPIOC, &GPIO_InitStructure);   
  
/*  
while (1)
{
  GPIO_WriteBit(GPIOB,GPIO_Pin_3,Bit_RESET); 
  GPIO_WriteBit(GPIOB,GPIO_Pin_3,Bit_SET); 
  GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_RESET); 
  GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_SET);   
} 
*/  
  // BETH SPIRIT STUFF
  //----------------------------------
  // SPIRIT RADIO INTERFACE SETUP - BRAD'S 
  //---------------------------------
  __disable_interrupt(); 
  ThreadingInit();
  TimerInit(15);
  //--------------------------
  // Initialize all application state
  // Register the application thread if needed
  if (!appThreadRegistered)
  {
    appThreadRegistered = TRUE;
//    RegisterThread(RunApplication, 0x01);
  }
  // Register for packets
  RegisterPacketRxCallback(PacketRX);
  __enable_interrupt();   
  AppSpiritInit();  
	// Register the flash interface
//  SetPromiscuousMode(TRUE); 
  RegisterFlashInterface(APPSIZEALLOWED, FlashEraseScratch, FlashWriteScratch);   
  ButtonInit();
  
  __enable_interrupt(); 

  /* Start here ***************************************************************/
  /* GUI, this section is present only if LCD, DAC or serial communication is */
  /* enabled.                                                                 */
#if (defined(LCD_FUNCTIONALITY) | defined(DAC_FUNCTIONALITY) | defined(SERIAL_COMMUNICATION))
  UI_TaskInit(UI_INIT_CFG,wConfig,MC_NUM,oMCI,oMCT,s_fwVer);
//  UI_TaskInit(6,wConfig,MC_NUM,oMCI,oMCT,s_fwVer); 
#endif  
  /* End here******************************************************************/  
//              MCI_ExecSpeedRamp(oMCI[0],100,0);   
//              MCI_StartMotor(oMCI[0]);       
  newRPM = 0;               
  while(1)
  {        
 
    schedDone = 0;
    schedCount = 0;
  
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
        RunAllThreads(); 
        ButtonSample();
      }
      schedCount++;
    }   

 //   motorTemp = TSNS_GetAvTemp_C(TempSensr1);    
 //   Tempdac = motorTemp * 345;     //TT scale temp for DAC output, -20 to 95 C
 //   motorLibState = MCI_GetSTMState(oMCI[0]);
    
#ifdef SERIAL_COMMUNICATION
  
#else
    //-----------------------------
    // process the C-monster key 
    //----------------------------
    newKey = ButtonChanged();
    if (newKey != 0)
    {
      if ((newKey & KEY_CMONSTER)!= 0)
      {
//        if (cMonsterPressed != 0)
//        {
          SetPromiscuousMode(TRUE);  
          CM2SendPairRequest();
          timerPairingSession = TIMEPAIRSESSION; 
//        }
      }  
    }
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
    /* Start here ***********************************************************/
    /* GUI, this section is present only if LCD is enabled.                 */
    if (UI_IdleTimeHasElapsed())
    {  
      UI_SetIdleTime(UI_TASK_OCCURENCE_TICKS);
      UI_LCDRefresh();
    }
    /* End here**************************************************************/  
#endif

/********************************   EXAMPLE AREA ******************************/
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
/*****************************************************************************/
  }
}



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
  
  statusTable.Item.FirmwareVersion[0] = SW_VER0;
  statusTable.Item.FirmwareVersion[1] = SW_VER1;
  statusTable.Item.FirmwareVersion[2] = SW_VER2;
  statusTable.Item.FirmwareVersion[3] = SW_VER3;
  statusTable.Item.FirmwareVersion[4] = SW_VER4;
  statusTable.Item.FirmwareVersion[5] = SW_VER5;
  
  stemp = motorTemp>>8;
  statusTable.Item.MotorTempMSB = stemp;
  stemp = motorTemp & 0xFF;
  statusTable.Item.MotorTempLSB = stemp;
 
  stemp = busV>>8;
  statusTable.Item.BusVoltageMSB = stemp;
  stemp = busV & 0xFF;
  statusTable.Item.BusVoltageLSB = stemp;
  
  stemp = newRPM>>8;
  statusTable.Item.RPMMSB = stemp;
  stemp = newRPM & 0xFF;
  statusTable.Item.RPMLSB = stemp;  
  
  CM2SendStatusTable((uint8_t *)&statusTable);
}


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
