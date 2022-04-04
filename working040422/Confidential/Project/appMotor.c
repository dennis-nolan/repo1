//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: appMotor.c
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
#include "MCInterfaceClass.h" 
#include "parameters conversion_f10x.h" 
#include "mctasks.h" 
#include "intrinsics.h"
#include "appMotorHeader.h"
#include "config.h" 
#include "uitask.h"
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
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

//---------------------GLOBAL DEFINITIONS-------------------------- 
uint16_t newRPM; 
uint16_t oldRPM; 

    
#define FIRMWARE_VERS "STM32 FOC SDK\0Ver.4.2.0"
const char s_fwVer[32] = FIRMWARE_VERS;
uint32_t wConfig[MC_NUM] = {UI_CONFIG_M1,UI_CONFIG_M2};

CMCI oMCI[MC_NUM];
CMCT oMCT[MC_NUM];  

  //--------------------------
  // MOTOR state machine object to check fault state
  //--------------------------
CTSNS TempSensr1;
CSTM STM1;
CVBS BusVoltSense1;
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
 
uint16_t motorOccurredFaults;
uint16_t motorCurrentFaults;
  

//---------------------LOCAL FUNCTION PROTOTYPES--------------------------  

void AppMotorInit(void)
{
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
    
 /* Start here ***************************************************************/
  /* GUI, this section is present only if LCD, DAC or serial communication is */
  /* enabled.                                                                 */
#if (defined(LCD_FUNCTIONALITY) | defined(DAC_FUNCTIONALITY) | defined(SERIAL_COMMUNICATION))
  UI_TaskInit(UI_INIT_CFG,wConfig,MC_NUM,oMCI,oMCT,s_fwVer);
#endif  
  /* End here******************************************************************/  
//              MCI_ExecSpeedRamp(oMCI[0],100,0);   
//              MCI_StartMotor(oMCI[0]);           
    
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
  motorOccurredFaults = MCI_GetOccurredFaults(oMCI[0]);
  motorCurrentFaults = MCI_GetCurrentFaults(oMCI[0]);

  stemp = motorOccurredFaults>>8;
  tablex51.Item.OccurredFaultsMSB = stemp;
  stemp = motorOccurredFaults & 0xFF;
  tablex51.Item.OccurredFaultsLSB = stemp;   

  stemp = motorCurrentFaults>>8;
  tablex51.Item.CurrentFaultsMSB = stemp;
  stemp = motorCurrentFaults & 0xFF;
  tablex51.Item.CurrentFaultsLSB = stemp;   
  
  tablex51.Item.MotorState = motorLibState; 
  
 //-------------------------------------------------- 
  stemp = motorTemp>>8;
  tablex51.Item.MotorTempMSB = stemp;
  stemp = motorTemp & 0xFF;
  tablex51.Item.MotorTempLSB = stemp;
 //---------------------------------------------------- 
  stemp = busV>>8;
  tablex51.Item.BusVoltageMSB = stemp;
  stemp = busV & 0xFF;
  tablex51.Item.BusVoltageLSB = stemp;
 //---------------------------------------------- 
  stemp = newRPM>>8;
  tablex51.Item.RPMSettingMSB = stemp;
  stemp = newRPM & 0xFF;
  tablex51.Item.RPMSettingLSB = stemp;  
  
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)==0)
  {
    tablex51.Item.OCLReading = 0;
  }
  else
  {
    tablex51.Item.OCLReading = 1;
  }
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  
//------------------------------------------------------------------------------
// This function 
//==============================================================================
void MyStopMotor(void)
{
     MCI_StopMotor(oMCI[0]); 
} 






