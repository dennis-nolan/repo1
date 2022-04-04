//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: beep.c 
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Processor: STM32F103R
// TOOLS: IAR Workbench 
// DATE:
// CONTENTS: This file contains  
//------------------------------------------------------------------------------
// HISTORY: This file  
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#include "defs.h"

#include "motorHeader.h"
#include "beep.h"
//#include "driverRTCHeader.h"

#define BEEPVALUE  2000  //1500  //4000

uint16_t debounceBlockingTimer; 

TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
//extern TIM_TimeBaseInitTypeDef MC_TIMx_TimeBaseStructure;



//---------------------GLOBAL VARIABLES-----------------------------------
uint8_t tempo=60;

//---------------------LOCAL VARIABLES------------------------------------
  

 
 
//---------------------LOCAL FUNCTION PROTOTYPES--------------------------  
void Charge_Tune(void);
void ProgramEnterTune(void);
void Sucess_Tune(void);
void Fail_Tune(void);
void Error_Tune(void);
void Power_Pole_Tune(void);
void Scale_Tune(void);
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// ---------------------------GLOBAL FUNCTIONS ----------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   Beep
//------------------------------------------------------------------------------
// This function    
// divider represents the prescaler divide ratio generating the tone
// beat represents note length where 1000 is a whole note 
// 250 is a quarter note 125 is eighth etc. relative to tempo
//==============================================================================
void Beep(uint16_t beat, uint16_t divider)

{
  uint16_t millisecs;
  millisecs = ((60000/tempo*beat)/1000);

  
  if (divider != 0)  // if divider is 0 then play nothing
  {
//    divider = divider/4;
  // Unarm OCL
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
      
//    GPIO_WriteBit(GPIOC,GPIO_Pin_14,Bit_RESET);  // low active signal - this holds latch reset
      
    
    TIM_TimeBase_InitStructure.TIM_ClockDivision      = TIM_CKD_DIV1;       // 72MHz
    TIM_TimeBase_InitStructure.TIM_Period             = BEEPVALUE;
    TIM_TimeBase_InitStructure.TIM_Prescaler          = divider;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBase_InitStructure); // apply settings
    
    
//    while(Delay(General_Blocking,15) == FALSE); //keep trying if the delay fails     
    
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
  } 
//CLEAN  while(MicroDelay(Debounce_Blocking,millisecs) == FALSE); //keep trying if the delay fails
  debounceBlockingTimer = (millisecs+14)/15; 
  while (debounceBlockingTimer >0);

  if (divider != 0)  // if divider is 0 then play nothing
  {  
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
  }
  
} 

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   BeepInit
//------------------------------------------------------------------------------
// This function    
//==============================================================================
void BeepInit(void)
{

//    ChangePwrMode(OP_PWR);      // turns on power to the motor
    
      //-------------channel init-------------
    MotorChannelInit(Channel_1, Channel_Floating, 0);
    MotorChannelInit(Channel_2, Channel_Floating, 0);
    MotorChannelInit(Channel_3, Channel_Floating, 0);
    
        //delay
        DEADTIME_DELAY;
    
    MotorChannelInit(Channel_3, Channel_Low, 75); //65);  //was 5
    MotorChannelInit(Channel_2, Channel_Floating, 75); //65); //was 5
    MotorChannelInit(Channel_1, Channel_PWM_Complementary, 75); //65); //was 5    
  
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   BeepDeInit
//------------------------------------------------------------------------------
// This function    
//==============================================================================
void BeepDeInit(void)
{

//    ChangePwrMode(IDLE_PWR);      // turns on power to the motor    
  
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  StartBeep
//------------------------------------------------------------------------------
// This function    
//==============================================================================
void StartBeep(uint16_t divider)

{

  if (divider != 0)  // if divider is 0 then play nothing
  {
  // Unarm OCL
    TIM_CtrlPWMOutputs(TIM1, DISABLE);        
    
    TIM_TimeBase_InitStructure.TIM_ClockDivision      = TIM_CKD_DIV1;       // 72MHz
    TIM_TimeBase_InitStructure.TIM_Period             = BEEPVALUE;
    TIM_TimeBase_InitStructure.TIM_Prescaler          = divider;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBase_InitStructure); // apply settings
   
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
  } 
 
} 

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  EndBeep
//------------------------------------------------------------------------------
// This function    
//==============================================================================
void EndBeep(uint16_t divider)

{

  if (divider != 0)  // if divider is 0 then play nothing
  {  
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
  }
  
} 



void PlayTune(Tune which_tune)
{

    if (motorOn == FALSE)
    {  
    BeepInit();
    
    switch(which_tune)
    {
      case PPOLE:
        
        Power_Pole_Tune();
        break;

      case SUCS:
        
        Sucess_Tune();
        break;     
        
      case FAIL:
        
        Fail_Tune();
        break;   

      case ERR:
        
        Error_Tune();
        break;   

      case CHARGE:
        
        Charge_Tune();
        break;   
        
      case TUNE_PROGRAM_ENTER:
        
        ProgramEnterTune();
        break;   
      case SCALE:
        
        Scale_Tune();
        break;   
    }
    
    BeepDeInit();
    TIM1Reset();  // Reset the Timer1 after playing tune.     
    }
  
} 

void PlayTuneSuccess(void)
{
     if (motorOn == FALSE)
    {   
    BeepInit(); 
    tempo = 60;
    Beep(150,Csharp6);
    Beep(300,Gsharp6);
    BeepDeInit();
    TIM1Reset();  // Reset the Timer1 after playing tune.     
    }
} 

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// ---------------------------LOCAL FUNCTIONS ----------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   ProgramEnterTune
//------------------------------------------------------------------------------
// This function    
//==============================================================================
void ProgramEnterTune(void)
{
     tempo = 60;
    Beep(150,Csharp6);
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   Sucess_Tune
//------------------------------------------------------------------------------
// This function    
//==============================================================================
void Sucess_Tune(void)
{
  tempo = 60;
  Beep(150,Csharp6);
  Beep(300,Gsharp6);
}

void Fail_Tune(void)
{
    tempo = 60;
    Beep(150,Gsharp6);
    Beep(300,Csharp6);
}
void Error_Tune(void)
{
  tempo = 60;
  Beep(250,Csharp6);
  Beep(250,rest);
  Beep(250,Csharp6);
  Beep(250,rest);
  Beep(250,Csharp6);
  Beep(250,rest);
  Beep(250,Csharp6);
  Beep(250,rest);
}

void Power_Pole_Tune(void)
{
    tempo = 40;
  
    Beep(100,D5);
    Beep(200,D5);
    Beep(100,D5);
    Beep(200,E5);
    Beep(100,E5);
    Beep(200,G5);
    Beep(100,Asharp5);
    Beep(300,B5);
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   Charge_Tune
//------------------------------------------------------------------------------
// This function    
//==============================================================================
void Charge_Tune(void)
{
   tempo = 80;
  Beep(150,Gsharp5);
  Beep(150,Csharp6);
  Beep(150,F6);
  Beep(300,Gsharp6);
  Beep(150,F6);
  Beep(300,Gsharp6);
}

void Scale_Tune(void)
{
  tempo = 40;
  Beep(500,Cn5);
  Beep(500,Csharp5);
  Beep(500,D5);
  Beep(500,Dsharp5);
  Beep(500,E5);
  Beep(500,F5);
  Beep(500,Fsharp5);
  Beep(500,G5);
  Beep(500,Gsharp5);
  Beep(500,A5);
  Beep(500,Asharp5);
  Beep(500,B6);
  Beep(500,Csharp6);
  Beep(500,D6);
  Beep(500,Dsharp6);
  Beep(500,E6);
  Beep(500,F6);
  Beep(500,Fsharp6);
  Beep(500,G6);
  Beep(500,Gsharp6);
  Beep(500,A6);
  Beep(500,Asharp6);
  Beep(500,B6);
}
