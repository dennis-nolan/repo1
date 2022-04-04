//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: motor.c
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

#include "MCTuningClass.h"
#include "MCInterfaceClass.h"
#include "MCTasks.h"
#include "Parameters conversion.h" 

uint8_t Averaging; 

uint16_t speedPercent; 
//---------------------LOCAL VARIABLES------------------------------------
//keeps track of the current setting of the channel
MotorChannelSetting channel_1_setting = Channel_Floating;
MotorChannelSetting channel_2_setting = Channel_Floating;
MotorChannelSetting channel_3_setting = Channel_Floating;

short int previous_mode;     
uint8_t prev_spd = 0;
  //---------------------
  // System error state
  // - setup and used in 
extern CSTM STM1;  //declared in main.c
uint32_t  error;
uint32_t  currenterror;
uint32_t  previouserror;

bool lastMotorStartReturn;
//---------------------GLOBAL VARIABLES-----------------------------------
unsigned char TCFAULT;
 
//----------------
  // current run time for the motor for this session.
uint32_t run_time_count = 0;      // counter for run time
uint16_t motorRunElapse = 0; 

 
  //---------------------------------------
  // VALUES SHARED WITH THE MOTOR LIBRARY - IN FOCDriveClass.c
  // clDirection = ramp direction for clamp
  // 0 = stop, 1= up, -1 = down
  // set- up in RunBLDC
  // used in CheckAuto()
  //------------------------------
  // filterintegral - what is wierd is that this is initialized only in manual mode. 
signed char clDirection = 0;  // ramp direction for clamp
int32_t   filterintegral;  // must be static or global and initialized to zero
signed char myC1Direction; //copy of c1Direction - found this was getting a 0xff value some times 

bool auto_op;                     // motor is running in auto mode
bool man_op;                      // motor is running in manual mode


//---------------------LOCAL FUNCTION PROTOTYPES--------------------------  

//keeps track of the current duty cycle of the channel
uint8_t channel_1_duty_cycle_percent = 0x00;
uint8_t channel_2_duty_cycle_percent = 0x00;
uint8_t channel_3_duty_cycle_percent = 0x00;

  //-----------------------------------
  //init structures from MC App - used in channel intiialization
extern TIM_TimeBaseInitTypeDef MC_TIMx_TimeBaseStructure;
extern TIM_OCInitTypeDef CH1_TIMx_OCInitStructure, CH2_TIMx_OCInitStructure,
                        CH3_TIMx_OCInitStructure;


//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// -----------------------------GLOBAL FUNCTIONS -------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  
/*************************************************************************
 * Function Name: MotorChannelInit
 * Parameters: channel - the output channel to change
 *             setting - what setting to set the channel to
 *             PWM_percent - what PWM duty cycle to use if PWMing
 *
 * Return: none
 *
 * Description: Initializes registers to set up pins and
 *              timer for motor control on the specified
 *              channel.
 *
 *************************************************************************/
void MotorChannelInit(MotorChannel channel, MotorChannelSetting setting, uint8_t PWM_Percent)
{
  uint16_t high_pin, low_pin;
  TIM_BDTRInitTypeDef TIM_BDTR_InitStructure;
  MotorChannelSetting old_setting;
 TIM_OCInitTypeDef TIM_OC_InitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //pick pins based on the channel selection
  if (channel == Channel_1)
  {
    //averaging
    PWM_Percent = (PWM_Percent + Averaging * channel_1_duty_cycle_percent) / (Averaging+1);//----------------------------
      
    //if there is no change then just return
    if (channel_1_setting == setting && channel_1_duty_cycle_percent == PWM_Percent)
      return;
    
    //save old setting
    old_setting = channel_1_setting;
    
    //set pins to set up
    high_pin = GPIO_Pin_8;
    low_pin = GPIO_Pin_13;
  }
  else if (channel == Channel_2)
  {
    //averaging
    PWM_Percent = (PWM_Percent + Averaging * channel_2_duty_cycle_percent) / (Averaging+1);
    //----------------------------
      
    //if there is no change then just return
    if (channel_2_setting == setting && channel_2_duty_cycle_percent == PWM_Percent)
      return;
    
    //save old setting
    old_setting = channel_2_setting;   
    //set pins to set up
    high_pin = GPIO_Pin_9;
    low_pin = GPIO_Pin_14; 
  }
  else if (channel == Channel_3)
  {
    //averaging
    PWM_Percent = (PWM_Percent + Averaging * channel_3_duty_cycle_percent) / (Averaging+1);//----------------------------
    
    //if there is no change then just return
    if (channel_3_setting == setting && channel_3_duty_cycle_percent == PWM_Percent)
      return;
    
    //save old setting
    old_setting = channel_3_setting;  
    //set pins to set up
    high_pin = GPIO_Pin_10;
    low_pin = GPIO_Pin_15; 
  }
  else
    return; //invalid
  
  
  if (setting == Channel_Floating || setting == Channel_Low || setting == Channel_High)
  {
    //regular push/pull output
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //speed
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // push/pull
    
    //positive output
    GPIO_InitStructure.GPIO_Pin = high_pin;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //negative output
    GPIO_InitStructure.GPIO_Pin = low_pin;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    if (setting == Channel_Floating)
    {
      //high and low off
      GPIO_WriteBit(GPIOA,high_pin,Bit_RESET);
      GPIO_WriteBit(GPIOB,low_pin,Bit_RESET);
    }
    else if (setting == Channel_Low)
    {
      //if needed add dead time  
      if (old_setting != Channel_Floating)
      {
        //float
        GPIO_WriteBit(GPIOA,high_pin,Bit_RESET);
        GPIO_WriteBit(GPIOB,low_pin,Bit_RESET);
        
        //delay
        DEADTIME_DELAY;
      }
      
      //high off, low on
      GPIO_WriteBit(GPIOA,high_pin,Bit_RESET);
      GPIO_WriteBit(GPIOB,low_pin,Bit_SET);
    }
    else //Channel_High
    {
      //if needed add dead time
      if (old_setting != Channel_Floating)
      {
        //float
        GPIO_WriteBit(GPIOA,high_pin,Bit_RESET);
        GPIO_WriteBit(GPIOB,low_pin,Bit_RESET);
        
        //delay
        DEADTIME_DELAY;
      }
      
      //high on, low off
      GPIO_WriteBit(GPIOA,high_pin,Bit_SET);
      GPIO_WriteBit(GPIOB,low_pin,Bit_RESET);
    }
    
    //no need to set timer for this channel setting
    
  }
  else if (setting == Channel_PWM_Non_Complementary || setting == Channel_PWM_Complementary)
  {
    //if needed add dead time
    if (old_setting != Channel_Floating)
    {
      //regular push/pull output
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //speed
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // push/pull
      
      //positive output
      GPIO_InitStructure.GPIO_Pin = high_pin;
      GPIO_Init(GPIOA, &GPIO_InitStructure);
      
      //negative output
      GPIO_InitStructure.GPIO_Pin = low_pin;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
    
      //float
      GPIO_WriteBit(GPIOA,high_pin,Bit_RESET);
      GPIO_WriteBit(GPIOB,low_pin,Bit_RESET);
      
      //delay
      DEADTIME_DELAY;
    }
    
    if (setting == Channel_PWM_Non_Complementary)
    {
      //--------High side--------
      //alternate function push/pull
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
      
      //positive output
      GPIO_InitStructure.GPIO_Pin = high_pin;
      GPIO_Init(GPIOA, &GPIO_InitStructure);
      
      //Non-complementary PWM with no dead time
      TIM_OC_InitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
      TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
      TIM_OC_InitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
      TIM_OC_InitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
      TIM_OC_InitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
      TIM_OC_InitStructure.TIM_OutputState  = TIM_OutputState_Enable;
      TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//orig      TIM_OC_InitStructure.TIM_Pulse        = 1500 / 100 * PWM_Percent; //set duty cycle
      TIM_OC_InitStructure.TIM_Pulse        = 1500 / 100 * PWM_Percent; //set duty cycle
     
      //---------Low side--------
      //regular push/pull output
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //speed
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // push/pull
      
      //negative output
      GPIO_InitStructure.GPIO_Pin = low_pin;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      
      //low off
      GPIO_WriteBit(GPIOB,low_pin ,Bit_RESET);
    }
    else //Channel_PWM_Complementary
    {
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //speed
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //alternate function push/pull
      
      //positive outputs
      GPIO_InitStructure.GPIO_Pin = high_pin;
      GPIO_Init(GPIOA, &GPIO_InitStructure);
      
      //negative outputs
      GPIO_InitStructure.GPIO_Pin = low_pin;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      
      //Complementary PWM with dead time
      TIM_OC_InitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
      TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
      TIM_OC_InitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
      TIM_OC_InitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
      TIM_OC_InitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High; //TIM_OCNPolarity_Low;--------------------------------
      TIM_OC_InitStructure.TIM_OutputState  = TIM_OutputState_Enable;
      TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
//      TIM_OC_InitStructure.TIM_Pulse        = 1500 / 100 * PWM_Percent; //set duty cycle       
      TIM_OC_InitStructure.TIM_Pulse        = 1500 / 100 * PWM_Percent; //set duty cycle       
      
      //dead time
      TIM_BDTRStructInit(&TIM_BDTR_InitStructure);
      TIM_BDTR_InitStructure.TIM_DeadTime = 12; // ~500ns dead time?
      TIM_BDTR_InitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
      TIM_BDTR_InitStructure.TIM_Break = TIM_Break_Disable;
      TIM_BDTR_InitStructure.TIM_OSSRState = TIM_OSSRState_Disable;
      TIM_BDTRConfig(TIM1, &TIM_BDTR_InitStructure);
    }
    
    //channel specific settings
    if (channel == Channel_1)
    {
      TIM_OC1Init(TIM1, &TIM_OC_InitStructure); //apply settings
      TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
      TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable); //Enabled
    }
    else if (channel == Channel_2)
    {
      TIM_OC2Init(TIM1, &TIM_OC_InitStructure); //apply settings
      TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
      TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable); //Enabled
    }
    else //channel 3
    {
      TIM_OC3Init(TIM1, &TIM_OC_InitStructure); //apply settings
      TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
      TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable); //Enabled
    }
  }
  
  
  //set state
  if (channel == Channel_1)
  {
    channel_1_setting = setting;
    channel_1_duty_cycle_percent = PWM_Percent;
  }
  else if (channel == Channel_2)
  {
    channel_2_setting = setting;
    channel_2_duty_cycle_percent = PWM_Percent;
  }
  else if (channel == Channel_3)
  {
    channel_3_setting = setting;
    channel_3_duty_cycle_percent = PWM_Percent;
  }
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION: TIM1Reset/------------------------------------------------------------------------------
// Re-initialize Timer 1 settings to initial MC settings
//              timer for motor control on the specified
//==============================================================================
void TIM1Reset(void)
{ 
  
    MotorChannelTIMReset(Channel_1);
    MotorChannelTIMReset(Channel_2);
    MotorChannelTIMReset(Channel_3);  
    TIM_TimeBaseInit(TIM1, &MC_TIMx_TimeBaseStructure); // apply settings    
}
 
  
/*************************************************************************
 * Function Name: MotorChannelTIMReset
 * Parameters: channel - the output channel to change
 *             setting - what setting to set the channel to
 *             PWM_percent - what PWM duty cycle to use if PWMing
 *
 * Return: none
 *
 * Description: Initializes registers to set up pins and
 *              timer for motor control on the specified
 *              channel.
 *
 *************************************************************************/
void MotorChannelTIMReset(MotorChannel channel)
{
  uint16_t high_pin, low_pin;
//  TIM_BDTRInitTypeDef TIM_BDTR_InitStructure;
//  MotorChannelSetting old_setting;
 GPIO_InitTypeDef GPIO_InitStructure;
  
  //pick pins based on the channel selection
  if (channel == Channel_1)
  {
    //set pins to set up
    high_pin = GPIO_Pin_8;
    low_pin = GPIO_Pin_13;
  }
  else if (channel == Channel_2)
  {
    //set pins to set up
    high_pin = GPIO_Pin_9;
    low_pin = GPIO_Pin_14;   
  }
  else if (channel == Channel_3)
  { 
    //set pins to set up
    high_pin = GPIO_Pin_10;
    low_pin = GPIO_Pin_15;  
  }  
  
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //speed
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //alternate function push/pull
      
      //positive outputs
      GPIO_InitStructure.GPIO_Pin = high_pin;
      GPIO_Init(GPIOA, &GPIO_InitStructure);
      
      //negative outputs
      GPIO_InitStructure.GPIO_Pin = low_pin;
      GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
    //channel specific settings
    if (channel == Channel_1)
    {
      TIM_OC1Init(TIM1, &CH1_TIMx_OCInitStructure); //apply settings
      TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
      TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable); //Enabled
    }
    else if (channel == Channel_2)
    {
      TIM_OC2Init(TIM1, &CH2_TIMx_OCInitStructure); //apply settings
      TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
      TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable); //Enabled
    }
    else //channel 3
    {
      TIM_OC3Init(TIM1, &CH3_TIMx_OCInitStructure); //apply settings
      TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
      TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable); //Enabled
    }
  
}



 
 
