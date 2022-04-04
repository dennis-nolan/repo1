//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: MOTOR.H
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "defs.h"

 

//---------------------GLOBAL DEFINITIONS--------------------------
//pole position
typedef enum
{ DOCKED, UNDOCKED, BOTTOM } Pole_Position_State;

//setting enum
typedef enum
{ 
  Channel_Floating = 1,           //high and low side off
  Channel_Low,                    //high off, low on
  Channel_High,                   //high on, low off
  Channel_PWM_Non_Complementary,  //PWM high, low off
  Channel_PWM_Complementary       //PWM high, complement low with dead time
}MotorChannelSetting;

//channel enum
typedef enum
{ 
  Channel_1 = 1,
  Channel_2,
  Channel_3
}MotorChannel;

//---------------------GLOBAL VARIABLES--------------------------
extern Pole_Position_State   pole_position;
#define MAX_MOTOR_ACTIONS 3
#define MOTORACTION_AUTOUP              1
#define MOTORACTION_AUTODOWN            2
#define MOTORACTION_MANUALUP            3
#define MOTORACTION_MANUALDOWN          4
#define MOTORACTION_NONE                0
extern uint8_t motorActionQueue[MAX_MOTOR_ACTIONS];
extern uint8_t motorActionQueueOffset;
  //-------------------------------
  // turn off the motor - 
  // motor is turned off after the timerMotorOff expires or 
  // when motor_off is set to TRUE.
  //-------------------------------
extern uint16_t timerMotorOff;
extern int timerBoxDelay;
extern int timerRampUp;
#define TIMER_RAMP_UP   40
//short delay
#define DEADTIME_DELAY asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop")

 
//---------------------GLOBAL PROTOTYPES------------------------
void MotorChannelInit(MotorChannel channel, MotorChannelSetting setting, uint8_t PWM_Percent);
void MotorChannelTIMReset(MotorChannel channel);
void TIM1Reset(void);

#endif
//end of motor.h