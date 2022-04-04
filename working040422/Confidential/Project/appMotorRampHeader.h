//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: appMotorRampHeader.h
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// LAST MODIFIED:
//------------------------------------------------------------------------------
#ifndef __APPMOTORRAMPHEADER__
#define __APPMOTORRAMPHEADER__

#include "CM2CoreTypesHeader.h"
 

//---------------------GLOBAL DEFINITIONS--------------------------
extern uint8_t turnMotorOff; 
extern int32_t targetSpeedRPM;  
//---------------------GLOBAL VARIABLES--------------------------
 
//---------------------GLOBAL PROTOTYPES--------------------------
void TestPropulsionRampTask(void); 
void PropulsionRamp(int16_t finalSpeed);
void PropulsionRampTask(void);
void PropulsionStepRamp(int8_t step);
void MyStopMotor(void);
void MotorStartPropulsion(void);
int8_t PropulsionRPMToStep(int16_t rpm);
int16_t PropulsionStepRPMConversion(int8_t step);
void PropulsionRPMRamp(int16_t rpm);
#endif
 