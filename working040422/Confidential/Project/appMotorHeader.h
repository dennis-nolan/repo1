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
  
//---------------------GLOBAL VARIABLES--------------------------
 
//---------------------GLOBAL PROTOTYPES--------------------------
void SysMonitorTask(void);
void MyStopMotor(void);
#endif
 