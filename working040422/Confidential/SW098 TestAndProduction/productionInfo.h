//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: PRODUCTIONINFO_H
//------------------------------------------------------------------------------
// VERSION  DATE     PERSON   DESCRIPTION
//  00.01   03/28/19  EMH   FIRST release. integrated into EA103 and EA095
//  00.02   04/03/19  EMH   Added the testID[8] entry 
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#ifndef PRODUCTIONINFO_H
#define PRODUCTIONINFO_H

#include "stm32f10x.h"
//#include "stm32l4xx.h"
//---------------------GLOBAL DEFINITIONS-------------------------- 
typedef  union
{
  uint8_t Array[1024];
  __packed struct
  {
             uint8_t reserved[995];
             uint8_t testID[8];
             uint8_t lotCode[5];
             uint8_t rev[2];
             uint8_t model[2];
             uint8_t month; 
             uint8_t day;
             uint8_t year[2];
             uint8_t unused[2];
             uint8_t appMode;
             uint8_t toolVer[2];
             uint8_t checksum;
             uint8_t header[2];
  }Item;
} ToolLoad;

//---------------------GLOBAL VARIABLES-------------------------- 
extern ToolLoad myflashConfigSection;
//---------------------GLOBAL PROTOTYPES--------------------------
uint8_t AppProductionVerifyInfo(void);

#endif 