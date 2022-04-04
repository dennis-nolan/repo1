//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: CONFIG.H
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "defs.h"
 
//---------------------GLOBAL VARIABLES--------------------------
#define VERSION_TABLEx41        0x01
#define MAX_TABLEx41            22
typedef union
{
  uint8_t Index[sizeof(PropulsionStatusTable)];
  PropulsionStatusTable Item;
}Tablex41; 



   
#define VERSION_TABLEx40       AVENGERSCORE_PROP_CONFIG_VER
#define MAXSIZE_TABLEx40       sizeof(AvengerConfigPropTable)
typedef union
{
  uint8_t Index[sizeof(AvengerConfigPropTable)];
  AvengerConfigPropTable Item;
}Tablex40; 



  //--------------- offsets in flash
#define TIMINGADJ_MSB 4
#define TIMINGADJ_LSB 5

#define TABLEx40_OFFSET 10 
#define OFFSET_SPEEDELBOW_MSB           (TABLEx40_OFFSET+2)
#define OFFSET_MAXAPPLICATIONSPEED_MSB  (TABLEx40_OFFSET+10)
#define OFFSET_NUMBEROFSTEPS            (TABLEx40_OFFSET+12)
#define OFFSET_MINSTEP                  (TABLEx40_OFFSET+15)
#define OFFSET_MOMENTARYTO              (TABLEx40_OFFSET+18)




extern PropulsionStatusTable tablex41; 
extern Tablex40 tablex40; 
extern uint8_t flashConfigUpdateFlag; 
#define MAX_FLASH_CONFIG_SIZE 10 + MAXSIZE_TABLEx40
extern uint8_t flashConfigImage[MAX_FLASH_CONFIG_SIZE];

//---------------------GLOBAL PROTOTYPES--------------------------
uint8_t ReadSettings(void);
uint8_t FlashUpdateValue(uint8_t which,uint8_t data); 
void FlashConfigUpdate(void);
void FlashConfigRead(void);
#endif
//end of config.h






