//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: appProductionInfo.c
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Processor: STM32
// TOOLS: IAR Workbench 
// DATE:
// CONTENTS: This file contains  
//------------------------------------------------------------------------------
// HISTORY: This file  
//------------------------------------------------------------------------------
// VERSION  DATE     PERSON   DESCRIPTION
//  00.01   03/28/19  EMH   FIRST release. integrated into EA103 and EA095
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#include <stdlib.h>
#include "productionInfo.h"

//---------------------GLOBAL DEFINITIONS--------------------------------


//---------------------LOCAL VARIABLES------------------------------------
  
static uint8_t productionVerified = 0; 
static uint8_t productionChecked = 0; 
//---------------------LOCAL FUNCTION PROTOTYPES--------------------------  
 

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// ---------------------------GLOBAL FUNCTIONS ----------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  AppProductionVerifyInfo
//------------------------------------------------------------------------------
//   
//==============================================================================
uint8_t AppProductionVerifyInfo(void)
{
  uint8_t status,checksum; 
  uint16_t i; 
  
  status = 0;
  if (productionChecked != 0)
  {
    status = productionVerified;
  }
  else
  {
    checksum = 0;
    for (i=0;i<1021;i++)
    {
      checksum += myflashConfigSection.Array[i];
    }
    if ((checksum == myflashConfigSection.Item.checksum)&&(myflashConfigSection.Item.header[0] == 'P'))
    {
      status = 1; 
      productionVerified = 1; 
    }
  }
  productionChecked = 1; 
  return status; 
}
