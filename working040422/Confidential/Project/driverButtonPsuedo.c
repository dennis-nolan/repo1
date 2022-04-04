//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: driverButtons.c
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Processor: STM32F103R
// TOOLS: IAR Workbench 
// DATE:
// CONTENTS: This file contains  
//------------------------------------------------------------------------------
// HISTORY: This file  
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#include "defs.h"
#include "driverbuttonPsuedo.h"
 

//---------------------GLOBAL VARIABLES-----------------------------------
short int cMonsterPressed = 0; 
short int cMonsterBusVPressed = 0;


//---------------------LOCAL VARIABLES------------------------------------
  //--------------------------
  //button check histories
uint16_t cMonsterHistory = 0; 
uint16_t cMonsterBusVHistory = 0;

uint8_t busVCount = 0;
uint8_t keyChanged; 
 
uint16_t timerBusVNoChange;
uint16_t timerBusVChange;


//---------------------LOCAL FUNCTION PROTOTYPES--------------------------  
 

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// ---------------------------FUNCTIONS ----------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

 

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   ButtonChanged
//------------------------------------------------------------------------------
// This function 
//==============================================================================
uint8_t ButtonChanged(void)
{
	uint8_t which; 
	which = keyChanged; 
	keyChanged = 0;	
	return which;
	
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   ButtonInit
//------------------------------------------------------------------------------
// This function Initializes registers to allow button interrupts
//==============================================================================
void ButtonInit(void)
{

  cMonsterPressed = 0; 
  cMonsterHistory = 0;
  busVCount = 0;
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   ButtonSample
//------------------------------------------------------------------------------
// This function Delays then checks the buttons and updates button booleans
//==============================================================================
void ButtonSample(void)
{
  short int prevKey; 
  uint8_t value; 
  
  //------------------------
  //check pins
  busV = VBS_GetAvBusVoltage_V(BusVoltSense1);
  value = 0; 
  if (busV <13)
  {
    value = 1;
  }
  cMonsterBusVHistory = (cMonsterBusVHistory << 1) + value;
  prevKey = cMonsterBusVPressed;  
  if ((cMonsterBusVHistory & 7) == 0)  
    cMonsterBusVPressed = 1;
  else
    cMonsterBusVPressed = 0;  
  if (prevKey != cMonsterBusVPressed)
  {
    keyChanged |= KEY_BUSVCMONSTER;
    //------------------VOLTAGE is changing 
    // so restart WINDOW timer .... count how many times 
    // it changes in the window. 
    // also time NOCHANGE period as well. 
    //---------------------------------
    timerBusVChange = TIME_5SECONDS; 
    timerBusVNoChange = TIME_5SECONDS; 
    busVCount++; 
  }	
  if (timerBusVNoChange == 0)
  {
      if(busVCount >= BUSV_COUNT)
      {
            cMonsterPressed = 1;
            keyChanged |= KEY_CMONSTER;	  
      }  
      else
      {
        cMonsterPressed = 0;
      }
      busVCount = 0; 
  }
  if (timerBusVChange == 0)
  {
      busVCount = 0; 
  }
  
}

