//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: config.C
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#include "defs.h"
#include "config.h"
#include "mctasks.h"
#include "spiritInterfaceHeader.h"

//---------------------GLOBAL DEFINITIONS--------------------------
PropulsionStatusTable tablex41;
Tablex40 tablex40;

#define FPAGE_SIZE 0x0800
#define CONFIG_ADDRESS_START    0x8002000
#define MAX_FLASH_CONFIG_SIZE 10 + MAXSIZE_TABLEx40
uint8_t flashConfigImage[MAX_FLASH_CONFIG_SIZE];
uint8_t flashConfigUpdateFlag = FALSE;

//---------------------LOCAL FUNCTION PROTOTYPES--------------------------



//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  FlashConfigUpdate
//------------------------------------------------------------------------------
// This function
//==============================================================================
void FlashConfigUpdate(void)
{
  uint8_t *ramSource;
  uint8_t j;
  uint32_t nbrOfPage,flashDestination,ltemp,ltemp2,ltemp3,ltemp4,EraseCounter;
  FLASH_Status FLASHStatus = FLASH_COMPLETE; // added when adding FLASH protect
  uint16_t PageSize = FPAGE_SIZE;
  //--------------------------------
  // clear the  area.
  //-------------------------------------------------------
  // Erase the needed pages where the user application will be loaded
  // Define the number of page to be erased
  nbrOfPage = FLASH_PagesMask(MAX_FLASH_CONFIG_SIZE);
  //----------------------
  // Flash unlock
  //---------------------
  FLASH_Unlock();
  //--------------------------------------
  // Erase the FLASH pages
  for (EraseCounter = 0; (EraseCounter < nbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
    IWDG_ReloadCounter();
    FLASHStatus = FLASH_ErasePage(CONFIG_ADDRESS_START + (PageSize * EraseCounter));
    if (FLASHStatus != FLASH_COMPLETE)
    {
     FLASHStatus = FLASH_ErasePage(CONFIG_ADDRESS_START + (PageSize * EraseCounter));
    }
  }
  ramSource = flashConfigImage;
  flashDestination = CONFIG_ADDRESS_START;
  for (j = 0;j <MAX_FLASH_CONFIG_SIZE ;j += 4)
  {
        //-----------------------------------
        // Program the data received into STM32F10x Flash
        ltemp = (*ramSource++);
        ltemp2 = (*ramSource++)<<8;
        ltemp3 = (*ramSource++)<<16;
        ltemp4 = (*ramSource++)<<24;

        ltemp |= (ltemp2|ltemp3|ltemp4);
        FLASH_ProgramWord(flashDestination,ltemp);
        IWDG_ReloadCounter();
        if (*(uint32_t*)flashDestination != ltemp)
        {

        }
        flashDestination += 4;
  }
  flashConfigUpdateFlag = FALSE;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  FlashConfigRead
//------------------------------------------------------------------------------
// This function
//==============================================================================
void FlashConfigRead(void)
{
  uint8_t *flashDestination,j;

  flashDestination = (uint8_t *)CONFIG_ADDRESS_START;
  for (j = 0;j <MAX_FLASH_CONFIG_SIZE ;j++)
  {
    flashConfigImage[j] = *flashDestination++;
  }
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  ConfigTablex40Defaults
//------------------------------------------------------------------------------
// This function
//==============================================================================
void ConfigTablex40Defaults(void)
{
    uint8_t i;

    tablex40.Item.tblVer = VERSION_TABLEx40;
    tablex40.Item.tblFlags = (0x00 | TABLEFLAG_WRITEABLE);
    tablex40.Item.plcPowerProp = DEFAULT_PLC_PWR_PROP;
    tablex40.Item.plcCarrierMode = DEFUALT_PLC_CARRIER_MODE;
    tablex40.Item.plcRXCarrierThreshold = DEFAULT_PLC_RXCARRIER_THRESHOLD;
    tablex40.Item.stowStepForwardDefault = DEFAULT_PROP_STOWSTEPFORWARD;
    tablex40.Item.stowStepReverseDefault = DEFAULT_PROP_STOWSTEPREVERSE;
    tablex40.Item.speedElbow = DEFAULT_SPEED_ELBOW;
    tablex40.Item.rampStepPreElbow =  DEFAULT_RAMP_STEP_PREELBOW;
    tablex40.Item.rampStepPostElbow = DEFAULT_RAMP_STEP_POSTELBOW;
    tablex40.Item.piLowSetPoint = 0;
    tablex40.Item.maxApplicationSpeed = DEFAULT_MAX_APPLICATION_SPEED;
    tablex40.Item.numSpeedStepsForward = DEFAULT_NUMBEROFSTEPS_FORWARD;
    tablex40.Item.stepSize = 0;
    tablex40.Item.voltageUnderLimit = DEFAULT_PROPULSION_VOLTAGE_UNDER_LIMIT;
    tablex40.Item.voltageOverLimit = DEFAULT_PROPULSION_VOLTAGE_OVER_LIMIT;
    tablex40.Item.momentaryTimeOut = DEFAULT_MOMENTARYTO;
    tablex40.Item.minStepForward = DEFAULT_PROP_MINFORWARD;
    tablex40.Item.minStepReverse =  DEFAULT_PROP_MINREVERSE;
    tablex40.Item.numSpeedStepsReverse = DEFAULT_NUMBEROFSTEPS_REVERSE;
    tablex40.Item.configBitGroundingOn = DEFAULT_GROUNDING_OFF;
  //-------------- LOAD table x40
  for (i=0;i<MAXSIZE_TABLEx40;i++)
  {
    flashConfigImage[TABLEx40_OFFSET+i] = tablex40.Index[i];
  }

    FlashConfigUpdate();
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  ReadSettings
//------------------------------------------------------------------------------
// This function handles allows all the RAM copies of EEPROM values to be loaded
// in.
//==============================================================================
uint8_t ReadSettings(void)
{
  uint8_t value,n,addr,i;
  uint8_t health,setupAll;
  uint32_t ltemp,ltemp2;
  int16_t itemp,itemp2;

  setupAll = FALSE;

  FlashConfigRead();
  if ((flashConfigImage[0] != 'Z')|| (flashConfigImage[1] != 'E')||
      (flashConfigImage[2] != 'R')|| (flashConfigImage[3] != 'O'))
  {
    //----------------------------------
    // TABLE does not exist
    // load in zero default
    flashConfigImage[0] = 'Z';
    flashConfigImage[1] = 'E';
    flashConfigImage[2] = 'R';
    flashConfigImage[3] = 'O';
    flashConfigImage[TIMINGADJ_MSB] = 0;
    flashConfigImage[TIMINGADJ_LSB] = 0;
    //-----------------------
    // PLACE in table x40
    ConfigTablex40Defaults();
  }
  else
  {
   //-----------------------
    // PLACE in table x40
    if (flashConfigImage[TABLEx40_OFFSET+0] != VERSION_TABLEx40)
    {
      ConfigTablex40Defaults();
    }
  }

  tablex41.tblVer = VERSION_TABLEx41;
  tablex41.tblFlags = (0x00 |TABLEFLAG_READ_ONLY);

  itemp = flashConfigImage[TIMINGADJ_MSB];
  itemp2 = flashConfigImage[TIMINGADJ_LSB];
  tablex41.TimingAdjustMSB = itemp;
  tablex41.TimingAdjustLSB = itemp2;
  itemp = itemp<<8;
  itemp |= itemp2;
  SetTimingAdjust(itemp);

  //-------------- LOAD table x40
  for (i=0;i<MAXSIZE_TABLEx40;i++)
  {
    tablex40.Index[i] = flashConfigImage[TABLEx40_OFFSET+i];
  }
  return 1;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  FlashUpdateValue
//------------------------------------------------------------------------------
// This function
//==============================================================================
uint8_t FlashUpdateValue(uint8_t which,uint8_t data)
{
  uint8_t status;
  status = 0;
  if(which < MAX_FLASH_CONFIG_SIZE)
  {
    flashConfigImage[which] = data;
    status = 1;
  }
  return status;
}

