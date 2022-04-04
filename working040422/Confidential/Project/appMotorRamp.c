//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: appMotorRamp.c
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Processor: STM32F103R
// TOOLS: IAR Workbench
// DATE:
// CONTENTS: This file contains
//------------------------------------------------------------------------------
// HISTORY: This file
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#include "defs.h"
#include "version.h"
#include "commontypes.h"
#include "CM2CoreHeader.h"
#include "MCInterfaceClass.h"
#include "parameters conversion_f10x.h"
#include "mctasks.h"
#include "intrinsics.h"
#include "appMotorRampHeader.h"
#include "config.h"
#include "spiritInterfaceHeader.h"

extern int16_t hMeasuredSpeedReported;
//---------------------GLOBAL DEFINITIONS--------------------------

#define MAX_RPM_TABLE_ENTRY 21
const uint16_t RPMTimeTable[MAX_RPM_TABLE_ENTRY] = {
    0,0,607,399,303,240,  //0,1,2,3,4,5
    200,172,150,133,120,  //6,7,8,9,10
    109,100,93,85,80,     //11,12,13,14,15
    75,70,66,63,60};      //16,17,18,19,20

  //================================================
  //RAMP_STEP
  // the number to increase the RPM over 100s
  // the reason for this is to get some granularity over the 1 second number
  // so for 19.44 RPS/s = 1944
  //-------------------------------------
  // there is an ELBOW defined at 300 RPM
  // before the ELBOW the Step is RAMP_STEP_PREELBOW
  // after the ELBOW the Step is RAMP_STEP_POSTELBOW
  //-----------------------------------------------------
//#define SPEED_ELBOW     300
//#define RAMP_STEP_PREELBOW 19440
//#define RAMP_STEP_POSTELBOW 100000
#define RAMP_TIME_BASE 100  //100msec time base

extern CMCI oMCI[MC_NUM];

int32_t valueSpeedRPM = 0;
int32_t absoluteValueSpeedRPM = 0;
int32_t tempValueSpeedRPM = 0;
int32_t targetSpeedRPM = 0;
int32_t tempTargetSpeedRPM = 0;
int32_t absoluteTargetSpeedRPM = 0;
uint8_t propulsionRampOn = 0;
int32_t lastPropulsionRampSet = 0;
uint8_t newRamp = 0;
uint8_t turnMotorOff = 0;
unsigned short start_dcnq = 0;

#define MAX_RPM_REPORTED 20
uint8_t motorRPMReportedOffset = 0;
int16_t motorRPMReportedBuffer[MAX_RPM_REPORTED];
int16_t averageMotorRPMReported;
//---------------------LOCAL FUNCTION PROTOTYPES--------------------------

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  MotorStartPropulsion
//------------------------------------------------------------------------------
//
//==============================================================================
void MotorStartPropulsion(void)
{
//  State_t currentState;

  if (motorOn == 0)
  {
    start_dcnq = 255;

    if (MCI_StartMotor(oMCI[0]) == TRUE)
    {
            turnMotorOff = 0;
            motorOn = 1;
    }
    else
    {
//      currentState = MCI_GetSTMState(oMCI[0]);
      motorStartFailureCount++;
    }
  }
  turnMotorOff = 0;
}
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  MyStopMotor
//------------------------------------------------------------------------------
// This function
//==============================================================================
void MyStopMotor(void)
{
  if (MCI_StopMotor(oMCI[0]) == TRUE)
  {
     turnMotorOff = 0;
     motorOn = 0;
  }
  else
  {
    motorStopFailureCount++;
  }

}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  PropulsionRamp
//------------------------------------------------------------------------------
//
//==============================================================================
int16_t testSpeedRPM = 0;
void TestPropulsionRampTask(void)
{
    PropulsionRamp(testSpeedRPM);
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  PropulsionRPMToStep
//------------------------------------------------------------------------------
//
//==============================================================================
int8_t PropulsionRPMToStep(int16_t rpm)
{
  int16_t finalSpeed,itemp3,itemp4;
  uint16_t itemp,itemp2;
  int8_t myTemp;

  itemp = tablex40.Item.maxApplicationSpeed;
  itemp = itemp/6;
  //------------------------
  // the forward value is used .... as this should be the largest
  // vs the reverse.
  //------------------------
  itemp2 = tablex40.Item.numSpeedStepsForward;
  itemp = itemp/itemp2;

  itemp3 = rpm;
  itemp4 = itemp;
  itemp4 = itemp3/itemp4;

  if (itemp4 >= 0)
  {
      if (itemp4 > tablex40.Item.numSpeedStepsForward)
      {
        itemp4 = tablex40.Item.numSpeedStepsForward;
      }
  }
  else
  {
      if (itemp4 > tablex40.Item.numSpeedStepsReverse)
      {
        itemp4 = tablex40.Item.numSpeedStepsReverse;
        itemp4 = itemp4 * -1;
      }
  }
  myTemp = itemp4;
  return myTemp;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  PropulsionStepRamp
//------------------------------------------------------------------------------
//
//==============================================================================
int16_t PropulsionStepRPMConversion(int8_t step)
{
  int16_t finalSpeed,itemp3,itemp4;
  uint16_t itemp,itemp2;

  itemp = tablex40.Item.maxApplicationSpeed;
  itemp = itemp/6;
  //------------------------
  // the forward value is used .... as this should be the largest
  // vs the reverse.
  //------------------------
  itemp2 = tablex40.Item.numSpeedStepsForward;
  itemp = itemp/itemp2;
  itemp3 = step;
  itemp4 = itemp;
  itemp4 = itemp4 * itemp3;

  return itemp4;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  PropulsionStepRamp
//------------------------------------------------------------------------------
//
//==============================================================================
void PropulsionRPMRamp(int16_t rpm)
{
  targetSpeedRPM = rpm;
  propulsionRampOn = 1;
  newRamp = 1;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  PropulsionRamp
//------------------------------------------------------------------------------
//
//==============================================================================
void PropulsionRamp(int16_t finalSpeed)
{
  targetSpeedRPM = finalSpeed;
  propulsionRampOn = 1;
  newRamp = 1;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  PropulsionRampTask
//------------------------------------------------------------------------------
//
//==============================================================================
void PropulsionRampTask(void)
{
  int32_t myspeed;
  uint8_t change = 0;
  uint8_t targetReverse,actualReverse = 0;
  int16_t motorRPMReported,reportedRPM;

  if ((turnMotorOff != 0)&&(propulsionRampOn != 0)&&(IsTimingAdjustActive()==0))
  {
      if (valueSpeedRPM == 0)
      {
          MyStopMotor();
      }
  }

  if ((propulsionRampOn != 0)&&(motorOn != 0)&&(IsTimingAdjustActive()==0))
  {
    tablex41.ramping = 0;
    //-------------------------------
    // currently only moving in this direction
    // match RPM neg/pos
    if (targetSpeedRPM != valueSpeedRPM)
    {
      tablex41.ramping = 1;
      targetReverse = 0;
      actualReverse = 0;
      absoluteValueSpeedRPM = valueSpeedRPM;
      absoluteTargetSpeedRPM = targetSpeedRPM;
      if (targetSpeedRPM < 0)
      {
        targetReverse = 1;
        absoluteTargetSpeedRPM = -1 * targetSpeedRPM;
      }
      if (valueSpeedRPM < 0)
      {
        actualReverse = 1;
        absoluteValueSpeedRPM = -1 * valueSpeedRPM;
      }

       tempValueSpeedRPM = valueSpeedRPM * RAMP_TIME_BASE;
       tempTargetSpeedRPM = targetSpeedRPM * RAMP_TIME_BASE;
       absoluteValueSpeedRPM = tempValueSpeedRPM;
       if (tempValueSpeedRPM < 0)
       {
         absoluteValueSpeedRPM = -tempValueSpeedRPM;
       }
      //--------------------------
      //
      //--------------------------
      if (targetSpeedRPM <valueSpeedRPM)
      {
        //------------------------------
        // deaccelerating if in forward
        // accelerating if in reverse
        //------------------------------
        if (absoluteValueSpeedRPM< (tablex40.Item.speedElbow*RAMP_TIME_BASE))
        {
          tempValueSpeedRPM -= (tablex40.Item.rampStepPreElbow);
          if (tempValueSpeedRPM < tempTargetSpeedRPM)
          {
            tempValueSpeedRPM = tempTargetSpeedRPM;
          }
          myspeed = tempValueSpeedRPM/RAMP_TIME_BASE;
          valueSpeedRPM = myspeed;
          if (myspeed < 0)
          {
            myspeed = -myspeed;
            MCI_ExecSpeedRamp(oMCI[0],(SetSpeed(myspeed)),0);
          }
          else
          {
            valueSpeedRPM = myspeed;
            MCI_ExecSpeedRamp(oMCI[0],-(SetSpeed(myspeed)),0);
          }
        }
        else
        {
          if (absoluteValueSpeedRPM >= (tablex40.Item.rampStepPostElbow))
          {
            tempValueSpeedRPM -= (tablex40.Item.rampStepPostElbow);
            if (tempValueSpeedRPM < tempTargetSpeedRPM)
            {
              tempValueSpeedRPM = tempTargetSpeedRPM;
            }
            myspeed = tempValueSpeedRPM/RAMP_TIME_BASE;
            valueSpeedRPM = myspeed;
            if (myspeed < 0)
            {
              myspeed = -myspeed;
              MCI_ExecSpeedRamp(oMCI[0],(SetSpeed(myspeed)),0);
            }
            else
            {
              valueSpeedRPM = myspeed;
              MCI_ExecSpeedRamp(oMCI[0],-(SetSpeed(myspeed)),0);
            }
          }
          else
          {
            myspeed = 0;  //serious error if here
          }
        }
        lastPropulsionRampSet = myspeed;
        change = 1;
      }
      else
      {
        //----------------------------
        // accelerating in forward
        // deaccelerating in reverse
        if (absoluteValueSpeedRPM< (tablex40.Item.speedElbow*RAMP_TIME_BASE))
        {
          tempValueSpeedRPM += (tablex40.Item.rampStepPreElbow);
          if (tempValueSpeedRPM > tempTargetSpeedRPM)
          {
            tempValueSpeedRPM = tempTargetSpeedRPM;
          }
          myspeed = tempValueSpeedRPM/RAMP_TIME_BASE;
          valueSpeedRPM = myspeed;
          if (myspeed < 0)
          {
            myspeed = -myspeed;
            MCI_ExecSpeedRamp(oMCI[0],(SetSpeed(myspeed)),0);
          }
          else
          {
            valueSpeedRPM = myspeed;
            MCI_ExecSpeedRamp(oMCI[0],-(SetSpeed(myspeed)),0);
          }
        }
        else
        {
          tempValueSpeedRPM += (tablex40.Item.rampStepPostElbow);

          if (tempValueSpeedRPM >= (MY_MAX_APPLICATION_SPEED*RAMP_TIME_BASE))
          {
            tempValueSpeedRPM = MY_MAX_APPLICATION_SPEED*RAMP_TIME_BASE;
          }
          myspeed = tempValueSpeedRPM/RAMP_TIME_BASE;
          valueSpeedRPM = myspeed;
          if (myspeed < 0)
          {
            myspeed = -myspeed;
            MCI_ExecSpeedRamp(oMCI[0],(SetSpeed(myspeed)),0);
          }
          else
          {
            valueSpeedRPM = myspeed;
            MCI_ExecSpeedRamp(oMCI[0],-(SetSpeed(myspeed)),0);
          }
        }
        change = 1;
        lastPropulsionRampSet = myspeed;
      }
    }
    else
    {
        if (newRamp != 0)
        {
          if (targetSpeedRPM < 0)
          {
//            MCI_ExecSpeedRamp(oMCI[0],(SetSpeed(targetSpeedRPM)),0);
          }
          else
          {
//            MCI_ExecSpeedRamp(oMCI[0],-(SetSpeed(targetSpeedRPM)),0);
          }
        }
        else
        {
          reportedRPM = averageMotorRPMReported;
        }
    }
  }
#if TESTING_RAMP
  if ((motorOn == 0)&&(change != 0))
  {
    motorOn = 1;
    MCI_StartMotor(oMCI[0]);
  }
#endif
  newRamp = 0;
/*
   motorRPMReported = hMeasuredSpeedReported;;  //MCI_GetAvrgMecSpeed01Hz(oMCI[0]);

  if (motorRPMReportedOffset >=MAX_RPM_REPORTED)
  {
    motorRPMReportedOffset = 0;
  }
  motorRPMReportedBuffer[motorRPMReportedOffset++]=motorRPMReported;
  ltemp2 = 0;
  for (i=0;i<MAX_RPM_REPORTED;i++)
  {
    ltemp = motorRPMReportedBuffer[i];
    ltemp2 += ltemp;
  }
  ltemp2 = ltemp2/MAX_RPM_REPORTED;
  averageMotorRPMReported = ltemp2;
*/
}







