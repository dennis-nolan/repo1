/**
  ******************************************************************************
  * @file    OneTouchTuningClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains interface of OneTouchTuning class      
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ONETOUCHTUNINGCLASS_H
#define __ONETOUCHTUNINGCLASS_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"
#include "PIRegulatorClass.h"
#include "SpeednPosFdbkClass.h"
#include "SpeednTorqCtrlClass.h"

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup OneTouchTuning
  * @{
  */
  
/** @defgroup OneTouchTuning_class_exported_types OneTouchTuning class exported types
* @{
*/

/** 
  * @brief  Public OneTouchTuning class definition 
  */
typedef struct COTT_t *COTT;

/** 
  * @brief OneTouchTuning class init structure type definition
  */
typedef struct
{
  CSPD oSpeedSensor;       /*!< Related speed sensor used. */
  pFOCVars_t pFOCVarsPtr;  /*!< Related structure of FOC vars.*/
  CPI oPIDSpeed;           /*!< Related speed controller used. */
  CSTC oSTC;               /*!< Related speed and torque controller used.*/
} OTT_Init_t, *pOTT_Init_t;

/** 
  * @brief  OneTouchTuning class parameters definition  
  */
typedef const struct
{
  uint16_t hOTTFrequencyHz;   /*!< Frequency expressed in Hz at which the user
                                   clocks the OTT calling OTT_MF method */
  float fBWdef;               /*!< Default bandwidth of speed regulator.*/
  float fMeasWin;             /*!< Duration of measurement window for speed and
                                   current Iq, expressed in seconds.*/
  uint8_t bPolesPairs;        /*!< Number of motor poles pairs.*/
  uint16_t hMaxPositiveTorque;/*!< Maximum positive value of motor 
                                   torque. This value represents 
                                   actually the maximum Iq current 
                                   expressed in digit.*/
  float fCurrtRegStabTimeSec; /*!< Current regulation stabilization time in seconds.*/
  float fOttLowSpeedPerc;     /*!< OTT lower speed percentage.*/
  float fOttHighSpeedPerc;    /*!< OTT higher speed percentage.*/
  float fSpeedStabTimeSec;    /*!< Speed stabilization time in seconds.*/
  float fTimeOutSec;          /*!< Timeout for speed stabilization.*/
  float fSpeedMargin;         /*!< Speed margin percentage to validate speed ctrl.*/
} OneTouchTuningParams_t, *pOneTouchTuningParams_t;
  
/**
* @}
*/

/** @defgroup OneTouchTuning_class_exported_methods OneTouchTuning class exported methods
  * @{
  */

/**
  * @brief  Creates an object of the class OneTouchTuning
  * @param  pOneTouchTuningParams pointer to an OneTouchTuning parameters structure
  * @retval COTT new instance of OneTouchTuning object
  */
COTT OTT_NewObject(pOneTouchTuningParams_t pOneTouchTuningParams);

/**
  * @brief  Initializes all the object variables, usually it has to be called
  *         once right after object creation. 
  * @param  this related object of class COTT.
  * @param  pOTT_Init pointer to the OTT init structure.
  * @retval none.
  */
void OTT_Init(COTT this, pOTT_Init_t pOTT_Init);

/**
  * @brief  It should be called before each motor restart. It initialize
  *         internal state of OTT.
  * @param  this related object of class COTT.
  * @retval none.
  */
void OTT_Clear(COTT this);

/**
  * @brief  It should be called at MF and execute the OTT algorithm.
  * @param  this related object of class COTT.
  * @retval none.
  */
void OTT_MF(COTT this);

/**
  * @brief  It should be called in START_RUN state. It begins the OTT procedure.
  * @param  this related object of class COTT.
  * @retval none.
  */
void OTT_SR(COTT this);

/**
  * @brief  Call this method before start motor to force new OTT procedure.
  * @param  this related object of class COTT.
  * @retval none.
  */
void OTT_ForceTuning(COTT this);

/**
  * @brief  It returns the nominal speed estimated by OTT.
  * @param  this related object of class COTT.
  * @retval uint32_t It returns the nominal speed estimated by OTT, it is a 
  *         floating point number codified into a 32bit integer.
  */
uint32_t OTT_GetNominalSpeed(COTT this);

/**
  * @brief  It returns the number of states of OTT.
  * @param  this related object of class COTT.
  * @retval uint8_t It returns the number of states of Selfcommissioning procedure.
  */
uint8_t OTT_GetSteps(COTT this);

/**
  * @brief  It returns the state of OTT.
  * @param  this related object of class COTT.
  * @retval uint8_t It returns the state of OTT.
  */
uint8_t OTT_GetState(COTT this);

/**
  * @brief  It returns TRUE if OTT procedure has been completed, FALSE otherwise.
  * @param  this related object of class COTT.
  * @retval bool It returns TRUE if OTT procedure has been completed, FALSE otherwise.
  */
bool OTT_IsSpeedPITuned(COTT this);

/**
  * @brief  It returns the nominal speed estimated by OTT in RPM.
  * @param  this related object of class COTT.
  * @retval float It returns the nominal speed estimated by OTT in RPM.
  */
float OTT_fGetNominalSpeedRPM(COTT this);

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* __ONETOUCHTUNINGCLASS_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
