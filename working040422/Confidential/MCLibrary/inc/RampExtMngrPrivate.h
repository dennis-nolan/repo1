/**
  ******************************************************************************
  * @file    RampExtMngrPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains private definition of RampExtMngr class      
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
#ifndef __RAMPEXTMNGRPRIVATE_H
#define __RAMPEXTMNGRPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup RampExtMngr
  * @{
  */

/** @defgroup RampExtMngr_class_private_types RampExtMngr class private types
* @{
*/

/** 
  * @brief  RampExtMngr class members definition
  */
typedef struct
{
  int32_t wExt;                 /*!< Current state variable multiplied by 32768.*/
  int32_t wTargetFinal;         /*!< Backup of wTargetFinal to be applied in the
                                     last step.*/
  uint32_t wRampRemainingStep;	/*!< Number of steps remaining to complete the
                                     ramp.*/
  int32_t wIncDecAmount;	/*!< Increment/decrement amount to be applied to
                                     the reference value at each
                                     CalcTorqueReference.*/
  uint32_t wScalingFactor;      /*!< Scaling factor between output value and 
                                     its internal rapresentation.*/
}Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef RampExtMngrParams_t Params_t, *pParams_t;

/** 
  * @brief  Private RampExtMngr class definition 
  */
typedef struct
{
	Vars_t Vars_str; 		/*!< Class members container */
	pParams_t pParams_str;	/*!< Class parameters container */
	void *DerivedClass;		/*!< Pointer to derived class */
}_CREMNG_t, *_CREMNG;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__RAMPEXTMNGRPRIVATE_H*/
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
