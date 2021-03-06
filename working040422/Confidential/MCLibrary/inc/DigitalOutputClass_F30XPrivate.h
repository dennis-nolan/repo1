/**
  ******************************************************************************
  * @file    DigitalOutputPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains private definition of DigitalOutput class      
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
#ifndef __DIGITALOUTPUTPRIVATE_H
#define __DIGITALOUTPUTPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */

/** @addtogroup DigitalOutput
  * @{
  */

/** @defgroup DigitalOutput_class_private_types DigitalOutput class private types
* @{
*/

/** 
  * @brief  DigitalOutput class members definition
  */
typedef struct
{
  DOutputState_t OutputState;
}Vars_t,*pVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef DigitalOutputParams_t Params_t, *pParams_t;

/** 
  * @brief  Private DigitalOutput class definition 
  */
typedef struct
{
	Vars_t Vars_str; 		/*!< Class members container */
	pParams_t pParams_str;	/*!< Class parameters container */
}_CDOUT_t, *_CDOUT;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__DIGITALOUTPUTPRIVATE_H*/

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
