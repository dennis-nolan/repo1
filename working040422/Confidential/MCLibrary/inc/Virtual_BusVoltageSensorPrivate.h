/**
  ******************************************************************************
  * @file    Virtual_BusVoltageSensorPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains private definition of Virtual Vbus sensor class      
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
#ifndef __VIRTUAL_BUSVOLTAGESENSORPRIVATE_H
#define __VIRTUAL_BUSVOLTAGESENSORPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup BusVoltageSensor_Virtual
  * @{
  */

/** @defgroup Virtual_private_types Virtual Vbus sensor private types
* @{
*/

/** 
  * @brief  Redefinition of parameter structure
  */
typedef VirtualParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private Virtual Vbus sensor class definition 
  */
typedef struct
{
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_CVVBS_VBS_t, *_CVVBS_VBS;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__VIRTUAL_BUSVOLTAGESENSORPRIVATE_H*/
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
