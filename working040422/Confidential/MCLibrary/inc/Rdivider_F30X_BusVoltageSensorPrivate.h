/**
  ******************************************************************************
  * @file    Rdivider_BusVoltageSensorPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains private definition of Rdivider class      
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
#ifndef __RDIVIDER_BUSVOLTAGESENSORPRIVATE_H
#define __RDIVIDER_BUSVOLTAGESENSORPRIVATE_H

/** @addtogroup STM32_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup BusVoltageSensor_Rdivider
  * @{
  */

#define BUFF_MAX 6 /* Maximum buffer size to compute average value.*/

/** @defgroup Rdivider_private_types Rdivider private types
* @{
*/

/** 
  * @brief  Rdivider class members definition 
  */
typedef struct
{
  CPWMC oPWMnCurrentSensor;     /*! CPWMC object to be used for regular 
                                    conversions*/
  uint16_t vbuffer[BUFF_MAX];   /*!< Buffer used to compute average value.*/
  uint8_t vindex;               /*!< Index of last stored element in the average buffer.*/
  uint8_t nelem;                /*!< Number of stored elements in the average buffer.*/
} DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef RdividerParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private Rdivider class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_CRVBS_VBS_t, *_CRVBS_VBS;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__RDIVIDER_BUSVOLTAGESENSORPRIVATE_H*/
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
