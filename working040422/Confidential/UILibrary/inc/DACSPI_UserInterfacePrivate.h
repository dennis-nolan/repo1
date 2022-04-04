/**
  ******************************************************************************
  * @file    DACSPI_UserInterfacePrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains private definition of DACSPI class      
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
#ifndef __DACSPI_USERINTERFACEPRIVATE_H
#define __DACSPI_USERINTERFACEPRIVATE_H

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */
  
/** @addtogroup DAC_SPI_UserInterface
  * @{
  */

/** @defgroup DACSPI_private_types DACSPI private types
* @{
*/

#define DAC_CH_NBR 2
#define DAC_CH_USER 2

/** 
  * @brief  DACSPI class members definition 
  */
typedef struct
{
  MC_Protocol_REG_t bChannel_variable[DAC_CH_NBR]; /*!< Array of codes of 
                                                        variables to be provided
                                                        in out to the related 
                                                        channel.*/
  int16_t hUserValue[DAC_CH_USER];                 /*!< Array of user defined 
                                                        DAC values.*/
}DVars_t,*pDVars_t;

/** 
  * @brief  Redefinition of parameter structure
  */
typedef DACSPIParams_t DParams_t, *pDParams_t; 

/** 
  * @brief Private DACSPI class definition 
  */
typedef struct
{
	DVars_t DVars_str;			/*!< Derived class members container */
	pDParams_t pDParams_str;	/*!< Derived class parameters container */
}_DCDACS_UI_t, *_DCDACS_UI;
/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

#endif /*__DACSPI_USERINTERFACEPRIVATE_H*/

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
