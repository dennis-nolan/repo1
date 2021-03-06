/**
  ******************************************************************************
  * @file    UIIRQHandlerClass.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains interface of UI IRQ handler class      
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
#ifndef __UI_IRQHANDLERCLASS_H
#define __UI_IRQHANDLERCLASS_H

/** @addtogroup STM32_PMSM_UI_Library
  * @{
  */

/** @addtogroup UserInterface_IRQHandler
  * @{
  */
  
/** @defgroup UserInterface_class_exported_types UserInterface class exported types
* @{
*/

/* MC IRQ Addresses */
#define UI_IRQ_USART            0u  /*!< Reserved for UIClass serial communication.*/ 

/**
* @}
*/

/** @defgroup UserInterface_class_exported_methods UserInterface class exported methods
  * @{
  */

/*Methods*/
void* Exec_UI_IRQ_Handler(unsigned char bIRQAddr, unsigned char flag, unsigned short rx_data);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __UI_IRQHANDLERCLASS_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
