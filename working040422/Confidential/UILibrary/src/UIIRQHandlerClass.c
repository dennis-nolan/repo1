/**
  ******************************************************************************
  * @file    UIIRQClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   User Interface IRQ handler class
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

/* Includes ------------------------------------------------------------------*/

#include "UIIRQHandlerClass.h"
#include "UIIRQHandlerPrivate.h"

/* Definition of MC IRQ Table */
#define MAX_UI_IRQ_NUM 1
_CUIIRQ oUI_IRQTable[MAX_UI_IRQ_NUM];

void Set_UI_IRQ_Handler(unsigned char bIRQAddr, _CUIIRQ oIRQ)
{
  oUI_IRQTable[bIRQAddr] = oIRQ;
}

void* Exec_UI_IRQ_Handler(unsigned char bIRQAddr, unsigned char flag, unsigned short rx_data)
{  
  return(oUI_IRQTable[bIRQAddr]->pIRQ_Handler((void*)(oUI_IRQTable)[bIRQAddr], flag, rx_data));
}

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
