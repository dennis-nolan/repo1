/**
  ******************************************************************************
  * @file    MCTasksFunction.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file implementes user functions for High/Medium freq task 
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
#include "MC.h"  
#include "MCTaskFunction.h"
#include "stm32f10x_tim.h"

/*Create the CMCT local reference: CMCT oMCT*/
static CMCT oMCT;
int16_t value_Speed_RPM = 0;

void TSK_MediumFrequencyTaskM1Completed()
{
   /* It is editable by the user */
}

void TSK_HighFrequencyTaskCompleted()
{
   /* It is editable by the user */
}
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
