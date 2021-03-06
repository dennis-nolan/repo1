/**
  ******************************************************************************
  * @file    MotorControlProtocolPrivate.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   Private definition for MotorControlProtocol class
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
#ifndef __MCP_MCUI_H
#define __MCP_MCUI_H

typedef struct
{
  // Derived Class variables 
  CFCP oFCP;
  uint8_t BufferFrame[FRAME_MAX_BUFFER_LENGTH];
  uint8_t BufferSize;
  const char *s_fwVer;
  CUI oDAC;
}DVars_t,*pDVars_t;

typedef MCPParams_t DParams_t, *pDParams_t; 

typedef struct
{
  DVars_t DVars_str;
  pDParams_t pDParams_str;
}_DCMCP_t, *_DCMCP;

#endif /* __MCP_MCUI_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
