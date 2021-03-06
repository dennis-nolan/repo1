/**
  ******************************************************************************
  * @file    LCDIExportedFunctions.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains the definitions of LCDI exported functions      
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
#ifndef __LCDIEXPORTEDFUNCTIONS_H
#define __LCDIEXPORTEDFUNCTIONS_H

enum {
  EF_LCDI_Init,
  EF_LCDI_UpdateMeasured,
  EF_LCDI_UpdateAll,
  EF_LCDI_Polling,
  EF_LCDI_NUMBERS
};

typedef void (*pLCDI_Init_t) (CUI,CUI,const char*);
typedef void (*pLCDI_Polling_t) (void);
typedef void (*pLCDI_UpdateMeasured_t) (CUI);
typedef void (*pLCDI_UpdateAll_t) (CUI oUI);

#endif /*__LCDIEXPORTEDFUNCTIONS_H*/
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
