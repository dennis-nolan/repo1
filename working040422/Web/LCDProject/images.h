/**
  ******************************************************************************
  * @file    images.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains images definition for LCD      
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

#ifndef __IMAGES_H_
#define __IMAGES_H_

#include "MC_Type.h"

typedef struct Image_s
{
  uint16_t *Buffer;
}Image_t;

extern Image_t LogoImage;
extern Image_t MotorImage;

#endif 
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
