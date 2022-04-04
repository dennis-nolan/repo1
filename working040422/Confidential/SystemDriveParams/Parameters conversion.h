/**
  ******************************************************************************
  * @file    Parameters conversion.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file includes the proper Parameter conversion on the base
  *          of stdlib for the first drive
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
#ifndef __PARAMETERS_CONVERSION_H
#define __PARAMETERS_CONVERSION_H

#if (defined(STM32F10X_MD) || defined (STM32F10X_LD) || defined(STM32F10X_MD_VL) || defined(STM32F10X_LD_VL) || defined(STM32F10X_HD))
  #include "Parameters conversion_F10x.h"
#elif defined(STM32F2XX)
  #include "Parameters conversion_F2xx.h"
#elif defined(STM32F30X)
  #include "Parameters conversion_F30x.h"
#elif defined(STM32F40XX)
  #include "Parameters conversion_F4xx.h"
#elif defined(STM32F0XX)
  #include "Parameters conversion_F0xx.h"
#endif

/* Common parameters conversions */
#if defined(OTF_STARTUP)
#define OTF_STARTUP_EN TRUE
#else
#define OTF_STARTUP_EN FALSE
#endif

#if defined(MOTOR_PROFILER)
 #undef MAX_APPLICATION_SPEED
 #define MAX_APPLICATION_SPEED 50000
 #undef F1
 #define F1 0
 #undef CORD_F1
 #define CORD_F1 0

/* This shall be managed by WB in future versions */
 #if (SPEED_REGULATOR_BANDWIDTH == 1)
  #define MP_RAMP_DURATION 15000
 #elif (SPEED_REGULATOR_BANDWIDTH == 10)
  #define MP_RAMP_DURATION 5000
 #elif (SPEED_REGULATOR_BANDWIDTH == 30)
  #define MP_RAMP_DURATION 1000
 #else
  #error "Specify here the MP_RAMP_DURATION"
  #define MP_RAMP_DURATION 1000
 #endif
#else
 #define MP_RAMP_DURATION 0 /* Dummy */
#endif

#endif /*__PARAMETERS_CONVERSION_H*/

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
