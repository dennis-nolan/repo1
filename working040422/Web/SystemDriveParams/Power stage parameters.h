/**
  ******************************************************************************
  * @file    Power stage parameters.h
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains motor parameters needed by STM32 PMSM MC FW  
  *                 library v4.1.0
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
#ifndef __POWER_STAGE_PARAMETERS_H
#define __POWER_STAGE_PARAMETERS_H

/************* PWM Driving signals section **************/
#define PHASE_UH_POLARITY             H_ACTIVE_HIGH 
#define PHASE_VH_POLARITY             H_ACTIVE_HIGH 
#define PHASE_WH_POLARITY             H_ACTIVE_HIGH 

#define HW_COMPLEMENTED_LOW_SIDE      ENABLE 

#define PHASE_UL_POLARITY             L_ACTIVE_HIGH 
#define PHASE_VL_POLARITY             L_ACTIVE_HIGH 
#define PHASE_WL_POLARITY             L_ACTIVE_HIGH 

#define HW_DEAD_TIME_NS              500 /*!< Dead-time inserted 
                                                         by HW if low side signals 
                                                         are not used */
/********** Inrush current limiter signal section *******/
#define INRUSH_CURR_LIMITER_POLARITY  DOUT_ACTIVE_HIGH 

/******* Dissipative brake driving signal section *******/
#define DISSIPATIVE_BRAKE_POLARITY    DOUT_ACTIVE_HIGH 

/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR      0.0526 /*!< It expresses how 
                                                       much the Vbus is attenuated  
                                                       before being converted into 
                                                       digital value */
#define NOMINAL_BUS_VOLTAGE_V         11 
/******** Current reading parameters section ******/
/*** Topology ***/
#define THREE_SHUNT
/* #define SINGLE_SHUNT */
/* #define ICS_SENSORS */


#define RSHUNT                        0.330 

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN            1.53 

/*** Noise parameters ***/
#define TNOISE_NS                     700 
#define TRISE_NS                      700 
   
/*********** Over-current protection section ************/   
#define OVERCURR_FEEDBACK_POLARITY       EMSTOP2_ACTIVE_LOW 
#define OVERCURR_PROTECTION_HW_DISABLING  DOUT_ACTIVE_HIGH 
   
/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V                          1.055 /*!< in Volts */
#define T0_C                          25.0 /*!< in Celsius degrees */
#define dV_dT                         0.023 /*!< V/Celsius degrees */
#define T_MAX                         110 /*!< Sensor measured 
                                                       temperature at maximum 
                                                       power stage working 
                                                       temperature, Celsius degrees */

/*** Motor Profiler ***/
#define CALIBRATION_FACTOR            0.00 /*!< Power stage calibration factor. 
                                                            (Measured experimentally).*/
#define BUS_VOLTAGE_CONVERSION_FACTOR 62.700 /*!< Bus voltage conversion factor.*/

#endif /*__POWER_STAGE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
