/**
  @page Speed regulation with external potentiometer

  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    Potentiometer/readme.txt 
  * @author  Motor Control Team - System Lab 
  * @version 4.1.0
  * @date    26-May-2015 14:11
  * @brief   How to regulate the motor speed using an external potentiometer 
             connected to ADC channel of microcontroller
  ******************************************************************************
  *
  * Licensed under ST Liberty SW License Agreement V2, (the "License");
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
  @endverbatim

@par Example Description 

This example describes how to use the API of Motor Control library to regulate the
motor speed through an external potentiometer connected to one ADC channel of 
microcontroller (MCU).

The ADC is configured to convert continuously from ADC_Channel_14 on PC4 and the 
variable "potentiometer_value" stores the information each time an end of conversion 
occurs. The converted value set the mechanical speed.

MC-Lib API used:

- MCI_ExecSpeedRamp()
- MCI_StartMotor()
- MC_RegularConvState()
- MC_RequestRegularConv()
- MC_GetRegularConv()
- MCI_StopMotor()

@par Directory contents 

  - Potentiometer/potentiometer.c     Example configuration file

@par Hardware and Software environment 

  - This example runs on STM32F1xx Devices.
  
  - This example has been tested with STM3210E-EVAL and MB459 with Shinano motor and 
    it can be easily tailored to any other development board.

  - STM3210E-EVAL Set-up
    - Use the Potentiometer (RV1) of the Eval board (connected to PC.04). 


@par How to use it ? 

In order to make the program work, you must do the following:

 - Use the ST Motor Control Workbench application to configure this
   STM32 PMSM FOC library v4.0 opening the WB file created for STM3210E-EVAL 
 - From IAR application open the workspace file named as "STM32F10x_Example"
 - Select the right example from drop box menu
 - Press F8 to batch-build the entire workspace (select a proper project configuration, 
   according to the uC part in use);
 - Read the messages window and download the firmware;
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
