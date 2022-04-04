/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "Timebase.h"
#include "defs.h"
#include "driverButtonPsuedo.h"
#include "mctasks.h"
#include "driverFlash.h"
#include "spiritInterfaceHeader.h"
#include "appTestsupport.h"
#include "productionInfo.h"
#include "beep.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

#ifndef FREE_RTOS
/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}
#endif

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

#ifndef FREE_RTOS
/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}
#endif

#ifndef FREE_RTOS
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

  //-----------------------
  // 10 msec timers.
extern uint16_t dupTimer;
uint8_t spiritWdog;
uint8_t msec;
uint8_t spiritInterfaceTimer;

  //----------10msec timers
uint8_t timerSysMonitor;

  //----------10msec timers
uint8_t timerSupMonitor;

  //------------------------
  // 10 msec sup timer that requires
  // a packet over PLC to determine if loss
  // of communications with BASE.
  // base packets to anyone on PLC will
  // refresh this timer.
  // only used in MODE_NORMAL
  // initialized to 0, so off on startup
//#define SPIRIT_SUP_TIME 20  - defined in defs.h
uint16_t spiritSupTimer= 0;
uint16_t spiritSupTimer_expire_count= 0;

uint16_t ledTimer;
uint8_t ledToggle = 0;

uint8_t periodicTimer = 0;
uint8_t hundredMillisecondTimer = 0;
uint16_t secondTimer = 0;
uint32_t freeRunningTimer = 0;
extern uint8_t brownOut;
uint8_t pa4Toggle;

void SysTick_Handler(void)
{
  uint8_t i;


  if (pa4Toggle == 0)
  {
    pa4Toggle = 1;
#if TEST_MEASURE_CLOCK
    GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET);
#endif
  }
  else
  {
    pa4Toggle = 0;
#if TEST_MEASURE_CLOCK
    GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_SET);
#endif
  }
  if(brownOut == TRUE)
  {
        /* Qualify enviroment before scheduling tasks */
      return;
  }

  TB_Scheduler();
  //--------------
  // the following is a FIX to make the rest of the items in
  // systick to be executed on a 1msec basis.
  //---------------------
  if (pa4Toggle == 0)
  {
    return;
  }
  Tick();
  freeRunningTimer++;

  appTestEvent = 1;
  if (dupTimer >0)
  {
    dupTimer--;
  }
    for (i=0;i<MAX_DUP_ENTRIES;i++)
    {
      if (dupTimers[i] > 0)
      {
        dupTimers[i] = dupTimers[i] - 1;
      }
    }


  msec++;
  if (msec >10)         /*!< Executes every 10 ms */
  {
    if ((opModeTimer >1)&&(opModeTimer != 0xffff))
    {
      opModeTimer--;
      if (opModeTimer == 1)
      {
        schedByte |= SCHEDBYTE_APPMODETIMEOUT;
      }
    }
    //----------------debounceBlockingTimer
    // used to hold off operation in configuration operation.
    // not really the way i like to do this ... original code
    if (debounceBlockingTimer >0)
    {
      debounceBlockingTimer--;
    }


    msec = 0;

    if (newFirmwareTimer >0)
    {
      newFirmwareTimer--;
      if (newFirmwareTimer == 0)
      {
        schedByte |= SCHEDBYTE_DOWNLOADTIMEOUT;
      }
    }
    if (spiritWdog >0)
    {
      spiritWdog--;
    }

    if (timerBusVChange >0)
    {
      timerBusVChange--;
    }
    if (timerBusVNoChange >0)
    {
      timerBusVNoChange--;
    }

    if (timerSysMonitor >0)
    {
      timerSysMonitor--;
    }
    if (timerSysMonitor ==0)
    {
      timerSysMonitor = TIMERSYSMONITORTIME;
      schedByte |= SCHEDBYTE_SYSMONITOR;
    }

   if (timerSupMonitor >0)
    {
      timerSupMonitor--;
    }
    if (timerSupMonitor ==0)
    {
      timerSupMonitor = TIMERSUPMONITORTIME;
      schedByte |= SCHEDBYTE_SUPERVISOR;
    }

   //----------spiritInterfaceTimer
  spiritInterfaceTimer++;
//  if (spiritInterfaceTimer >= SPIRIT_INTERFACE_TIME)
//  {
//    spiritInterfaceTimer = 0;
    schedByte |= SCHEDBYTE_CM2COREBACKGROUND;
    schedByte |= SCHEDBYTE_SPIRITBACKGROUND;
    schedByte |= SCHEDBYTE_CM2PORTS;

//  }
  schedByte |= SCHEDBYTE_TIMERBACKGROUND;

    //------------------------------
    // blink LED
    ledTimer++;
    if (ledTimer >20)
    {
        ledTimer = 0;
        if (ledToggle == 0)
        {
          //both LEDS off
          GREEN_LED_ON;
          ledToggle = 1;
        }
        else
        {
          GREEN_LED_OFF;
          ledToggle = 0;
        }
    }
    if (spiritSupTimer >0)
    {
      spiritSupTimer--;
    }

  }
  //----------------------- 100msec timer base
  hundredMillisecondTimer++;
  if (hundredMillisecondTimer >= 100)
  {
    hundredMillisecondTimer = 0;
    if ((myflashConfigSection.Item.appMode!= 0)&&(AppProductionVerifyInfo()!=0))
    {

    }
    else
    {
      PropulsionRampTask();
    }
  }

  secondTimer++;
  if (secondTimer >= 1000)
  {
    secondTimer = 0;
    if (periodicTimer >0)
    {
      periodicTimer--;
    }
    if (periodicTimer ==0)
    {
     periodicTimer = 2;
//      schedByte |= SCHEDBYTE_CM2PERIODICTASK;
    }
    if (getAngleTimer >0)
    {
      getAngleTimer--;
      if (getAngleTimer == 0)
      {
        schedByte |= SCHEDBYTE_SAVETIMNGADJ;
      }
    }
    if (stopMotorTimer >0)
    {
      stopMotorTimer--;
      if (stopMotorTimer == 0)
      {
        schedByte |= SCHEDBYTE_STOPMOTOR;
      }
    }

    if (runRPMMotorTimer >0)
    {
      runRPMMotorTimer--;
      if (runRPMMotorTimer == 0)
      {
        schedByte |= SCHEDBYTE_RPMMOTORSTOP;
      }
    }

  }

}
#endif
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
