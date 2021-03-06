/**
  ******************************************************************************
  * @file    R3_HD2_PWMnCurrFdbkClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains private implementation of current sensor class in 
  *          case of dual motors and in case ICS and STM32F103 High Density is 
  *          used           
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

#if (STM32F10X_HD)

/* Includes ------------------------------------------------------------------*/
#include "PWMnCurrFdbkClass.h"
#include "PWMnCurrFdbkPrivate.h"
#include "R3_HD2_PWMnCurrFdbkClass.h"
#include "R3_HD2_PWMnCurrFdbkPrivate.h"
#include "MCIRQHandlerClass.h"
#include "MCIRQHandlerPrivate.h"
#include "MCLibraryConf.h"
#include "MCLibraryISRPriorityConf.h"
#include "MC_type.h"

/* ADC1 Data Register address */
#define ADC1_DR_Address     0x4001244Cu

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))
/* Bit Banding */
#define ADC1_CR2_EXTTRIG_SWSTART_BB             0x42248158u

#define TIMxCCER_MASK              ((uint16_t)  ~0x1555u)
#define TIMxCCER_MASK_CH123        ((uint16_t)  0x555u)

#define TIMx_CC4E_BIT              ((uint16_t)  0x1000u) 

#define CONV_STARTED               ((uint32_t) (0x8))
#define CONV_FINISHED              ((uint32_t) (0xC))
#define FLAGS_CLEARED              ((uint32_t) (0x0))
#define ADC_SR_MASK                ((uint32_t) (0xC))

#define ADC_RIGHT_ALIGNMENT 3u

#define NB_CONVERSIONS 16u

#define CLASS_VARS   ((_CPWMC)this)->Vars_str
#define DCLASS_PARAMS ((_DCR3HD2_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  ((_DCR3HD2_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str

#define PWM_PERIOD pLocalVars_Str->Half_PWMPeriod

#define PHASE_A_MSK       (u32)((u32)(pDParams_str->bIaChannel) << 15)
#define PHASE_B_MSK       (u32)((u32)(pDParams_str->bIbChannel) << 15)
#define PHASE_C_MSK       (u32)((u32)(pDParams_str->bIcChannel) << 15)

#define CR2_JEXTTRIG_Reset      ((uint32_t)0xFFFF7FFFu)
#define AUX_CR2_JEXTTRIG 0x001E1901u

#define CCMR2_CH4_DISABLE 0x8FFFu
#define CCMR2_CH4_PWM1    0x6000u
#define CCMR2_CH4_PWM2    0x7000u

#ifdef MC_CLASS_DYNAMIC
#include "stdlib.h" /* Used for dynamic allocation */
#else
_DCR3HD2_PWMC_t R3HD2_PWMCpool[MAX_DRV_PWMC_NUM];
unsigned char R3HD2_PWMC_Allocated = 0u;
#endif

static void R3HD2_Init(CPWMC this);
static void R3HD2_TIMxInit(TIM_TypeDef* TIMx, CPWMC this);
static void R3HD2_CurrentReadingCalibration(CPWMC this);
static void R3HD2_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static void R3HD2_TurnOnLowSides(CPWMC this);
static void R3HD2_SwitchOnPWM(CPWMC this);
static void R3HD2_SwitchOffPWM(CPWMC this);
static uint16_t R3HD2_WriteTIMRegisters(CPWMC this);
static uint16_t R3HD2_SetADCSampPointSect1(CPWMC this);
static uint16_t R3HD2_SetADCSampPointSect2(CPWMC this);
static uint16_t R3HD2_SetADCSampPointSect3(CPWMC this);
static uint16_t R3HD2_SetADCSampPointSect4(CPWMC this);
static uint16_t R3HD2_SetADCSampPointSect5(CPWMC this);
static uint16_t R3HD2_SetADCSampPointSect6(CPWMC this);
static void *R3HD2_IRQHandler(void *this, unsigned char flag);
static uint16_t R3HD2_ExecRegularConv(CPWMC this, uint8_t bChannel);
static void R3HD2_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct);
static void R3HD2_HFCurrentsCalibrationAB(CPWMC this,Curr_Components* pStator_Currents);
static void R3HD2_HFCurrentsCalibrationC(CPWMC this,Curr_Components* pStator_Currents);
static uint16_t R3HD2_IsOverCurrentOccurred(CPWMC this);

/**
* @brief  Creates an object of the class R3_HD2
* @param  pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
* @param  pR3_DDParams pointer to an R3_DD parameters structure
* @retval CR3HD2_PWMC new instance of R3_HD2 object
*/
CR3HD2_PWMC R3HD2_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, 
                                    pR3_DDParams_t pR3_DDParams)
{
  _CPWMC _oPWMnCurrFdbk;
  _DCR3HD2_PWMC _oR3_HD2;
  
  _oPWMnCurrFdbk = (_CPWMC)PWMC_NewObject(pPWMnCurrFdbkParams);
  
#ifdef MC_CLASS_DYNAMIC
  _oR3_HD2 = (_DCR3HD2_PWMC)calloc(1u,sizeof(_DCR3HD2_PWMC_t));
#else
  if (R3HD2_PWMC_Allocated  < MAX_DRV_PWMC_NUM)
  {
    _oR3_HD2 = &R3HD2_PWMCpool[R3HD2_PWMC_Allocated++];
  }
  else
  {
    _oR3_HD2 = MC_NULL;
  }
#endif
  
  _oR3_HD2->pDParams_str = pR3_DDParams;
  _oPWMnCurrFdbk->DerivedClass = (void*)_oR3_HD2;
  
  _oPWMnCurrFdbk->Methods_str.pIRQ_Handler = &R3HD2_IRQHandler;
  
  Set_IRQ_Handler(pR3_DDParams->IRQnb, (_CMCIRQ)_oPWMnCurrFdbk);
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_Init = &R3HD2_Init;
  _oPWMnCurrFdbk->Methods_str.pPWMC_GetPhaseCurrents = &R3HD2_GetPhaseCurrents;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOffPWM = &R3HD2_SwitchOffPWM;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOnPWM = &R3HD2_SwitchOnPWM;        
  _oPWMnCurrFdbk->Methods_str.pPWMC_CurrentReadingCalibr = 
                                                 &R3HD2_CurrentReadingCalibration;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_TurnOnLowSides = &R3HD2_TurnOnLowSides;         
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect1 = 
                                                      &R3HD2_SetADCSampPointSect1;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect2 = 
                                                      &R3HD2_SetADCSampPointSect2;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect3 = 
                                                      &R3HD2_SetADCSampPointSect3;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect4 = 
                                                      &R3HD2_SetADCSampPointSect4;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect5 = 
                                                      &R3HD2_SetADCSampPointSect5;
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect6 = 
                                                      &R3HD2_SetADCSampPointSect6;
  _oPWMnCurrFdbk->Methods_str.pPWMC_ExecRegularConv= &R3HD2_ExecRegularConv;
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_SetSamplingTime= &R3HD2_ADC_SetSamplingTime;
  
  _oPWMnCurrFdbk->Methods_str.pPWMC_IsOverCurrentOccurred = 
    &R3HD2_IsOverCurrentOccurred;
  return ((CR3HD2_PWMC)_oPWMnCurrFdbk);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
* @{
*/

/** @addtogroup PWMnCurrFdbk_R3_HD2
* @{
*/

/** @defgroup R3_HD2_class_private_methods R3_HD2 class private methods
* @{
*/

/**
* @brief  It initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading 
*         in ICS configuration using STM32F103x High Density
* @param  this: related object of class CR3HD2_PWMC
* @retval none
*/
static void R3HD2_Init(CPWMC this)
{
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  uint16_t hAux;
  pVars_t pVars_str = &CLASS_VARS;
  pDVars_t pDVars_str = &DCLASS_VARS;  
  pDParams_t pDParams_str = DCLASS_PARAMS; 
  uint32_t wAux1, wAux2, wAux3, wAux4;
  
  pDVars_str->Half_PWMPeriod = ((((_CPWMC) this)->pParams_str->hPWMperiod)/2u);
  pVars_str->bMotor = (pDParams_str->bInstanceNbr==1u?M1:M2);
    
  /* Peripheral clocks enabling ---------------------------------------------*/
  
  RCC->AHBENR |= RCC_AHBPeriph_CRC;
  
  /* ADCCLK = PCLK2 */
  RCC_ADCCLKConfig(pDParams_str ->wADC_Clock_Divider);
  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* Enable ADC2 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); 
  /* Enable GPIOA-GPIOF clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | 
                         RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | 
                           RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | 
                             RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);  
   /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
  
  /* Enable the CCS */
  RCC_ClockSecuritySystemCmd((FunctionalState)(ENABLE));
  
  if(pDParams_str->TIMx == TIM1)
  {
    /* Enable TIM1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    /* Stores the value for the context switching */
    /* ADCx Injected conversions trigger is swithced to TIM1 CH4 */
    /* Stdlib replaced: ADC_ExternalTrigInjectedConvConfig(ADCx, ADC_ExternalTrigInjecConv_T1_CC4);*/ 
    pDVars_str->wADCTriggerSet =   0x1E9901u;
    pDVars_str->wADCTriggerUnSet = 0x1E1901u;
        
    /* Store the bit-banding address to activate/deactivate TIMx CH4 channel */
    pDVars_str->wTIMxCH4_BB_Addr = 0x42258430u;
  }
  else
  {
    /* Enable TIM8 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    
    /* Stores the value for the context switching */
    /* ADCx Injected conversions trigger is swithced to TIM8 CH4 */
    /* Stdlib replaced: ADC_ExternalTrigInjectedConvConfig(ADCx, ADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4);*/
    pDVars_str->wADCTriggerSet =   0x1EE901u;
    pDVars_str->wADCTriggerUnSet = 0x1E6901u;
    
    /* Store the bit-banding address to activate/deactivate TIMx CH4 channel */
    pDVars_str->wTIMxCH4_BB_Addr = 0x42268430u;
  }
  
  /* GPIOs configurations --------------------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  
  /****** Configure phase A ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hIaPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(pDParams_str->hIaPort,
            &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams_str->hIaPort, pDParams_str->hIaPin);
  
  /****** Configure phase B ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hIbPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(pDParams_str->hIbPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams_str->hIbPort, pDParams_str->hIbPin);
  
  /****** Configure phase C ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = pDParams_str->hIcPin;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(pDParams_str->hIcPort, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams_str->hIcPort, pDParams_str->hIcPin);
  
  /* Alternate function remapping available only for TIM1 */
  if(pDParams_str->TIMx == TIM1)
  {     
    /****** Timer1 alternate function full remapping ******/  
    if((pDParams_str-> wTIM1Remapping) != GPIO_NoRemap_TIM1)
    {   
      GPIO_PinRemapConfig(pDParams_str->wTIM1Remapping, ENABLE);  
    }  
  }
  
  /****** Configure TIMx Channel 1, 2 and 3 Outputs ******/ 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  hAux = (pDParams_str->hCh1Pin) | (pDParams_str->hCh2Pin);  
  hAux = hAux | (pDParams_str->hCh3Pin);  
  GPIO_InitStructure.GPIO_Pin = hAux; 
  GPIO_Init(pDParams_str->hCh1Port, &GPIO_InitStructure);
  GPIO_PinLockConfig(pDParams_str->hCh1Port, hAux);
  
  /****** Configure TIMx Channel 1N, 2N and 3N Outputs, if enabled ******/    
  if ((pDParams_str->LowSideOutputs)== LS_PWM_TIMER) 
  { 
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh1NPin;  
    GPIO_Init(pDParams_str->hCh1NPort, &GPIO_InitStructure);  
    GPIO_PinLockConfig(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin);
    
    hAux = (pDParams_str->hCh2NPin) | (pDParams_str->hCh3NPin);
    GPIO_InitStructure.GPIO_Pin = hAux; 
    GPIO_Init(pDParams_str->hCh2NPort, &GPIO_InitStructure);    
    GPIO_PinLockConfig(pDParams_str->hCh2NPort, hAux);
  }  else if ((pDParams_str->LowSideOutputs)== ES_GPIO)
  {
    /* Only "active high" polarity is supported */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hCh1NPin;  
    GPIO_Init(pDParams_str->hCh1NPort, &GPIO_InitStructure);  
    GPIO_PinLockConfig(pDParams_str->hCh1NPort, pDParams_str->hCh1NPin);
    
    hAux = (pDParams_str->hCh2NPin) | (pDParams_str->hCh3NPin);
    GPIO_InitStructure.GPIO_Pin = hAux; 
    GPIO_Init(pDParams_str->hCh2NPort, &GPIO_InitStructure);    
    GPIO_PinLockConfig(pDParams_str->hCh2NPort, hAux);
  }
  else
  {
  }
  
  if ((pDParams_str->EmergencyStop)!= DISABLE)  
  {
    /****** Configure TIMx BKIN input, if enabled ******/   
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pDParams_str->hBKINPin;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(pDParams_str->hBKINPort, &GPIO_InitStructure); 
    GPIO_PinLockConfig(pDParams_str->hBKINPort, pDParams_str->hBKINPin);
  }
  
  
  R3HD2_TIMxInit(pDParams_str->TIMx, this);
  
  if(pDParams_str->TIMx == TIM1)
  {   
    /* TIM1 Counter Clock stopped when the core is halted */
    DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
  }
  else
  {
    /* TIM8 Counter Clock stopped when the core is halted */
    DBGMCU_Config(DBGMCU_TIM8_STOP, ENABLE);
  }
  
  /* It saves the sampling time settings before DeInit in case of second instance */  
  wAux1 = ADC1->SMPR1;
  wAux2 = ADC1->SMPR2;
  wAux3 = ADC2->SMPR1;
  wAux4 = ADC2->SMPR2;
  
  /* ADC1 and ADC2 registers configuration ---------------------------------*/
  /* ADC1 and ADC2 registers reset */  
  ADC_DeInit(ADC1);
  ADC_DeInit(ADC2);

  /* ADC1 and ADC2 registers configuration ---------------------------------*/
  /* Enable ADC1 and ADC2 */
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);

  
  ADC1->SMPR1 = wAux1;
  ADC1->SMPR2 = wAux2;  
  ADC2->SMPR1 = wAux3;
  ADC2->SMPR2 = wAux4;
  
  /* ADC Init */
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_NbrOfChannel = 1u;
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);
  
  ADC_InjectedDiscModeCmd(ADC1, ENABLE);
  ADC_InjectedDiscModeCmd(ADC2, ENABLE);

  /* Enable external trigger (it will be SW) for ADC1 and ADC2 regular 
  conversions */ 
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  ADC_ExternalTrigConvCmd(ADC2, ENABLE);
  
  /* Remap of Eternal trigger of ADC1 and 2 into TIM8 CH4 */
  GPIO_PinRemapConfig(GPIO_Remap_ADC1_ETRGINJ, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_ADC2_ETRGINJ, ENABLE);
    
  /* Start calibration of ADC1 and ADC2 */
  ADC_StartCalibration(ADC1);
  ADC_StartCalibration(ADC2);
  
  /* Wait for the end of ADCs calibration */
  while (ADC_GetCalibrationStatus(ADC1) & ADC_GetCalibrationStatus(ADC2))
  {
  }
  
/* It is used only to configure the sampling time to the corresponding channel*/
  ADC_InjectedChannelConfig(ADC1, pDParams_str->bIaChannel, 1u,
                                                pDParams_str->b_IaSamplingTime);
  ADC_InjectedChannelConfig(ADC1, pDParams_str->bIbChannel, 1u,
                                                pDParams_str->b_IbSamplingTime);
  ADC_InjectedChannelConfig(ADC1, pDParams_str->bIcChannel, 1u,
                                                pDParams_str->b_IcSamplingTime);
  ADC_InjectedChannelConfig(ADC2, pDParams_str->bIaChannel, 1u,
                                                pDParams_str->b_IaSamplingTime);
  ADC_InjectedChannelConfig(ADC2, pDParams_str->bIbChannel, 1u,
                                                pDParams_str->b_IbSamplingTime);
  ADC_InjectedChannelConfig(ADC2, pDParams_str->bIcChannel, 1u,
                                                pDParams_str->b_IcSamplingTime);    
  
  /* DMA Event related to ADC regular conversion*/
  /* DMA1 channel1 configuration */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&(pDVars_str->hRegConv));
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1u;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  /* Enable DMA1 Channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  /* Enable ADC1 EOC DMA */
  ADC_DMACmd(ADC1,ENABLE);  

  /* ADC1 Injected conversions end interrupt enabling */
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC); 	 
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE); 
  
  /* Enable the ADC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 
    ADC_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
    
  if(pDParams_str->TIMx==TIM1)
  {
    /* Enable the TIM1 Update interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_UP_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_UP_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);   
  }
  else
  {
    /* Enable the TIM1 Update interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t) TIM8_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_UP_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_UP_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);     
  }
}

/**
* @brief  It initializes TIMx peripheral for PWM generation
* @param 'TIMx': Timer to be initialized
* @param 'this': related object of class CR3HD2_PWMC
* @retval none
*/
static void R3HD2_TIMxInit(TIM_TypeDef* TIMx, CPWMC this)
{
  TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
  TIM_OCInitTypeDef TIMx_OCInitStructure;
  TIM_BDTRInitTypeDef TIMx_BDTRInitStructure;
  pDVars_t pDVars_str = &DCLASS_VARS;  
  pDParams_t pDParams_str =DCLASS_PARAMS; 
  
  /* TIMx Peripheral Configuration -------------------------------------------*/
  /* TIMx Registers reset */
  TIM_DeInit(TIMx);
  TIM_TimeBaseStructInit(&TIMx_TimeBaseStructure);
  /* Time Base configuration */
  TIMx_TimeBaseStructure.TIM_Prescaler = (uint16_t)(pDParams_str->bTim_Clock_Divider) - 1u;
  TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
  TIMx_TimeBaseStructure.TIM_Period = pDVars_str->Half_PWMPeriod;
  TIMx_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
  TIMx_TimeBaseStructure.TIM_RepetitionCounter = pDParams_str->
                                                            bRepetitionCounter;
  TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);
  
  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCStructInit(&TIMx_OCInitStructure);  
  TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIMx_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIMx_OCInitStructure.TIM_Pulse = pDVars_str->Half_PWMPeriod/2u; /* dummy value */
  
  /* Channel 1 */
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams_str->hCh1Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams_str->hCh1IdleState;    
  if ((pDParams_str-> LowSideOutputs)== LS_PWM_TIMER)
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; 
    TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh1NPolarity; 
    TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh1NIdleState;     
  }    
  else
  {
    TIMx_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  }    
  TIM_OC1Init(TIMx, &TIMx_OCInitStructure); 
  
  
  /* Channel 2 */
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams_str->hCh2Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams_str->hCh2IdleState;    
  if ((pDParams_str-> LowSideOutputs)== LS_PWM_TIMER)
  {
  TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh2NPolarity; 
  TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh2NIdleState;         
  }
  TIM_OC2Init(TIMx, &TIMx_OCInitStructure); 
  
  
  /* Channel 3 */
  TIMx_OCInitStructure.TIM_OCPolarity = pDParams_str->hCh3Polarity;      
  TIMx_OCInitStructure.TIM_OCIdleState = pDParams_str->hCh3IdleState;    
  if ((pDParams_str-> LowSideOutputs)== LS_PWM_TIMER)
  {
  TIMx_OCInitStructure.TIM_OCNPolarity = pDParams_str->hCh3NPolarity; 
  TIMx_OCInitStructure.TIM_OCNIdleState = pDParams_str->hCh3NIdleState;         
  }
  TIM_OC3Init(TIMx, &TIMx_OCInitStructure);   
  
    /* Channel 4 */
  TIMx_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIMx_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
  TIMx_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; 
  TIMx_OCInitStructure.TIM_Pulse = DCLASS_VARS.Half_PWMPeriod-5u;
  TIM_OC4Init(TIMx, &TIMx_OCInitStructure); 
  
  /* Enables the TIMx Preload on CC1 Register */
  TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC2 Register */
  TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC3 Register */
  TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
  /* Enables the TIMx Preload on CC4 Register */
  TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable); 
  
  TIM_BDTRStructInit(&TIMx_BDTRInitStructure);
  /* Dead Time */
  TIMx_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIMx_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIMx_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
  TIMx_BDTRInitStructure.TIM_DeadTime = (pDParams_str->hDeadTime)/2u;
  /* BKIN, if enabled */
  if ((pDParams_str->EmergencyStop)!= DISABLE)  
  {
    TIMx_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
    TIMx_BDTRInitStructure.TIM_BreakPolarity = pDParams_str->hBKINPolarity;
    TIMx_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_ClearITPendingBit(TIMx, TIM_IT_Break);
    TIM_ITConfig(TIMx, TIM_IT_Break, ENABLE);
  }  
  TIM_BDTRConfig(TIMx, &TIMx_BDTRInitStructure);
 
  TIM_SelectInputTrigger(TIMx,TIM_TS_ITR1);
  TIM_SelectSlaveMode(TIMx,TIM_SlaveMode_Trigger);

  /* Prepare timer for synchronization */
  TIM_GenerateEvent(TIMx,TIM_EventSource_Update);
      
  if (pDParams_str->bFreqRatio == 2u) 
  {
    if (pDParams_str->bIsHigherFreqTim == HIGHER_FREQ)
    {
      if (pDParams_str->bRepetitionCounter == 3u)
      {
        /* Set TIMx repetition counter to 1 */
        TIMx->RCR =0x01u; 
        TIM_GenerateEvent(TIMx, TIM_EventSource_Update);
        /* Repetition counter will be set to 3 at next Update */
        TIMx->RCR =0x03u; 
      }
    }
    
    TIM_SetCounter(TIMx, pDVars_str->Half_PWMPeriod-1u);     
  }
  else /* bFreqRatio equal to 1 or 3 */
  {
    if (pDParams_str->bInstanceNbr == 1u)
    {
      TIM_SetCounter(TIMx, pDVars_str->Half_PWMPeriod-1u);
    }
  }
}

/**
* @brief  It perform the start of all the timers required by the control. 
          It utilizes TIM2 as temporary timer to achieve synchronization between 
          PWM signals.
          When this function is called, TIM1 and/or TIM8 must be in frozen state
          with CNT, ARR, REP RATE and trigger correctly set (these setting are 
          usually performed in the Init method accordingly with the configuration)
* @param  none
* @retval none
*/
 void R3HD2_StartTimers(void)
{
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;   
  
  /* Temporary Enable TIM2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_DeInit(TIM2);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0u;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 1u; /* dummy */ 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);  
     
  TIM_Cmd(TIM2, ENABLE);
  
  TIM_DeInit(TIM2);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE); 
}


/**
* @brief  It stores into 'this' object variables the voltage present on Ia and 
*         Ib current feedback analog channels when no current is flowin into the
*         motor
* @param  this: related object of class CR3HD2_PWMC
* @retval none
*/
static void R3HD2_CurrentReadingCalibration(CPWMC this)
{
  pDVars_t pDVars_str = &DCLASS_VARS;
  pDParams_t pDParams_str =  DCLASS_PARAMS;

  TIM_TypeDef*  LocalTIMx = pDParams_str->TIMx;    
  
  pDVars_str-> wPhaseAOffset = 0u;
  pDVars_str-> wPhaseBOffset = 0u; 
  pDVars_str-> wPhaseCOffset = 0u; 
  
  pDVars_str->bIndex=0u;
  
  /* It forces inactive level on TIMx CHy and CHyN */
  LocalTIMx->CCER &= TIMxCCER_MASK;
   
  /* Offset calibration for A & B phases */
  /* Change function to be executed in ADCx_ISR */ 
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents = &R3HD2_HFCurrentsCalibrationAB;
  
  pDVars_str->wADC1Channel = PHASE_A_MSK;
  pDVars_str->wADC2Channel = PHASE_B_MSK;
  
  R3HD2_SwitchOnPWM(this);
  
  /* Wait for NB_CONVERSIONS to be executed */
  while (pDVars_str->bIndex < (NB_CONVERSIONS))
  {
    if (LocalTIMx->DIER & TIM_IT_Update)
    {}
    else
    {
      pDVars_str->bIndex = NB_CONVERSIONS;
    }
  }

  /* Offset calibration for C phase */
  /* Reset bIndex */
  pDVars_str->bIndex=0u;

  /* Change function to be executed in ADCx_ISR */ 
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents = &R3HD2_HFCurrentsCalibrationC;

  pDVars_str->wADC1Channel = PHASE_C_MSK;
  pDVars_str->wADC2Channel = PHASE_C_MSK;
  
  R3HD2_SwitchOnPWM(this);
  
  /* Wait for NB_CONVERSIONS to be executed */
  while (pDVars_str->bIndex < (NB_CONVERSIONS/2u))
  {
    if (LocalTIMx->DIER & TIM_IT_Update)
    {}
    else
    {
      pDVars_str->bIndex = NB_CONVERSIONS;
    }
  }
  
  pDVars_str->wPhaseAOffset >>=3; 
  pDVars_str->wPhaseBOffset >>=3; 
  pDVars_str->wPhaseCOffset >>=3; 

  /* Change back function to be executed in ADCx_ISR */ 
  ((_CPWMC) this)->Methods_str.pPWMC_GetPhaseCurrents = &R3HD2_GetPhaseCurrents;

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to 
     force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */  
  LocalTIMx->CCMR1 &= 0xF7F7u;
  LocalTIMx->CCMR2 &= 0xF7F7u;
  LocalTIMx->CCR1 = pDVars_str->Half_PWMPeriod;
  LocalTIMx->CCR2 = pDVars_str->Half_PWMPeriod;
  LocalTIMx->CCR3 = pDVars_str->Half_PWMPeriod;
  
  /* Enable TIMx preload */
  LocalTIMx->CCMR1 |= 0x0808u;
  LocalTIMx->CCMR2 |= 0x0808u;
  
  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LocalTIMx->CCER |= 0x555u;
}

/**
* @brief  It computes and return latest converted motor phase currents motor
* @param  this: related object of class CR3HD2_PWMC
* @retval Ia and Ib current in Curr_Components format
*/
static void R3HD2_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{
  uint8_t bSector;
  int32_t wAux;
  pDVars_t pDVars_str = &(((_DCR3HD2_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str);
  
  /* Deactivate TIMx CH4 to disable next triggers using bit-banding access */
  *(uint32_t*)(pDVars_str->wTIMxCH4_BB_Addr) = 0u;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pDVars_str->bSoFOC = 0u;
  
  bSector = (uint8_t)(((_CPWMC)this)->Vars_str.hSector);

  switch (bSector)
  {
  case SECTOR_4:
  case SECTOR_5: 
    /* Current on Phase C is not accessible     */
    /* Ia = PhaseAOffset - ADC converted value) */
    wAux = (int32_t)(ADC1->JDR1);
    wAux *= 2;
    wAux = (int32_t)(pDVars_str->wPhaseAOffset) - wAux;
    
    /* Saturation of Ia */
    if (wAux < S16_MIN)
    {
      pStator_Currents->qI_Component1= S16_MIN;
    }  
    else  if (wAux > S16_MAX)
    { 
      pStator_Currents->qI_Component1= S16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component1= (int16_t)wAux;
    }
    
    /* Ib = PhaseBOffset - ADC converted value) */
    wAux = (int32_t)(ADC2->JDR1);
    wAux *= 2;
    wAux = (int32_t)(pDVars_str->wPhaseBOffset) - wAux;
    
    /* Saturation of Ib */
    if (wAux < S16_MIN)
    {
      pStator_Currents->qI_Component2= S16_MIN;
    }  
    else  if (wAux > S16_MAX)
    { 
      pStator_Currents->qI_Component2= S16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component2= (int16_t)wAux;
    }
    break;
    
  case SECTOR_6:
  case SECTOR_1:  
    /* Current on Phase A is not accessible     */
    /* Ib = PhaseBOffset - ADC converted value) */
    wAux = (int32_t)(ADC1->JDR1);
    wAux *= 2;
    wAux = (int32_t)(pDVars_str->wPhaseBOffset) - wAux;
    
    /* Saturation of Ib */
    if (wAux < S16_MIN)
    {
      pStator_Currents->qI_Component2= S16_MIN;
    }  
    else  if (wAux > S16_MAX)
    { 
      pStator_Currents->qI_Component2= S16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component2= (int16_t)wAux;
    }
    
    /* Ia = -Ic -Ib */
    wAux = (int32_t)(ADC2->JDR1);
    wAux *= 2;
    wAux -= (int32_t)pDVars_str->wPhaseCOffset;
    wAux -= (int32_t)pStator_Currents->qI_Component2;

    /* Saturation of Ia */
    if (wAux> S16_MAX)
    {
      pStator_Currents->qI_Component1 = S16_MAX;
    }
    else  if (wAux <S16_MIN)
    {
      pStator_Currents->qI_Component1 = S16_MIN;
    }
    else
    {  
      pStator_Currents->qI_Component1 = (int16_t)wAux;
    }
    break;
    
  case SECTOR_2:
  case SECTOR_3:
    /* Current on Phase B is not accessible     */
    /* Ia = PhaseAOffset - ADC converted value) */
    wAux = (int32_t)(ADC1->JDR1);
    wAux *= 2;
    wAux = (int32_t)(pDVars_str->wPhaseAOffset) - wAux;
    
    /* Saturation of Ia */
    if (wAux < S16_MIN)
    {
      pStator_Currents->qI_Component1= S16_MIN;
    }  
    else  if (wAux > S16_MAX)
    { 
      pStator_Currents->qI_Component1= S16_MAX;
    }
    else
    {
      pStator_Currents->qI_Component1= (int16_t)wAux;
    }
    
    /* Ib = -Ic -Ia */
    wAux = (int32_t)(ADC2->JDR1);
    wAux *= 2;
    wAux -= (int32_t)pDVars_str->wPhaseCOffset;
    wAux -= (int32_t)pStator_Currents->qI_Component1;

    /* Saturation of Ib */
    if (wAux> S16_MAX)
    {
      pStator_Currents->qI_Component2=S16_MAX;
    }
    else  if (wAux <S16_MIN)
    {  
      pStator_Currents->qI_Component2 = S16_MIN;
    }
    else  
    {
      pStator_Currents->qI_Component2 = (int16_t)wAux;
    }                     
    break;
    
  default:
    break;
  }   
}

/**
* @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during 
*         calibration. It sum up injected conversion data into wPhaseAOffset and
*         wPhaseBOffset to compute the offset introduced in the current feedback
*         network. It is requied to proper configure ADC inputs before to enable
*         the offset computation.
* @param  this: related object of class CPWMC
* @retval It always returns {0,0} in Curr_Components format
*/
static void R3HD2_HFCurrentsCalibrationAB(CPWMC this,Curr_Components* pStator_Currents)
{  
  /* Derived class members container */
  pDVars_t pDVars_str = &DCLASS_VARS; 
  
  /* Deactivate TIMx CH4 to disable next triggers using bit-banding access */
  *(uint32_t*)(pDVars_str->wTIMxCH4_BB_Addr) = 0u;
  
  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pDVars_str->bSoFOC = 0u;
  
  if (pDVars_str->bIndex < NB_CONVERSIONS)
  {
    pDVars_str-> wPhaseAOffset += ADC1->JDR1;
    pDVars_str-> wPhaseBOffset += ADC2->JDR1; 
    pDVars_str->bIndex++;
  }
}

/**
* @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during 
*         calibration. It sum up injected conversion data into wPhaseCOffset
*         to compute the offset introduced in the current feedback
*         network. It is requied to proper configure ADC input before to enable
*         the offset computation.
* @param  this: related object of class CPWMC
* @retval It always returns {0,0} in Curr_Components format
*/
static void R3HD2_HFCurrentsCalibrationC(CPWMC this,Curr_Components* pStator_Currents)
{
  /* Derived class members container */
  pDVars_t pDVars_str = &DCLASS_VARS;  

  /* Deactivate TIMx CH4 to disable next triggers using bit-banding access */
  *(uint32_t*)(pDVars_str->wTIMxCH4_BB_Addr) = 0u;
  
  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pDVars_str->bSoFOC = 0u;
  
  if (pDVars_str->bIndex < NB_CONVERSIONS/2u)
  {
    pDVars_str-> wPhaseCOffset += ADC1->JDR1;
    pDVars_str-> wPhaseCOffset += ADC2->JDR1; 
    pDVars_str->bIndex++;
  }
}

/**
  * @brief  It turns on low sides switches. This function is intended to be 
  *         used for charging boot capacitors of driving section. It has to be 
  *         called each motor start-up when using high voltage drivers
  * @param  this: related object of class CR3HD2_PWMC
  * @retval none
  */
static void R3HD2_TurnOnLowSides(CPWMC this)
{
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;  
  
  /* Clear Update Flag */
  TIM_ClearFlag(LocalTIMx, TIM_FLAG_Update);
  
  /*Turn on the three low side switches */
  LocalTIMx->CCR1 = 0u;
  LocalTIMx->CCR2 = 0u;
  LocalTIMx->CCR3 = 0u;
  
  /* Wait until next update */
  while (TIM_GetFlagStatus(LocalTIMx,TIM_FLAG_Update)==RESET)
  {}
  
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(LocalTIMx, ENABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_SET);
  }
  return; 
}


/**
* @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
*         bit
* @param  this: related object of class CR3HD2_PWMC
* @retval none
*/
static void R3HD2_SwitchOnPWM(CPWMC this)
{  
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;  
 
  /* It clears ADCs JSTRT and JEOC bits */
  ADC1->SR &= ~ADC_SR_MASK;
  ADC2->SR &= ~ADC_SR_MASK;
  
  /* Clear Update Flag */
  TIM_ClearFlag(LocalTIMx, TIM_FLAG_Update);
    
  TIM_ITConfig(LocalTIMx, TIM_IT_Update, ENABLE);
  
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(LocalTIMx, ENABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_SET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_SET);
  }
  return; 
}


/**
* @brief  It disables PWM generation on the proper Timer peripheral acting on 
*         MOE bit
* @param  this: related object of class CR3HD2_PWMC
* @retval none
*/
static void R3HD2_SwitchOffPWM(CPWMC this)
{ 
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;
 
    /* Disable UPDATE ISR */
  TIM_ITConfig(LocalTIMx, TIM_IT_Update, DISABLE);  
  
  LocalTIMx->CCER &= (uint16_t)(~TIMxCCER_MASK_CH123);
  
  while (TIM_GetFlagStatus(LocalTIMx,TIM_FLAG_Update)==RESET)
  {
    if (LocalTIMx->DIER & TIM_IT_Update)
    { break;}
  }
   
    /* Main PWM Output Disable */
  TIM_CtrlPWMOutputs(LocalTIMx, DISABLE);
  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    GPIO_WriteBit(pLocalDParams->hCh1NPort, pLocalDParams->hCh1NPin, Bit_RESET);
    GPIO_WriteBit(pLocalDParams->hCh2NPort, pLocalDParams->hCh2NPin, Bit_RESET);
    GPIO_WriteBit(pLocalDParams->hCh3NPort, pLocalDParams->hCh3NPin, Bit_RESET);
  }
  LocalTIMx->CCER |= TIMxCCER_MASK_CH123; 
  
  return; 
}

/**
* @brief  It stores into 'this' object variables the voltage present on Ia and 
*         Ib current feedback analog channels when no current is flowin into the
*         motor
* @param  this: related object of class CR3HD2_PWMC
* @retval none
*/
static uint16_t R3HD2_WriteTIMRegisters(CPWMC this)
{
  uint16_t hAux;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;
  /* Derived class members container */
  pDVars_t pDVars_str = &DCLASS_VARS;  
    
  LocalTIMx->CCR1 = ((_CPWMC) this)->Vars_str.hCntPhA;
  LocalTIMx->CCR2 = ((_CPWMC) this)->Vars_str.hCntPhB;
  LocalTIMx->CCR3 = ((_CPWMC) this)->Vars_str.hCntPhC;
    
  /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred 
  and thus the FOC rate is too high */
  if (pDVars_str->bSoFOC != 0u)
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  return hAux;
}

/**
* @brief  Configure the ADC for the current sampling related to sector 1.
*         It means set the sampling point via TIMx_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3HD2_SetADCSampPointSect1(CPWMC this)
{
  uint16_t hCntSmp, hDeltaDuty;
  Vars_t *pBaseVars = &((_CPWMC) this)->Vars_str;
  pDParams_t pDParams_str =  ((_DCR3HD2_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  TIM_TypeDef*  LocalTIMx = pDParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  LocalTIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  if ((u16)(PWM_PERIOD-pBaseVars->hCntPhA) > pDParams_str->hTafter)
  {
    hCntSmp = PWM_PERIOD - 1u;
  }
  else
  {
    hDeltaDuty = (u16)(pBaseVars->hCntPhA - pBaseVars->hCntPhB);
    
    /* Definition of crossing point */
    if (hDeltaDuty > (u16)(PWM_PERIOD-pBaseVars->hCntPhA)*2u)
    {
      hCntSmp = pBaseVars->hCntPhA - pDParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pBaseVars->hCntPhA + pDParams_str->hTafter;
      
      if (hCntSmp >= PWM_PERIOD)
      { 
        /* Set CC4 as PWM mode 1 */
        LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        LocalTIMx->CCMR2 |= CCMR2_CH4_PWM1;
        
        hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
      }
    }
  }
  
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC1, PHASE_B_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC1Channel = PHASE_B_MSK;
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC2, PHASE_C_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC2Channel = PHASE_C_MSK;
  
  /* Set TIMx_CH4 value */
  LocalTIMx->CCR4 = hCntSmp; 

  return R3HD2_WriteTIMRegisters(this);
}

/**
* @brief  Configure the ADC for the current sampling related to sector 2.
*         It means set the sampling point via TIMx_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3HD2_SetADCSampPointSect2(CPWMC this)
{
  uint16_t hCntSmp, hDeltaDuty;
  Vars_t *pBaseVars = &((_CPWMC) this)->Vars_str;
  pDParams_t pDParams_str =  ((_DCR3HD2_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  TIM_TypeDef*  LocalTIMx = pDParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  LocalTIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  if ((u16)(PWM_PERIOD-pBaseVars->hCntPhB) > pDParams_str->hTafter)
  {
    hCntSmp = PWM_PERIOD - 1u;
  }
  else
  {
    hDeltaDuty = (u16)(pBaseVars->hCntPhB - pBaseVars->hCntPhA);
    
    /* Definition of crossing point */
    if (hDeltaDuty > (u16)(PWM_PERIOD-pBaseVars->hCntPhB)*2u)
    {
      hCntSmp = pBaseVars->hCntPhB - pDParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pBaseVars->hCntPhB + pDParams_str->hTafter;
      
      if (hCntSmp >= PWM_PERIOD)
      {
        /* Set CC4 as PWM mode 1 */
        LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        LocalTIMx->CCMR2 |= CCMR2_CH4_PWM1;
        
        hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
      }
    }
  }
  
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC1, PHASE_A_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC1Channel = PHASE_A_MSK;                
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC2,PHASE_C_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC2Channel = PHASE_C_MSK;
        
  /* Set TIMx_CH4 value */
  LocalTIMx->CCR4 = hCntSmp; 
  
  return R3HD2_WriteTIMRegisters(this);
}

/**
* @brief  Configure the ADC for the current sampling related to sector 3.
*         It means set the sampling point via TIMx_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3HD2_SetADCSampPointSect3(CPWMC this)
{
  uint16_t hCntSmp, hDeltaDuty;
  Vars_t *pBaseVars = &((_CPWMC) this)->Vars_str;
  pDParams_t pDParams_str =  ((_DCR3HD2_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  TIM_TypeDef*  LocalTIMx = pDParams_str->TIMx;

  /* Set CC4 as PWM mode 2 (default) */
  LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  LocalTIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  if ((u16)(PWM_PERIOD-pBaseVars->hCntPhB) > pDParams_str->hTafter)
  {
    hCntSmp = PWM_PERIOD - 1u;
  }
  else
  {
    hDeltaDuty = (u16)(pBaseVars->hCntPhB - pBaseVars->hCntPhC);
    
    /* Definition of crossing point */
    if (hDeltaDuty > (u16)(PWM_PERIOD-pBaseVars->hCntPhB)*2u) 
    {
      hCntSmp = pBaseVars->hCntPhB - pDParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pBaseVars->hCntPhB + pDParams_str->hTafter;
      
      if (hCntSmp >= PWM_PERIOD)
      {
        /* Set CC4 as PWM mode 1 */
        LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        LocalTIMx->CCMR2 |= CCMR2_CH4_PWM1;
        
        hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
      }
    }
  }
  
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC1, PHASE_A_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC1Channel = PHASE_A_MSK;
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC2, PHASE_C_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC2Channel = PHASE_C_MSK;
  
  /* Set TIMx_CH4 value */
  LocalTIMx->CCR4 = hCntSmp; 
  
  return R3HD2_WriteTIMRegisters(this);
}

/**
* @brief  Configure the ADC for the current sampling related to sector 4.
*         It means set the sampling point via TIMx_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3HD2_SetADCSampPointSect4(CPWMC this)
{
  uint16_t hCntSmp, hDeltaDuty;
  Vars_t *pBaseVars = &((_CPWMC) this)->Vars_str;
  pDParams_t pDParams_str =  ((_DCR3HD2_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  TIM_TypeDef*  LocalTIMx = pDParams_str->TIMx;
  
  /* Set CC4 as PWM mode 2 (default) */
  LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  LocalTIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  if ((u16)(PWM_PERIOD-pBaseVars->hCntPhC) > pDParams_str->hTafter)
  {
    hCntSmp = PWM_PERIOD - 1u;
  }
  else
  {
    hDeltaDuty = (u16)(pBaseVars->hCntPhC - pBaseVars->hCntPhB);
    
    /* Definition of crossing point */
    if (hDeltaDuty > (u16)(PWM_PERIOD-pBaseVars->hCntPhC)*2u)
    {
      hCntSmp = pBaseVars->hCntPhC - pDParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pBaseVars->hCntPhC + pDParams_str->hTafter;
      
      if (hCntSmp >= PWM_PERIOD)
      {
        /* Set CC4 as PWM mode 1 */
        LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        LocalTIMx->CCMR2 |= CCMR2_CH4_PWM1;
        
        hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
      }
    }
  }
  
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC1, PHASE_A_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC1Channel = PHASE_A_MSK;
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC2, PHASE_B_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC2Channel = PHASE_B_MSK;
  
  /* Set TIMx_CH4 value */
  LocalTIMx->CCR4 = hCntSmp; 
  
  return R3HD2_WriteTIMRegisters(this);
}

/**
* @brief  Configure the ADC for the current sampling related to sector 5.
*         It means set the sampling point via TIMx_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3HD2_SetADCSampPointSect5(CPWMC this)
{
  uint16_t hCntSmp, hDeltaDuty;
  Vars_t *pBaseVars = &((_CPWMC) this)->Vars_str;
  pDParams_t pDParams_str =  ((_DCR3HD2_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  TIM_TypeDef*  LocalTIMx = pDParams_str->TIMx;
  
  /* Set CC4 as PWM mode 2 (default) */
  LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  LocalTIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  if ((u16)(PWM_PERIOD-pBaseVars->hCntPhC) > pDParams_str->hTafter)
  {
    hCntSmp = PWM_PERIOD - 1u;
  }
  else
  {
    hDeltaDuty = (u16)(pBaseVars->hCntPhC - pBaseVars->hCntPhA);
    
    /* Definition of crossing point */
    if (hDeltaDuty > (u16)(PWM_PERIOD-pBaseVars->hCntPhC)*2u) 
    {
      hCntSmp = pBaseVars->hCntPhC - pDParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pBaseVars->hCntPhC + pDParams_str->hTafter;
      
      if (hCntSmp >= PWM_PERIOD)
      {
        /* Set CC4 as PWM mode 1 */
        LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        LocalTIMx->CCMR2 |= CCMR2_CH4_PWM1;
        
        hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
      }
    }
  }
  
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC1, PHASE_A_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC1Channel = PHASE_A_MSK;
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC2, PHASE_B_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC2Channel = PHASE_B_MSK;
  
  /* Set TIMx_CH4 value */
  LocalTIMx->CCR4 = hCntSmp; 
  
  return R3HD2_WriteTIMRegisters(this);
}

/**
* @brief  Configure the ADC for the current sampling related to sector 6.
*         It means set the sampling point via TIMx_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  this related object of class CPWMC
* @retval none
*/
static uint16_t R3HD2_SetADCSampPointSect6(CPWMC this)
{
  uint16_t hCntSmp, hDeltaDuty;
  Vars_t *pBaseVars = &((_CPWMC) this)->Vars_str;
  pDParams_t pDParams_str =  ((_DCR3HD2_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str;
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  TIM_TypeDef*  LocalTIMx = pDParams_str->TIMx;
  
  /* Set CC4 as PWM mode 2 (default) */
  LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
  LocalTIMx->CCMR2 |= CCMR2_CH4_PWM2;
  
  if ((u16)(PWM_PERIOD-pBaseVars->hCntPhA) > pDParams_str->hTafter)
  {
    hCntSmp = PWM_PERIOD - 1u;
  }
  else
  {
    hDeltaDuty = (u16)(pBaseVars->hCntPhA - pBaseVars->hCntPhC);
    
    /* Definition of crossing point */
    if (hDeltaDuty > (u16)(PWM_PERIOD-pBaseVars->hCntPhA)*2u) 
    {
      hCntSmp = pBaseVars->hCntPhA - pDParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pBaseVars->hCntPhA + pDParams_str->hTafter;
      
      if (hCntSmp >= PWM_PERIOD)
      {   
        /* Set CC4 as PWM mode 1 */
        LocalTIMx->CCMR2 &= CCMR2_CH4_DISABLE;
        LocalTIMx->CCMR2 |= CCMR2_CH4_PWM1;
        
        hCntSmp = (2u * PWM_PERIOD) - hCntSmp - 1u;
      }
    }
  }
  
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC1, PHASE_B_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC1Channel = PHASE_B_MSK;
  /* Stdlib replaced: ADC_InjectedChannelConfig(ADC2, PHASE_C_CHANNEL,1,SAMPLING_TIME_CK); */
  pLocalVars_Str->wADC2Channel = PHASE_C_MSK;
  
  /* Set TIMx_CH4 value */
  LocalTIMx->CCR4 = hCntSmp; 
  
  return R3HD2_WriteTIMRegisters(this);
}

/**
* @brief  It contains the TIMx Update event interrupt
* @param  this: related object of class CR3HD2_PWMC
* @retval none
*/
static void *R3HD2_IRQHandler(void *this, unsigned char flag)
{
  uint32_t wADCInjFlags;
  pVars_t pVars_str = &CLASS_VARS;
  pDParams_t pDParams_str = DCLASS_PARAMS;
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx; 
  pDVars_t pDVars_str = &DCLASS_VARS;

  /* Set the SOFOC flag to indicate the execution of Update IRQ*/
  pDVars_str->bSoFOC = 1u;
  
  wADCInjFlags = (ADC1-> SR) & ADC_SR_MASK;
  
  if (wADCInjFlags == CONV_STARTED)
  {
    do
    {
      wADCInjFlags = (ADC1-> SR) & ADC_SR_MASK;
    }
    while (wADCInjFlags != CONV_FINISHED);
  }
  else if (wADCInjFlags == FLAGS_CLEARED)
  {
    while ((LocalTIMx->CNT) < (pDParams_str->Tw))
    {}
    wADCInjFlags = (ADC1-> SR) & ADC_SR_MASK;
    
    if (wADCInjFlags == CONV_STARTED)
    {
      do
      {
        wADCInjFlags = (ADC1-> SR) & ADC_SR_MASK;
      }
      while (wADCInjFlags != CONV_FINISHED);
    }
  }
  else {}
  
  /* Switch Context */
  /* Disabling trigger to avoid unwanted conversion */
  ADC1->CR2 = pDVars_str->wADCTriggerUnSet;
  ADC2->CR2 = pDVars_str->wADCTriggerUnSet;
  
  /* Enabling next Trigger */
  LocalTIMx->CCER |= 0x1000u;
  
  /* It re-initilize AD converter in run time when using dual MC */   
  ADC1->CR2 = pDVars_str->wADCTriggerSet;
  ADC2->CR2 = pDVars_str->wADCTriggerSet;
  
  /* Change channels keeping equal to 1 element the sequencer lenght */ 
  ADC1->JSQR = pDVars_str->wADC1Channel;
  ADC2->JSQR = pDVars_str->wADC2Channel;

  return &(pVars_str->bMotor);
}

/**
* @brief  Execute a regular conversion using ADC1. 
*         The function is not re-entrant (can't executed twice at the same time)
* @param  this related object of class CR3HD2_PWMC
* @retval It returns converted value or oxFFFF for conversion error
*/
static uint16_t R3HD2_ExecRegularConv(CPWMC this, uint8_t bChannel)
{
  pDVars_t pDVars_str = &DCLASS_VARS;
  uint32_t tmpflag = 0u;

  ADC1->SQR3 = bChannel;
  
  DMA1_Channel1->CMAR=(uint32_t)(&(pDVars_str->hRegConv));
  
  /* Reset DMA1_CH1 TC Flag */
  DMA1->IFCR = DMA1_FLAG_TC1;
  
  /* It starts software triggered regular conversion
  through bit banding access. It is equivalent to 
  ADC1->CR2 |= EXTTRIG_SWSTART_Set; */
  *(uint32_t *)(ADC1_CR2_EXTTRIG_SWSTART_BB)=(uint32_t)(0x1u);
  
  /* Wait until end of regular conversion */
  while (tmpflag == 0u)
  {
    tmpflag = (DMA1->ISR & DMA1_FLAG_TC1);
  }
 
  return (pDVars_str->hRegConv);
}

/**
* @brief  It sets the specified sampling time for the specified ADC channel
*         on ADC1. It must be called once for each channel utilized by user
* @param  ADC channel, sampling time
* @retval none
*/
static void R3HD2_ADC_SetSamplingTime(CPWMC this, ADConv_t ADConv_struct)
{ 
   uint32_t tmpreg1 = 0u, tmpreg2 = 0u, tmpreg3=0u, tmpreg4 = SMPR1_SMP_Set;
   
  /* if ADC_Channel_10 ... ADC_Channel_17 is selected */
  if (ADConv_struct.Channel> ADC_Channel_9)
  {
    /* Get the old register value */
    tmpreg1 = ADC1->SMPR1;
    /* Calculate the mask to clear */
    tmpreg3 = (uint32_t) (ADConv_struct.Channel) - 10u;
    tmpreg3 = tmpreg3 *3u;
    tmpreg2 =  tmpreg4 << (tmpreg3);
    /* Clear the old discontinuous mode channel count */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)(ADConv_struct.SamplTime) << (tmpreg3);
    /* Set the discontinuous mode channel count */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADC1->SMPR1 = tmpreg1;
  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    /* Get the old register value */
    tmpreg1 = ADC1->SMPR2;
    tmpreg3 = (uint32_t) (ADConv_struct.Channel) * 3u;
    /* Calculate the mask to clear */
    tmpreg4 = SMPR2_SMP_Set;
    tmpreg2 =  tmpreg4 << (tmpreg3);
    /* Clear the old discontinuous mode channel count */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (uint32_t)(ADConv_struct.SamplTime) << (tmpreg3);
    /* Set the discontinuous mode channel count */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADC1->SMPR2 = tmpreg1;
  }
}
/**
* @brief  It is used to check if an overcurrent occurred since last call.
* @param  this related object of class CPWMC
* @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been 
*                  detected since last method call, MC_NO_FAULTS otherwise.
*/
static uint16_t R3HD2_IsOverCurrentOccurred(CPWMC this)
{
  TIM_TypeDef*  LocalTIMx = DCLASS_PARAMS->TIMx;
  uint16_t retVal = MC_NO_FAULTS;
  if ((LocalTIMx->SR & TIM_FLAG_Break) != 0u)
  {
    retVal = MC_BREAK_IN;
    LocalTIMx->SR = (u16)~TIM_FLAG_Break;
  }
  return retVal;
}

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
