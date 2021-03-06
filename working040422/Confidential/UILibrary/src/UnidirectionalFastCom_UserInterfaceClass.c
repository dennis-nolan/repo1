/**
  ******************************************************************************
  * @file    UnidirectionalFastCom_UserInterfaceClass.c
  * @author  STMicroelectronics - System Lab - MC Team
  * @version 4.2.0
  * @date    20-Aug-2015 18:06
  * @brief   This file contains private impelementation of UnidirectionalFastCom class      
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
#include "UserInterfaceClass.h"
#include "UserInterfacePrivate.h"
#include "UnidirectionalFastCom_UserInterfaceClass.h"
#include "UnidirectionalFastCom_UserInterfacePrivate.h"
#include "MC_type.h"
#include "UIIRQHandlerPrivate.h"

#ifdef MC_CLASS_DYNAMIC
  #include "stdlib.h" /* Used for dynamic allocation */
#else
  #define MAX_UFC_UI_NUM 1u
  _DCUFC_UI_t UFC_UIpool[MAX_UFC_UI_NUM];
  unsigned char UFC_UI_Allocated = 0u;
#endif

#define DCLASS_PARAM ((_DCUFC_UI)(((_CUI) this)->DerivedClass))->pDParams_str
#define DCLASS_VARS  &(((_DCUFC_UI)(((_CUI) this)->DerivedClass))->DVars_str)
#define  CLASS_VARS  &(((_CUI)this)->Vars_str)
#define  CLASS_PARAM (((_CUI)this)->pParams_str)

/* Private function prototypes -----------------------------------------------*/  
void* UFC_IRQ_Handler(void* this,unsigned char flags, unsigned short rx_data);

/**
  * @brief  Creates an object of the class UnidirectionalFastCom
  * @param  pUserInterfaceParams pointer to an UserInterface parameters structure
  * @param  pUnidirectionalFastComParams pointer to an UnidirectionalFastCom parameters structure
  * @retval CUFC_UI new instance of UnidirectionalFastCom object
  */
CUFC_UI UFC_NewObject(pUserInterfaceParams_t pUserInterfaceParams, pUnidirectionalFastComParams_t pUnidirectionalFastComParams)
{
  _CUI _oUserInterface;
  _DCUFC_UI _oUnidirectionalFastCom;
  
  _oUserInterface = (_CUI)UI_NewObject(pUserInterfaceParams);
  
#ifdef MC_CLASS_DYNAMIC
  _oUnidirectionalFastCom = (_DCUFC_UI)calloc(1u,sizeof(_DCUFC_UI_t));
#else
  if (UFC_UI_Allocated  < MAX_UFC_UI_NUM)
  {
    _oUnidirectionalFastCom = &UFC_UIpool[UFC_UI_Allocated++];
  }
  else
  {
    _oUnidirectionalFastCom = MC_NULL;
  }
#endif
  
  _oUnidirectionalFastCom->pDParams_str = pUnidirectionalFastComParams;
  _oUserInterface->DerivedClass = (void*)_oUnidirectionalFastCom;
  
  _oUserInterface->Methods_str.pIRQ_Handler = &UFC_IRQ_Handler;
  Set_UI_IRQ_Handler(pUnidirectionalFastComParams->bUIIRQn, (_CUIIRQ)_oUserInterface);
  
  return ((CUFC_UI)_oUserInterface);
}

/** @addtogroup STM32F10x_PMSM_MC_Library
  * @{
  */
  
/** @addtogroup UserInterface_UnidirectionalFastCom
  * @{
  */

/** @defgroup UnidirectionalFastCom_class_private_methods UnidirectionalFastCom class private methods
* @{
*/

/**
  * @}
  */
  
/**
  * @}
  */

/**
  * @}
  */

/**
  * @brief  Initialization of the class UnidirectionalFastCom. It initialize all
  *         HW and private vars.
  * @param  this related object of class CUFC_UI
  * @retval none
  */
void UFC_Init(CUFC_UI this)
{
  pVars_t pVars = CLASS_VARS;
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAM;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Init Vars */
  pVars->bSelectedDrive = pDParams->bDefMotor;
  pDVars->bChannel[0] = pDParams->bDefChannel1;
  pDVars->bChannel[1] = pDParams->bDefChannel2;
  pDVars->bChByteNum[0] = pDParams->bCh1ByteNum;
  pDVars->bChByteNum[1] = pDParams->bCh2ByteNum;
  pDVars->comON = FALSE;
  pDVars->bChTransmitted = 0;
  pDVars->bByteTransmitted = 0;
  pDVars->bChNum = pDParams->bChNum;
  
  /* HW Init */
  
  /* Enable USART clock: UASRT1 -> APB2, USART2-5 -> APB1 */
  if (pDParams->wUSARTClockSource == RCC_APB2Periph_USART1)
  {
    RCC_APB2PeriphClockCmd(pDParams->wUSARTClockSource, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(pDParams->wUSARTClockSource, ENABLE);
  }  
  
  /* USART Init structure */
  /* Configure USART */
  USART_Init(pDParams->USARTx, pDParams->USART_InitStructure);
    
  /* Configures the GPIO ports for USART. */
  if (pDParams->wUSARTRemapping != 0)
  {
    /* Enable USART AFIO clock if remapped */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    /* Enable the USART Pins Software Remapping */
    GPIO_PinRemapConfig(pDParams->wUSARTRemapping , ENABLE);
  }
    
  /* Configure Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = pDParams->hTxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(pDParams->hTxPort, &GPIO_InitStructure);
  
  if (pDParams->NVIC_InitStructure->NVIC_IRQChannelCmd == ENABLE)
  {
    /* Enable the USARTy Interrupt */
    NVIC_Init(pDParams->NVIC_InitStructure);
  }
  
  /* Enable the USART */
  USART_Cmd(pDParams->USARTx, ENABLE);
}

/*******************************************************************************
* Function Name  : USART_IRQ_Handler
* Description    : Interrupt function for the serial communication
* Input          : none 
* Return         : none
*******************************************************************************/
void* UFC_IRQ_Handler(void* this,unsigned char flags, unsigned short rx_data)
{
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAM;
  void* pRetVal = MC_NULL;
  uint8_t txData = 0;
  uint8_t* pBuff;
  
  if (flags == 1) // Flag 1 = TX
  {
    if (pDVars->comON)
    {
      if (pDVars->bByteTransmitted == 0)
      {
        /* First byte to be transmitted, read value and populate the buffer */
        pDVars->wBuffer = UI_GetReg(this, pDVars->bChannel[pDVars->bChTransmitted]) >> 8;
      }
      
      pBuff = (uint8_t*)(&(pDVars->wBuffer));
      txData = pBuff[pDVars->bByteTransmitted];
      
      /* Write one byte to the transmit data register */
      USART_SendData(pDParams->USARTx, txData);
      
      pDVars->bByteTransmitted++;
      if (pDVars->bByteTransmitted == pDVars->bChByteNum[pDVars->bChTransmitted])
      {
        pDVars->bByteTransmitted = 0;
        pDVars->bChTransmitted++;
        if (pDVars->bChTransmitted == pDVars->bChNum)
        {
          pDVars->bChTransmitted = 0;
        }
      }
    }
  }
  
  return pRetVal;
}

/**
  * @brief  Starts the fast unidirectional communication.
  * @param  this related object of class CUFC_UI
  * @retval none
  */
void UFC_StartCom(CUFC_UI this)
{
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAM;
  pDVars->comON = TRUE;
  USART_SendData(pDParams->USARTx, ' ');
  /* Enable USART Transmit interrupts */
  USART_ITConfig(pDParams->USARTx, USART_IT_TXE, ENABLE);
}

/**
  * @brief  Starts the fast unidirectional communication.
  * @param  this related object of class CUFC_UI
  * @retval none
  */
void UFC_StopCom(CUFC_UI this)
{
  pDVars_t pDVars = DCLASS_VARS;
  pDParams_t pDParams = DCLASS_PARAM;
  pDVars->comON = FALSE;
  /* Disable USART Transmit interrupts */
  USART_ITConfig(pDParams->USARTx, USART_IT_TXE, DISABLE);
}

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
