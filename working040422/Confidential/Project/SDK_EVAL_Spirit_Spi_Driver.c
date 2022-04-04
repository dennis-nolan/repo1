
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: SDK_EVAL_Spirit_Spi_Driver
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
/**
* @file    SDK_EVAL_Spirit_Spi_Driver.c
* @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.0
 * @date    August 7, 2012
* @brief    This file provides all the low level SPI API to access to SPIRIT using a software watchdog timer to avoid stuck situation.
* @details
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*
* <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
*
*/
// Processor: STM32F103R
// TOOLS: IAR Workbench
// DATE:
// CONTENTS: This file contains
//------------------------------------------------------------------------------
// HISTORY: This file
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#include "defs.h"
#include "SDK_EVAL_Spirit_Spi_Config.h"
#include "SPIRIT_Types.h"
#include "cm2coretypesheader.h"
#include "JL_SpiritDriver.h"
  //-------------SPITESTING-----
  // set to "1" to
#define SPITESTING 1


#ifdef FREERTOS
#define SPI_ENTER_CRITICAL()           xSemaphoreTake(xSpiMutex, portMAX_DELAY);
#define SPI_EXIT_CRITICAL()            xSemaphoreGive(xSpiMutex);
#else
//#define SPI_ENTER_CRITICAL()            {};  //__disable_interrupt();   //SpiritGPIOInterruptCmd(DISABLE);
//#define SPI_EXIT_CRITICAL()             {};  //__enable_interrupt();   //SpiritGPIOInterruptCmd(ENABLE);

static bool spiCritical = false;

#define SPI_ENTER_CRITICAL() {spiCritical = true;}
#define SPI_EXIT_CRITICAL() {spiCritical = false;}

bool SpiAlreadyActive(void)
{
return spiCritical;
}
#endif



#define CS_TO_SCLK_DELAY  0x0001
#define CLK_TO_CS_DELAY   0x0001


#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/

#define LINEAR_FIFO_ADDRESS 0xFF  /*!< Linear FIFO address*/

#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)  /*!< macro to build the header byte*/
#define WRITE_HEADER    BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER     BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER  BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command header byte*/


#define SPIRIT_SPI_PERIPH_NB            SPI3
#define SPI_FLAG_TXE                    SPI_I2S_FLAG_TXE
#define SPI_GetFlagStatus               SPI_I2S_GetFlagStatus
#define SPI_FLAG_RXNE                   SPI_I2S_FLAG_RXNE
#define SPI_SendData                    SPI_I2S_SendData
#define SPI_ReceiveData                 SPI_I2S_ReceiveData
#define SPI_DeInit                      SPI_I2S_DeInit

#define SpiritSPICSLow()        {   GPIO_WriteBit(GPIOA,GPIO_Pin_15,Bit_RESET); }
#define SpiritSPICSHigh()       {   GPIO_WriteBit(GPIOA,GPIO_Pin_15,Bit_SET); }

//---------------------GLOBAL VARIABLES-----------------------------------
#define SPIWDOGTIME 500   //was 5000
uint16_t spiWDOG;

//---------------------LOCAL VARIABLES------------------------------------
typedef SpiritStatus StatusBytes;
uint8_t spiTimeout = 0;

//---------------------LOCAL FUNCTION PROTOTYPES--------------------------

void SpiritGPIOInterruptCmd(FunctionalState xNewState);


//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// ---------------------------GLOBAL FUNCTIONS ----------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   SpiritSpiInit
//------------------------------------------------------------------------------
// This function
//==============================================================================
void SpiritSpiInit(CM2PortType portType)
{
  SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  spiTimeout = 0;
  //-----------------------------------
  // Enable SPI and GPIO clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
  //----------------------------------------
  // Configure SPI1 pins: SCK, MISO and MOSI
  //-----------------------------------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //------------------------------------
  // Configure SPI pin: CS
  //------------------------------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOA,GPIO_Pin_15,Bit_SET);
  //------------------------------
  // Configure SPI peripheral
  //--------------------------------
  SPI_DeInit(SPI3);
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;//SPI_BaudRatePrescaler_4; //SPI_BaudRatePrescaler_8; /*Baud_Rate= fpclk/4=32MHz/4=8MHZ*/
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI3, &SPI_InitStructure);

  SPI_Cmd(SPI3, ENABLE);

  SpiritSPICSHigh();

}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   SdkEvalSpiWriteRegisters
//------------------------------------------------------------------------------
// This function   Write single or multiple SPIRIT register
// @param  cRegAddress: base register's address to be write
// @param  cNbBytes: number of registers and bytes to be write
// @param  pcBuffer: pointer to the buffer of values have to be written into registers
// @retval SPIRIT status
//==============================================================================
//StatusBytes SpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
SpiritStatus SpiWriteRegisters(CM2PortType portFlags, uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t header[2];
  uint16_t tmpstatus = 0x0000;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;
  //--------------------------
  // Build the header bytes
  //--------------------------
  header[0]=WRITE_HEADER;
  header[1]=cRegAddress;

  SPI_ENTER_CRITICAL();
  //---------------------------------
  // Puts the SPI chip select low to start the transaction
  //---------------------------------
  SpiritSPICSLow();
  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);
  //-------------------------
  /* Writes the header bytes and read the SPIRIT status bytes */
  for(int i=0; i<2; i++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }

  /* Writes the registers according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }
    SPI_SendData(SPIRIT_SPI_PERIPH_NB, pcBuffer[index]);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }
    SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB);
  }

  //-----------------------------
  // To be sure to don't rise the Chip
  // Select before the end of last sending
  //-----------------------------
  spiWDOG = SPIWDOGTIME;
  while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
  if (spiWDOG == 0)
  {
    spiTimeout = 1;
  }

  /* Puts the SPI chip select high to end the transaction */
  SpiritSPICSHigh();

  SPI_EXIT_CRITICAL();

  if (spiTimeout)
	SpiritSpiError(portFlags);

  return *status;

}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   SpiritSpiInit
//------------------------------------------------------------------------------
// This function
//==============================================================================

/**
* @brief  Read single or multiple SPIRIT register
* @param  cRegAddress: base register's address to be read
* @param  cNbBytes: number of registers and bytes to be read
* @param  pcBuffer: pointer to the buffer of registers' values read
* @retval SPIRIT status
*/
//StatusBytes SpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
SpiritStatus SpiReadRegisters(CM2PortType portFlags, uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;

  uint8_t header[2];
  uint8_t dummy=0xFF;

  /* Built the header bytes */
  header[0]=READ_HEADER;
  header[1]=cRegAddress;

  SPI_ENTER_CRITICAL();

  /* Put the SPI chip select low to start the transaction */
  SpiritSPICSLow();

  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);

  /* Write the header bytes and read the SPIRIT status bytes */
  for(int i=0; i<2; i++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }

  /* Read the registers according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, dummy);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    *pcBuffer = SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB);
    pcBuffer++;
  }
  spiWDOG = SPIWDOGTIME;
  while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
  if (spiWDOG == 0)
  {
    spiTimeout = 1;
  }
  /* Put the SPI chip select high to end the transaction */
  SpiritSPICSHigh();

  SPI_EXIT_CRITICAL();

  if (spiTimeout)
	SpiritSpiError(portFlags);

  return *status;

}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   SpiritSpiInit
//------------------------------------------------------------------------------
// This function
//==============================================================================

/**
* @brief  Send a command
* @param  cCommandCode: command code to be sent
* @retval SPIRIT status
*/
//StatusBytes SpiCommandStrobe(uint8_t cCommandCode)
SpiritStatus SpiCommandStrobe(CM2PortType portFlags, uint8_t cCommandCode)
{
  uint8_t header[2];
  uint16_t tmpstatus = 0x0000;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;

  /* Built the header bytes */
  header[0]=COMMAND_HEADER;
  header[1]=cCommandCode;

  SPI_ENTER_CRITICAL();

  /* Puts the SPI chip select low to start the transaction */
  SpiritSPICSLow();

  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);

  /* Writes the header bytes and read the SPIRIT status bytes */
  for(int i=0; i<2; i++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }
  spiWDOG = SPIWDOGTIME;
  while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
  if (spiWDOG == 0)
  {
    spiTimeout = 1;
  }

  /* Puts the SPI chip select high to end the transaction */
  SpiritSPICSHigh();

  SPI_EXIT_CRITICAL();

  if (spiTimeout)
	SpiritSpiError(portFlags);

  return *status;

}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   SpiritSpiInit
//------------------------------------------------------------------------------
// This function
//==============================================================================

/**
* @brief  Write data into TX FIFO
* @param  cNbBytes: number of bytes to be written into TX FIFO
* @param  pcBuffer: pointer to data to write
* @retval SPIRIT status
*/
//StatusBytes SpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
SpiritStatus SpiWriteFifo(CM2PortType portFlags, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;

  uint8_t header[2];

  /* Built the header bytes */
  header[0]=WRITE_HEADER;
  header[1]=LINEAR_FIFO_ADDRESS;

  SPI_ENTER_CRITICAL();

  /* Put the SPI chip select low to start the transaction */
  SpiritSPICSLow();

  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);

  /* Write the header bytes and read the SPIRIT status bytes */
  for(int i=0; i<2; i++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }

  /* Write the data into the FIFO according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, pcBuffer[index]);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB);
  }

  /* To be sure to don't rise the Chip Select before the end of last sending */
  spiWDOG = SPIWDOGTIME;
  while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
  if (spiWDOG == 0)
  {
    spiTimeout = 1;
  }

  /* Put the SPI chip select high to end the transaction */
  SpiritSPICSHigh();

  SPI_EXIT_CRITICAL();

  if (spiTimeout)
	SpiritSpiError(portFlags);

  return *status;

}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   SpiritSpiInit
//------------------------------------------------------------------------------
// This function
//==============================================================================
#if 0
SpiritStatus SpiritSpiWriteLinearFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;

  uint8_t header[2];

  /* Built the header bytes */
  header[0]=WRITE_HEADER;
  header[1]=LINEAR_FIFO_ADDRESS;

  SPI_ENTER_CRITICAL();

  /* Put the SPI chip select low to start the transaction */
  SpiritSPICSLow();

  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);

  /* Write the header bytes and read the SPIRIT status bytes */
  for(int i=0; i<2; i++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }

  /* Write the data into the FIFO according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, pcBuffer[index]);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }
    SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB);
  }

  /* To be sure to don't rise the Chip Select before the end of last sending */
  spiWDOG = SPIWDOGTIME;
  while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
  if (spiWDOG == 0)
  {
    spiTimeout = 1;
  }
  /* Put the SPI chip select high to end the transaction */
  SpiritSPICSHigh();

  SPI_EXIT_CRITICAL();

  if (spiTimeout)
	SpiritSpiError(portFlags);

  return *status;

}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   SpiritSpiInit
//------------------------------------------------------------------------------
// This function
//==============================================================================

/**
* @brief  Read data from RX FIFO
* @param  cNbBytes: number of bytes to read from RX FIFO
* @param  pcBuffer: pointer to data read from RX FIFO
* @retval SPIRIT status
*/
SpiritStatus SpiritSpiReadLinearFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;

  SPI_ENTER_CRITICAL();

  uint8_t header[2];
  uint8_t dummy=0xFF;

  /* Built the header bytes */
  header[0]=READ_HEADER;
  header[1]=LINEAR_FIFO_ADDRESS;

  /* Put the SPI chip select low to start the transaction */
  SpiritSPICSLow();

  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);

  /* Write the header bytes and read the SPIRIT status bytes */
  for(int i=0; i<2; i++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }

  /* Read the data from the FIFO according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, dummy);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    *pcBuffer = SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB);
    pcBuffer++;
  }

  /* To be sure to don't rise the Chip Select before the end of last sending */
  spiWDOG = SPIWDOGTIME;
  while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
  if (spiWDOG == 0)
  {
    spiTimeout = 1;
  }

  /* Put the SPI chip select high to end the transaction */
  SpiritSPICSHigh();

  SPI_EXIT_CRITICAL();

  if (spiTimeout)
	SpiritSpiError(portFlags);

  return *status;

}
#endif
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   SpiritSpiInit
//------------------------------------------------------------------------------
// This function
//==============================================================================
SpiritStatus SpiReadFifo(CM2PortType portFlags, uint8_t cNbBytes, uint8_t* pcBuffer)
//StatusBytes SpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint16_t tmpstatus = 0x0000;
  SpiritStatus *status=(SpiritStatus *)&tmpstatus;

  SPI_ENTER_CRITICAL();

  uint8_t header[2];
  uint8_t dummy=0xFF;

  /* Built the header bytes */
  header[0]=READ_HEADER;
  header[1]=LINEAR_FIFO_ADDRESS;

  /* Put the SPI chip select low to start the transaction */
  SpiritSPICSLow();

  for(volatile uint16_t i=0;i<CS_TO_SCLK_DELAY;i++);

  /* Write the header bytes and read the SPIRIT status bytes */
  for(int i=0; i<2; i++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, header[i]);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    tmpstatus += ((uint16_t)(SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB)))<<((1-i)*8);
  }

  /* Read the data from the FIFO according to the number of bytes */
  for(int index=0; index<cNbBytes; index++)
  {
    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    SPI_SendData(SPIRIT_SPI_PERIPH_NB, dummy);

    spiWDOG = SPIWDOGTIME;
    while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_RXNE) == RESET)&&(spiWDOG >0));
    if (spiWDOG == 0)
    {
      spiTimeout = 1;
    }

    *pcBuffer = SPI_ReceiveData(SPIRIT_SPI_PERIPH_NB);
    pcBuffer++;
  }

  /* To be sure to don't rise the Chip Select before the end of last sending */
  spiWDOG = SPIWDOGTIME;
  while ((SPI_GetFlagStatus(SPIRIT_SPI_PERIPH_NB, SPI_FLAG_TXE) == RESET)&&(spiWDOG >0));
  if (spiWDOG == 0)
  {
    spiTimeout = 1;
  }

  /* Put the SPI chip select high to end the transaction */
  SpiritSPICSHigh();

  SPI_EXIT_CRITICAL();

  if (spiTimeout)
	SpiritSpiError(portFlags);

  return *status;

}




/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/