//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: driverAccel.c 
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Processor: STM32F103R
// TOOLS: IAR Workbench 
// DATE:
// CONTENTS: This file contains  
//------------------------------------------------------------------------------
// HISTORY: This file  
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//#include "defs.h"
#include "driverMagSensor.h"
 

 
//---------------------GLOBAL VARIABLES-----------------------------------
 
//---------------------LOCAL VARIABLES------------------------------------
uint16_t accel_packet;   
uint16_t magRegisters[9]; 
uint16_t magRegister27h; 
  //--------------------------------
  // Initial position in free-space
  //--------------------------------
 
//---------------------LOCAL FUNCTION PROTOTYPES--------------------------  
 
 

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// ---------------------------GLOBAL FUNCTIONS ----------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
uint16_t valueRead; 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   AccelRead
//------------------------------------------------------------------------------
// This function Read a byte from the accelerometer. 
//==============================================================================
uint8_t MagRead(uint8_t address)
{
  uint8_t x;
  uint16_t temp;

  //-------------------------
  // Select the FLASH: Chip Select low 
  SPI_FLASH_CS_LOW();
   
  //---------------------------------
  //Send the accelerometer command byte
        
        /* Loop while DR register in not emplty */
        while (SPI_I2S_GetFlagStatus(SPI_FLASH, SPI_I2S_FLAG_TXE) == RESET);
        
        /* Create packet */
        accel_packet = ((address & ACCEL_ADDR_MASK) | ACCEL_READ) << 8;
      
        /* Send byte through the SPI1 peripheral */
        SPI_I2S_SendData(SPI_FLASH, accel_packet);
      
        /* Wait to receive a byte */
        while (SPI_I2S_GetFlagStatus(SPI_FLASH, SPI_I2S_FLAG_RXNE) == RESET);
        
        
        /* Return the byte read from the SPI bus */
        temp = SPI_I2S_ReceiveData(SPI_FLASH); 
        valueRead = temp;
        valueRead = valueRead>>8; 
        x = valueRead;
        
 
        /* Deselect the FLASH: Chip Select high */
        SPI_FLASH_CS_HIGH();         

	// Return data byte
	return x;
} 
  

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   AccelWrite
//------------------------------------------------------------------------------
// This function will write a byte to the accelerometer
//==============================================================================
void MagWrite(uint8_t address, uint8_t data)
{
  //--------------------------------
  // Select the FLASH: Chip Select low 
  SPI_FLASH_CS_LOW();

  //------------------------------
  // Loop while DR register in not emplty 
  while (SPI_I2S_GetFlagStatus(SPI_FLASH, SPI_I2S_FLAG_TXE) == RESET);
  //-----------------------
  // Create packet 
  //-----------------------
  accel_packet = ((address & ACCEL_ADDR_MASK) | ACCEL_WRITE/* | ACCEL_INCR*/) << 8;
  accel_packet = accel_packet | data;
  //-------------------------------------      
  // Send byte through the SPI1 peripheral 
  SPI_I2S_SendData(SPI_FLASH, accel_packet);
  //--------------------------------------      
  // Wait to receive a byte 
  while (SPI_I2S_GetFlagStatus(SPI_FLASH, SPI_I2S_FLAG_RXNE) == RESET);
  //------------------------
  // Read dummy data
  SPI_I2S_ReceiveData(SPI_FLASH);     
  //-----------------------
  // Deselect the FLASH: Chip Select high 
  SPI_FLASH_CS_HIGH();        
}

uint8_t test; 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   AccelIinit
//------------------------------------------------------------------------------
// This function will Initialize the accelerometer
//==============================================================================
uint8_t MagInit(void)
{
  uint8_t i; 
  uint16_t j; 

  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  //-----------------------------------
  // Enable SPI and GPIO clocks 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
  //----------------------------------------
  // Configure SPI1 pins: SCK, MISO and MOSI 
  //-----------------------------------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13| GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //---------------------------------------
  // Configure I/O for Flash Chip select 
  GPIO_InitStructure.GPIO_Pin = SPI_FLASH_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SPI_FLASH_CS_GPIO, &GPIO_InitStructure);
  //-------------------------------------
  // Deselect the FLASH: Chip Select high 
  SPI_FLASH_CS_HIGH();
  //------------------------
  // SPI configuration 
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;  //SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);
  //-----------------------
  // Enable the SPI  
  SPI_Cmd(SPI_FLASH, ENABLE);
               
  
  //-----------------------------------------
  // Set up the Interrupt pin configuration(s)
//  AccelWrite(MEMS_CTRL_REG3, ( MEMS_ICFG_FF_WU ));
 
  
  //---------------------------------
  // Clear the interrupt latch
//  MagRead(0);
  for (i=0;i<8;i++)
  {
    MagRead(i);
//    if (i>0)
//    {
      magRegisters[i] = MagRead(i);  
//    }  
  }
  
  MagRead(27);
  magRegister27h = MagRead(27);
  

  //-----------following writes the value 0x60 to register 0x07 and
  // then reads it back. 
  MagChangeRegister(0x07,0x60);  
  
  return 1;
}


void MagChangeRegister(uint8_t whichRegister,uint8_t valueInHex)
{
  uint16_t j; 
  
  if ((whichRegister <8)||(whichRegister == 0x27))
  {
    MagWrite(whichRegister,valueInHex);
    for (j=0;j<0xfff;j++)
    {
      test++;
    }
    if (whichRegister <8)
    {  
      magRegisters[whichRegister] = MagRead(whichRegister); 
    }  
    if (whichRegister == 27)
    {  
      magRegister27h = MagRead(whichRegister); 
    }    
  }
}  

 
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// ---------------------------LOCAL FUNCTIONS ----------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

 

 




