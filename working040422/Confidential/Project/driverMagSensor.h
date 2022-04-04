//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: DRIVER_ACCEL.H
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#ifndef __DRIVER_ACCEL_H__
#define __DRIVER_ACCEL_H__

#include "stm32f10x.h"



//---------------------GLOBAL DEFINITIONS--------------------------



#define ACCEL_READ		0x40		// Read from accelerometer register
#define ACCEL_WRITE		0x80		// Write to accelerometer register
//#define ACCEL_INCR		0x40		// increment register address on each I/O
#define ACCEL_ADDR_MASK	        0x3F		// Register address mask

//-----------------------------------------------------------
// Defines for the SPI and GPIO pins used to drive the SPI Flash
//-----------------------------------------------------------
#define SPI_FLASH                 SPI2
#define SPI_FLASH_CLK             RCC_APB2Periph_SPI2
#define SPI_FLASH_GPIO            GPIOB
#define SPI_FLASH_GPIO_CLK        RCC_APB2Periph_GPIOB
#define SPI_FLASH_PIN_SCK         GPIO_Pin_13
#define SPI_FLASH_PIN_MISO        GPIO_Pin_14
#define SPI_FLASH_PIN_MOSI        GPIO_Pin_15

#define SPI_FLASH_CS              GPIO_Pin_12
#define SPI_FLASH_CS_GPIO         GPIOB
#define SPI_FLASH_CS_GPIO_CLK     RCC_APB2Periph_GPIOB

//---------------------------------
// Exported macro
//---------------------------------
  // Select SPI FLASH: Chip Select pin low
  //---------------------------------
#define SPI_FLASH_CS_LOW()       GPIO_ResetBits(SPI_FLASH_CS_GPIO, SPI_FLASH_CS)
  //----------------------------------
  // Deselect SPI FLASH: Chip Select pin high
  //----------------------------------
#define SPI_FLASH_CS_HIGH()      GPIO_SetBits(SPI_FLASH_CS_GPIO, SPI_FLASH_CS)

//---------------------GLOBAL VARIABLES--------------------------

//---------------------GLOBAL PROTOTYPES--------------------------
uint8_t MagRead(uint8_t address);
void MagWrite(uint8_t address, uint8_t data);
uint8_t MagInit(void);
void MagChangeRegister(uint8_t whichRegister,uint8_t valueInHex);
uint16_t MagRead16(uint8_t address);
#endif //__ACCEL_H__



