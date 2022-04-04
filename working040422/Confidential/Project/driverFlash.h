//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: DRIVERFLASH.H
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#ifndef __DRIVERFLASH
#define __DRIVERFLASH

#include "defs.h"
 
 
 
//---------------------GLOBAL DEFINITIONS--------------------------------
#define MAX_VERSION_LENGTH			16    
typedef struct
{
	uint32_t checksum;
	uint32_t appLength;
	uint32_t checksumStartOffset;
 	uint32_t	secondaryChecksumAddr;       
	uint32_t	version;
	uint8_t		verString[MAX_VERSION_LENGTH];
	uint8_t		verLength;
} AppInfo;

#define APP_INFO_START				0x89ABCDEF
#define APP_INFO_END				0xFEDCBA98
#define MAX_SEARCH_OFFSET			512

#define ApplicationAddress        0x08003000
#define Temp_ApplicationAddress   0x08021800 

#define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
#define FLASH_SIZE                        (0x40000)  /* 256 KBytes */
#define APPSIZEALLOWED             0x1E800  

extern uint16_t newFirmwareTimer; 
extern uint8_t downloadActive;    
extern AppInfo app; 
extern AppInfo boot; 
 
uint32_t FLASH_PagesMask(__IO uint32_t Size);
bool FlashWriteScratch32(uint32_t offset, uint32_t data);
bool FlashEraseScratch(void);
uint8_t GetAppInfo(uint32_t startAddress, AppInfo* info);
void DownloadSupTask(void);
uint32_t ComputeChecksum(uint32_t appStart, uint32_t	appLength, uint32_t checksumOffset, uint32_t secondaryChecksumAddr);

#endif   