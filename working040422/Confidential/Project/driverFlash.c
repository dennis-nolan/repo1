//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: driverFlash.c
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Processor: STM32F103R
// TOOLS: IAR Workbench
// DATE:
// CONTENTS: This file contains
//------------------------------------------------------------------------------
// HISTORY: This file
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#include "defs.h"
#include "driverFlash.h"


//---------------------GLOBAL VARIABLES-----------------------------------


//---------------------LOCAL VARIABLES------------------------------------
  //--------------------------
  //

AppInfo app;
AppInfo boot;

uint32_t FlashDestination = Temp_ApplicationAddress; /* Flash user program offset */
uint32_t FlashSource = ApplicationAddress;
uint16_t PageSize = PAGE_SIZE;
uint32_t EraseCounter = 0x0;
uint32_t NbrOfPage = 0;
FLASH_Status FLASHStatus = FLASH_COMPLETE; // added when adding FLASH protect
uint8_t *RamSource;

uint16_t newFirmwareTimer;

uint8_t downloadActive=0;

//--------------ymFirmwareTimeout------
// interpacket timing for wrapping up packets if
// needed.
//-------------------------------------
#define YM_FIRMWARE_TIMEOUT     1000  //second?
uint16_t ymFirmwareTimeout;


//---------------------LOCAL FUNCTION PROTOTYPES--------------------------


//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// ---------------------------GLOBAL FUNCTIONS ----------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX



//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
void DownloadSupTask(void)
{

    downloadActive = 0;
    newFirmwareTimer = 0;

}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
uint32_t FLASH_PagesMask(__IO uint32_t Size)
{
  uint32_t pagenumber = 0x0;
  uint32_t size = Size;

  if ((size % PAGE_SIZE) != 0)
  {
    pagenumber = (size / PAGE_SIZE) + 1;
  }
  else
  {
    pagenumber = size / PAGE_SIZE;
  }
  return pagenumber;

}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION: FlashWriteScratch64
//------------------------------------------------------------------------------
// This function
//==============================================================================
bool FlashWriteScratch32(uint32_t offset, uint32_t data)
{

  bool status = TRUE;

  if (downloadActive != 0)
  {

    FLASH_Unlock();
    //write flash
    if (FLASH_ProgramWord(Temp_ApplicationAddress+offset, data) != FLASH_COMPLETE)
    {
      FLASH_Lock();
      status = FALSE;
    }
    else
    {
        newFirmwareTimer = YM_FIRMWARE_TIMEOUT;
    }
    //wrap up
    FLASH_Lock();
 }
 else
 {
   status = FALSE;
 }
  return status;
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:  FlashEraseScratch
//------------------------------------------------------------------------------
// This function executes a flash erase of the specified size at the address
//==============================================================================
bool FlashEraseScratch(void)
{
   bool status = TRUE;
  //--------------------------------------
  // Unlock the Flash to enable the
  // flash control register access
  //------------------------------------==
  FLASH_Unlock();
  //---------------------------------------
  // Get the number of pages to erase from 1st page
  //---------------------------------------
  NbrOfPage = FLASH_PagesMask(APPSIZEALLOWED);
  //--------------------------------------
  // Erase the FLASH pages
  for (EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
    IWDG_ReloadCounter();
    FLASHStatus = FLASH_ErasePage(FlashDestination + (PageSize * EraseCounter));
    if (FLASHStatus != FLASH_COMPLETE)
    {
      FLASHStatus = FLASH_ErasePage(FlashDestination + (PageSize * EraseCounter));
      if (FLASHStatus != FLASH_COMPLETE)
      {
        status = FALSE;
      }
    }
  }
  if (status == TRUE)
  {
    newFirmwareTimer = YM_FIRMWARE_TIMEOUT;
    downloadActive = TRUE;
  }
  return status;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION: GetAppInfo
//------------------------------------------------------------------------------
// This function parses app info starting at the given addr (failure returns 0)
//==============================================================================
uint8_t GetAppInfo(uint32_t startAddress, AppInfo* info)
{
	uint8_t appInfoStarted = 0;
	uint8_t i=0;
	info->secondaryChecksumAddr = 0;

	//search starting at the given address for the markers that designate the app info section
	for (uint32_t addr = startAddress; addr < startAddress + MAX_SEARCH_OFFSET; addr += 4)
	{
		if (*((uint32_t*)addr) == APP_INFO_START)
		{
			info->checksum = *(uint32_t*)(addr + 4);
			info->appLength = *(uint32_t*)(addr + 8);
			info->version = *(uint32_t*)(addr + 12);
			info->verLength = (uint8_t)(*((uint32_t*)(addr + 16)));

			if (info->verLength > MAX_VERSION_LENGTH)
				info->verLength = MAX_VERSION_LENGTH;

			appInfoStarted = 1;
			addr += 16;
		}
		else if (appInfoStarted)
		{
			if (*((uint32_t*)addr) == APP_INFO_END)
			{
				//save address to start checksum at
				info->checksumStartOffset = addr + 4 - startAddress;

				//success
				return 1;
			}
			else if (i < info->verLength)
			{
				//save version string
				info->verString[i++] = *(uint8_t*)(addr);
			}
			else
			{
				//secondary checksum found!
				info->secondaryChecksumAddr = addr;
				info->checksum = *(uint32_t*)addr;
			}
			//else we should have gotten an end marker by now...
		}
		//else we haven't found the app info start marker yet
	}

	//we didn't find both a start and end marker
	return 0;
}

uint8_t mytest;
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION: ComputeChecksum
//------------------------------------------------------------------------------
// This function computes the checksum of a given address range
//==============================================================================
uint32_t ComputeChecksum(uint32_t appStart, uint32_t	appLength, uint32_t checksumOffset, uint32_t secondaryChecksumAddr)
{
	//setup for the legacy checksum by default
	uint32_t sum = 0xA6A6A6A6;
	uint32_t firstAddress = appStart + checksumOffset;

	if (secondaryChecksumAddr != 0)
	{
		//compute newer checksum instead
		firstAddress = appStart;
		sum = 0xC8C8C8C8;
	}

	for (uint32_t addr = firstAddress; addr <= (appStart+appLength-4); addr+=4)
	{
                if ((addr%256)== 0)
                {
                    mytest++;
                }
		if (addr != secondaryChecksumAddr)
			sum += *((uint32_t*)addr);
	}

	return sum;
}
