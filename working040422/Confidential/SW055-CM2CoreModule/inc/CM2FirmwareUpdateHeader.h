
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FW_UPDATE_HEADER
#define FW_UPDATE_HEADER

#include "CommonTypes.h"

#define FWU_LL_TIMEOUT 1000 // 1 second
#define FWU_HL_TIMEOUT 3000 // 3 seconds
#define FWU_HL_REMOTETIMEOUT 1800  //1.8 seconds


typedef struct 
{
    uint32_t destID;
    uint32_t binaryStartAddr;
    uint32_t binaryLength; 
    uint8_t  deviceDest;
    uint8_t  destType; 
    uint8_t  sourceType; 
    uint32_t timeoutLLACK;
    uint32_t timeoutHLACK;
}FW_SESSION_INFO;  

enum {UPDATE_STATUS_NONE=0,UPDATE_STATUS_IN_PROGRESS,UPDATE_STATUS_FAIL,UPDATE_STATUS_SUCCESS};

void BeginFirmwareUpdate(FW_SESSION_INFO *sessionInfo);
void AbortFirmwareUpdate(void);
uint8_t GetFirmwareUpdateStatus(void);
void RunFirmwareUpdateTask(void);
void FWUpdateAckResult(uint16_t mac, bool highLevel, bool success,uint8_t reason);
void SetFirmwareUpdateStatusNone(void);
uint32_t RunFirmwareGetBytesRemaining(void);
// close recursive include ifdef
#endif