/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/11/2016: Added recursive include prevention macros
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	10/07/2015: Initial creation.
	04/22/2019: Cleanup unusued functions/paramaters
******************************* END VERSION INFO ******************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CM2ROUTING_HEADER
#define CM2ROUTING_HEADER

#include "CM2CoreTypesHeader.h"

void RouterInit(void);
void RouterDeinit(void);
bool RoutePacket(CM2Packet* packet);
void RouteNextPacket(void);
bool IsRouterQueueFull(void);
//void RegisterNoAckCallback(void (*NoAckCallback)(uint16_t mac));
void RegisterAckResultCallback(void (*AckResultCallback)(uint16_t mac, bool success));
void AbortRetries(uint16_t mac);
void AbortAllRetries(void);
void AbortSimilar(CM2Packet* packet);

// close recursive include ifdef
#endif