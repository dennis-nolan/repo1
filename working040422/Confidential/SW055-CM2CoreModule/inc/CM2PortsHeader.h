/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/11/2015: Initial creation.
******************************* END VERSION INFO ******************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CM2PORTS_HEADER
#define CM2PORTS_HEADER

#include "CM2CoreTypesHeader.h"

void PortHandlingInit(void);
void RunCM2PortsTasks(void);
int RegisterPort(Port* portInfo);
TxStatus TxOnSpecifiedPort(CM2Packet* packet, bool blocking);
bool RxBytes(CM2PortType portType, uint8_t num, uint8_t* data);
void TxCallback(CM2PortType portFlags, uint16_t mac, TxStatus status);
void ReportMacResult(CM2PortType type, bool success);
Port* GetPort(CM2PortType portType);

// close recursive include ifdef
#endif