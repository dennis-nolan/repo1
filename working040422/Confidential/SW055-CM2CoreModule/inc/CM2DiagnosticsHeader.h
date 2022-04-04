/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
******************************* END VERSION INFO ******************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DIAGNOSTICS_HEADER
#define DIAGNOSTICS_HEADER

#include "CommonTypes.h"

enum {DIAG_STORAGE_ALL=0, DIAG_STORAGE_RAM, DIAG_STORAGE_FLASH, DIAG_STORAGE_EXTERNAL};
enum {SEVERITY_DEBUG=0, SEVERITY_WARNING, SEVERITY_RECOVERABLE_ERROR, SEVERITY_FATAL_ERROR};

void DiagnosticsInit(uint8_t ramCacheSize);
void DiagnosticsDeinit(void);
void WriteDiagnosticLog(uint16_t source, uint8_t severity, uint8_t length, bool includeVIT, uint8_t* log, bool immediate);
void ReadDiagnosticLogs(uint8_t storageLocation, uint8_t*** logArray, uint8_t* numLogs);

//close recursive include ifdef
#endif