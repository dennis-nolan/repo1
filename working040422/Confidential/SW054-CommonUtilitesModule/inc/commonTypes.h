/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COMMONTYPES_HEADER
#define COMMONTYPES_HEADER

#include <stdint.h>

#ifndef __cplusplus
#ifndef MYBOOL
#define MYBOOL
typedef enum {FALSE=0, TRUE=1} bool;
#endif
#else
#define FALSE false
#define TRUE true
#endif

typedef enum {POWER_OFF=0, POWER_FULL, POWER_LOW, POWER_SLEEP} PowerMode;

//close recursive include ifdef
#endif