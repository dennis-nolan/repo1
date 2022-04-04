//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: spiritInterfaceHeader.h
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// LAST MODIFIED:
//------------------------------------------------------------------------------
#ifndef __SPIRITINTERFACEHEADER__
#define __SPIRITINTERFACEHEADER__

#include "CM2CoreTypesHeader.h"
#include "appTestSupport.h"

//---------------------GLOBAL DEFINITIONS--------------------------
#define TABLEFLAG_READ_ONLY     0x01
#define TABLEFLAG_WRITEABLE     0x00

typedef struct CM2Message_tag
{
    Opcode        op;
    union
    {
        struct
        {
            uint8_t subcmd;
            uint8_t speed;
            uint8_t torque;
            uint8_t angle[2];
        }                      doda;
        struct
        {
            uint8_t tblId;
            uint8_t offset;
            uint8_t length;
        }                      tblRead;
        struct
        {
            struct {
                uint8_t action;
                uint8_t tblId;
                uint8_t offset;
            }       hdr;
            uint8_t data[1];
        }                      tblData;
    }                              cmd;
} __packed CM2CmdField_t;

typedef struct
{
    uint16_t crc16;
    uint8_t setTarget:2;    // target enum
    uint8_t holdRelease:2;  // hold/release enum
    uint8_t setHome:2;      // set home enum
    uint8_t limitSpeed:1;   //set a max speed for this move?
    uint8_t minimumTime:1;  //force a minimum movement time?
    uint16_t speedLimit;    // deg/sec
    uint16_t time;          // mS
    int16_t angle;          // degrees * COUNTS_PER_DEGREE
    int16_t homeAngle;
    uint16_t reserved2;
} __packed AdvancedSteerCmd;


enum {
    HR_RELEASE,     //always forced
    HR_HOLD,        //
    HR_FORCE_HOLD,  //tries to re-hold even if it thinks it is already holding
    HR_NO_ACTION    //no change
}; // hold/release settings

enum {
    TA_ABSOLUTE,
    TA_RELATIVE,
    TA_RESERVED,
    TA_NO_ACTION    //no change
}; // target angle settings

enum {
    SH_HERE,        //
    SH_RELATIVE,    //
    SH_ABSOLUTE,    //
    SH_NO_ACTION    //no change
}; // hold/release settings

//---------------------GLOBAL VARIABLES--------------------------
extern uint32_t tableContents[10];
extern uint16_t dupTimer;
#define MAX_DUP_ENTRIES 5
extern uint16_t dupTimers[MAX_DUP_ENTRIES];

extern uint16_t opModeTimer;
//---------------------GLOBAL PROTOTYPES--------------------------
void AppSpiritInit(void);
RxPacketResult PacketRX(CM2Packet* packet);
void TxBatterySaver(bool enable);
bool CM2SendStatusTable(uint8_t *ptr);
bool SendOPVERSION(CM2Packet *packet);
uint16_t CM2SendPing(uint32_t ID,uint16_t offset,uint32_t myID);
bool CM2SendACK(CM2Packet *ptr);
bool SendOPTESTResult(CM2Packet *ptr,uint8_t broadcast,OpTest *opTester);
uint16_t SetSpeed(uint16_t spdcmd);
void AppModeHandleTask(void);
uint8_t AppModeGet(void);
void MomentaryTimeOutTask(void);
void CM2SupervisorTask(void);
#endif
