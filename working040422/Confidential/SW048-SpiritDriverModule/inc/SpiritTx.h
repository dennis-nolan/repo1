
// Define to prevent recursive inclusion
#ifndef SPIRIT_TX_H
#define SPIRIT_TX_H

#include "SpiritPrivateUtils.h"
#include "CM2CoreTypesHeader.h"

//update TX state machine for new information
void RunTxStateMachine(SpiritDevice* device);

//begin a new transmission
TxStatus BeginNonBlockingTx(SpiritDevice* device, CM2Packet* packet);

//try to leave TX in ISR
void AttemptTxToRxTransition(SpiritDevice* device);

#endif