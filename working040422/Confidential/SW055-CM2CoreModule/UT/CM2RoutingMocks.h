
#include "CM2CoreTypesHeader.h"

CM2Packet* GetLastTx(uint8_t num);
bool Transmit1(CM2Packet* packet);
bool Transmit2(CM2Packet* packet);
bool Transmit3(CM2Packet* packet);
bool Transmit4(CM2Packet* packet);
void ResetRoutingMocks(void);
bool Ready(void);