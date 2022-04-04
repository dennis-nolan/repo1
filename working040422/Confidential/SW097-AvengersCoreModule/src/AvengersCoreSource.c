
#include "AvengersCoreHeader.h"
#include "CM2CoreHeader.h"
#include "CM2RoutingHeader.h"
#include "CM2PortsHeader.h"
#include "CM2SecurityHeader.h"

bool PlcPacketFilter(CM2Packet* packet)
{
    if (0xFFFFFFFF == GetID())
    {
        //don't respond to anything but OP_VERSION until we have an ID
        return (bool)(OP_VERSION == packet->command[0]);
    }

    //only respond when it's our ID
    return (bool)(GetID() == packet->destID);
}

static CM2Packet ack =
{
    PORT_INTERNAL,
    PORT_UNKNOWN,       //destination device
    0,                  //retries
    PROTOCOL_VERSION,
    0,                  //src type
    0,                  //dest type
    0x00000000,         //src ID
    0x00000000,         //dest ID
    0x00,               //no subaddress
    0x03,               //3 byte payload
    0x0000,             //MAC to be filled out before acking
    {OP_LOW_LEVEL_ACK,0,0} //will contain opcode + msg MAC
};
void HandlePlcAcking_Common(CM2Packet* packet, uint8_t devType, Opcode ackType, uint8_t nackReason, bool blocking)
{
    if (OP_ACK != ackType && OP_NACK != ackType && OP_LOW_LEVEL_ACK != ackType)
        return;

    if (OP_LOW_LEVEL_ACK == packet->command[0])
        return; //don't ack acks

    if (GetID() != packet->destID)// && GetID() != packet->srcID)
        return; //not part of our PLC

    if (devType != packet->destType)
        return; //not talking to us

    //populate the packet details
    ack.deviceSource = PORT_INTERNAL;
    ack.deviceDest = packet->deviceSource;
    ack.srcID = GetID();
    ack.srcType = devType;
    ack.destType = packet->srcType;
    ack.destID = GetID();

    ack.command[0] = ackType;
    ack.command[1] = (packet->mac)>>8;
    ack.command[2] = packet->mac;
    ack.command[3] = nackReason;
    if (OP_NACK != ackType)
        ack.commandLen = 3;
    else
        ack.commandLen = 4;
    ack.mac = CalcMAC(&ack, GetID());

    //send the ack
    if (blocking)
    {
        TxOnSpecifiedPort(&ack, TRUE);
    }
    else
    {
        RoutePacket(&ack);
    }
}