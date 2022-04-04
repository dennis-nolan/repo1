
/* VERSION INFO
Version | Common Ver | Date       | Released? | Description
1 to 2  | ----       | ----       | Yes       | Before true version tracking
3 to 8  | ----       | ----       | No        | Stopped at alpha, never fully released.
0x0009  | 0x0004     | 9/07/2018  | No        | #def overrides, warning removal. Continuing with alpha.
0x000B    0x0004     | 10/15/2018   no        | Added high level ACK/NACK to firmware downloads 
                                              | added some NACK codes to CM2CoreTypesHeader.h 
                                              | added that high ACK can replace LLACK in retry area 
0x000D  | 0x0004     | 11/06/2018 | no        | reworked LLAck in fimrware downloads in OP_EX_PACKET_DATA needed 
                                              | to be blocking. Added high level ACK before Reset device. Added 
                                              | high level ACK to matching packets.
0x000E  | 0x0004     | 02/05/2019 | no        | Resolved bug where packets that fail to route cause endless acks.
                                              | Moved challenge/response into UnpairedRxCallback.
0x000F  | 0x0004     | 02/20/2019 | no        | Resolved bug where srcType of challenge is 0xFF.
0x0010  | 0x0004     | 03/08/2019 | no        | Merge in alphaPLC branch.
0x0011  | 0x0004     | 04/03/2019 | no        | Handle invalid port better, use duplicateFiltering flag.
0x0012  | 0x0004     | 04/22/2019 | no        | Fix device type filtering
0x0013  | 0x0004     | 05/02/2019 | no        | FW update changes for Avengers
0x0014  | 0x0004     | 05/13/2019 | no        | changed OP_ACK in fw downloads to NON-BLOCKING (requires a delay in ResetDevice function for last OP_ACK to be sent
0x0015  | 0x0004     | 05/22/2019 | no        | changed retry count in ACK_TIMEOUT FROM 30msec to 60msec
0x0016  | 0x0004     | 05/28/2019 | no        | with an OP_ACK rx WILL negate the need for LL_ACK on firmware udpate packets
0x0017  | 0x0004     | 06/04/2019 | no        | added FW_SESSION_INFO to FW downloads to add adjustments to timeouts
0x0018  | 0x0004     | 11/08/2019 | no        | added OP_PAIR_COMPLETE
0x0019  | 0x0004     | 11/12/2019 | no        | added some used device types
0x001A  | 0x0004     | 04/14/2020 | no        | removed subaddress from AbortSimilar, cleaned up some compiler warnings, added some nack codes
0x001B  | 0x0004     | 04/14/2020 | no        | removed LLACK being sent as high level is going out.  
0x001C  | 0x0004     | 07/07/2020 | no        |  added OP_OPEN_PAIR def
                                              |  #define NACK_BADTABLEVERSION def added 
                                              |  #define NACK_UNDEFINED def added 
                                              |  #define NACK_STOP_DOWNLOAD def and will force download to a FAIL STATUS to stop it
                                              |  #define SRC_DISPLAY_TYPE def added   
0x001D  | 0x0004    | 02/11/2021  | no        | added CheckDevice() call to determine if a download was successful. 
                                              | added a nack code if there is an error in a download  NACK_DONWLOADFAIL                 
*/

#define CM2CORE_VER 0x0000001D