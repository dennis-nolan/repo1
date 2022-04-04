
/* VERSION INFO
Version |            | Date       |  Description
1 to 2  | ----       | ----       |  Before true version tracking
0x0003  | ----       | 4/4/2019   |  added Dest Type to support added parameter in OP_TEST
        |            |                  pulled OP_TEST_RESULT and OP_PING cm2 transmit functions into module 
0x0004  |            | 4/5/2019   |  Added additoinal support required to track DeviceSource for use w 2 radios
0x0005  |            | 4/8/2019   |  Added a MAC queue to give time for processing of LLACK without requirement 
        |            |                 of the processing before a new OP_PING
        |            |               Put ALL SPIRIT and CM2 tasks on 1MSEC timing for first release
0x0006  |            | 4/10/2019  | added 2 specific tests for RADIO2 on base board for OTA and PLC
0x0007  |            | 5/19/2019  | added power settings for NORMAL mode 
0x0008  |            | 9/13/2019  | added another testState
0x0009  |            | 10/30/2019 | added propulsion fix to not rechecking the area constantly.
*/

#define APPTEST_VER 0x00000009