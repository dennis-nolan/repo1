
/* VERSION INFO
Version | Common Ver | CM2 Ver | Date       | Released? | Description
0x0001  | 0x0001     | 0x0001  | ----       | Yes       | Before true version tracking
0x0002  | 0x0002     | 0x0002  | ----       | Yes       | Before true version tracking
0x0003  | 0x0003     | 0x0003  | 7/16/2018  | No        | Stopped at alpha, never fully released.
0x0004  | 0x0003     | 0x0003  | 7/23/2018  | No        | Bugfix. Continuing with alpha.
0x0005  | 0x0003     | 0x0007  | 8/07/2018  | No        | Bugfix. Continuing with alpha.
0x0006  | 0x0004     | 0x0009  | 9/07/2018  | No        | Bugfix, warning removal, #def overrides. Continuing with alpha.
0x0007  | 0x0004     | 0x000A  | 9/19/2018  | No        | Avengers updates.
0x0008  | 0x0004     | 0x000C  | 1/04/2019  | No        | PLC frequency addition.
0x0009  | 0x0004     | 0x000C  | 1/25/2019  | No        | CW bugfix.
0x000A  | 0x0004     |         | 1/28/2020  |           | moved in SPIRIT_FREQ_EU_868_3_NORMAL
0x000B  | 0x0004     | 0x0019  | 4/14/2020  |           | changed the radio current vco to newer st default register 0x1a = 0x25
0x000C  | 0x0004     | 0x0019  | 4/27/2020  |           | Store TX power for reinit, added SPIRIT_ERROR_BLANKING
0x000D  | 0x0004     | 0x001C  | 12/16/2020 |           | Persistent RX enabled
0x000E  | 0x0004     | 0x001C  | 05/25/2021 |           | Updates to polling interval, IRQ read, TX->RX transition
*/

#define SPIRIT_VER 0x0000000E
