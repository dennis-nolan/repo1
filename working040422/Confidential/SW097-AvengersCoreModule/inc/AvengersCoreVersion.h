/* VERSION INFO
Version | Date       | Description
0x0001  | 03/15/2019 | Initial creation
0x0002  | 04/23/2019 | Added PLC_TX_POWER
0x0003  | 04/26/2019 | Macro bugfix + Avengers table IDs
0x0004  | 04/26/2019 | Making some PLC functions common
0x0005  | 05/08/2019 | Added nav commands
0x0006  | 06/17/2019 | Added head status table definition
0x0007  | 06/24/2019 | Added OP_ECHO
0x0008  | 06/27/2019 | Updated steering status table, propulsion status and avenger status tables.
0x0009  | 06/27/2019 | Foot pedal status table, misc cleanup.
0x0010  | 07/02/2019 | propulsion status table clean up
0x0011  | 08/02/2019 | OP_NAV_MODE definition, opcodes for OP_ROUTE_POINT_REQ and OP_ROUTE_POINT
0x0012  | 08/16/2019 | typo fixes
0x0013  | 09/09/2019 | base board updates
0x0014  | 09/13/2019 | Added OP_SET_HOME for steering
0x0015  | 09/23/2019 | propulsion update
0x0016  | 09/26/2019 | LINE IN THE SAND status table updates across all boards
0x0017  | 09/28/2019 | added 8 bytes to end of propulsion status table - for use of integrity
0x0018  | 09/30/2019 | added ping results to end of base status table
0x0019  | 10/01/2019 | added TBL_AVENGER_PLC_STATUS
0x0020  | 10/03/2019 | added sequence numbers to PLC tables
0x0021  | 10/09/2019 | added command info to head status table
0x0022  | 10/09/2019 | added command tables
0x0023  | 10/10/2019 | Updated prop command table to mirror OP_PROPULSION
0x0024  | 10/10/2019 | Updated steer command table to include setHome
0x0025  | 10/14/2019 | updated prop command table to add step for increment and decrement commands avoiding race conditions
0x0026  | 10/14/2019 | fixed step to int8 and added sequence byte to TBL_MASTER_CND
0x0027  | 11/05/2019 | Fixed typo in steering status, target angle is signed
0x0028  | 11/06/2019 | Added flag to PLC status table for external steering commands
0x0029  | 11/12/2019 | added OP_COMM OP_NAV_TOGGLE
0x0030  | 11/15/2019 | added "stowed" flag to head and avenger status tables
0x0031  | 11/27/2019 | added DEV_TYPE_STEER_MOTOR for base internal items
0x0032  | 12/18/2019 | added momentary timeout in prop config table
0x0033  | 01/07/2020 | updated Base status table to uint16 ping results
0x0034  | 01/11/2020 | prop config addition and rename of a field
0x0035  | 01/11/2020 | moved DEFAULT values fro prop config
0x0036  | 01/11/2020 | prop config - now have number of steps for forward and reverse separate
0x0037  | 01/23/2020 | prop - added turningOff status bit for on/off control.
                     | added 3 uint16 in avengerstatustable to report missed plc transactions per board
0x0038  | 02/05/2020 | changes to 0xa1 table and added a base config table
0x0039  | 02/17/2020 | Updated head table for Pro Nav support
0x0040  | 02/17/2020 | Updated head table for clarification
0x0041  | 02/18/2020 | added ping test results
0x0042  | 03/09/2020 | added demoMode to base config
0x0043  | 03/16/2020 | added AvengersRealtimeStatusTable and added baseCmdtable for use by foot pedal
0x0044  | 03/16/2020 | added lastHeadMac to end  of plc status table
0x0045  | 03/24/2020 | Increased steering range to +/- 300 degrees
0x0046  | 03/30/2020 | Updates to base-head tables
0x0047  | 04/02/2020 | added setting in base config table for steering rate
0x0048  | 04/03/2020 | added .returnStatus to steering cmd table to keep status repsonses
0x0049  | 04/06/2020 | put in use the 3 spare bytes in propulsion status for some sensorless feedback
        |            |    specifically motorMode and angleDifference
0x0050  | 04/08/2020 | Added model to the head config table
0x0051  | 04/10/2020 | step size made at 70 rpm by adjustment to default max speed to 8400 from 12000 - prop version 0x08
0x0052  | 04/10/2020 | added 2 status bits to AvengerPLCStatusTable for propMotorTurningOff and propMotorRamping
0x0053  | 04/11/2020 | had to add the version of the prop config table for both prop and base to match
0x0054  | 04/15/2020 | added session size to base config
0x0055  | 04/16/2020 | defaults for base ramp and base config version to 0007
0x0056  | 04/21/2020 | added a cmd_beep and changed beep bit to a byte
0x0057  | 05/01/2020 | added model conifg to base config
0x0058  | 05/05/2020 | added base heading and accelerometer data to base status table.
0x0059  | 05/13/2020 | updated prop config defaults and rev'd prop config table version to 10
0x0060  | 05/18/2020 | added speed/time limiting options to steering cmd
0x0061  | 05/28/2020 | added to steering cmd cmd_downloadMotor and downloadMotor
        |            |    added to steering status - downloadingMotor status bit
0x0062  | 06/11/2020 | added to steering status lastMotorDownloadFailure and lastMotorDownloadSuccess
0x0063  | 06/12/2020 | added download chunk offset to steering status
0x0064  | 06/17/2020 | added int to steering status to include chunk total number packets
0x0065  | 06/21/2020 | added status bits to steering status (failureChecksum and failureGetInfo)
0x0066  | 06/24/2020 | Update to head PLC status table 0Xa2 � added speed  mm/s support.
                       Update to prop CMD table 0x42 changed move to step to move to RPM and added saved forward and reverse RPM.
0x0067  | 06/25/2020 | Added OP_PROD_CAL
0x0068  | 07/01/2020 | changed the saved forward and reverese step on prop status to RPM . required int8 to int16 changes for both entries
0x0069  | 07/01/2020 | added 2 bits to OP_COMM/OP_PROPULSION
0x0070  | 07/07/2020 | removed changes in version 00069. updated base_cmd_table
                     | cleaned up prop status table (removed the forward/reverse saving of steps - removed step)
                     | renamed 2 fields in prop cmd table that are not being used to reserved.
                     | added OP_RESERVE, OP_SPEED_MODE
0x0071  | 07/13/2020 | added ramping bit to propulsion status table
0x0072  | 07/14/2020 | added beep tones
0x0073  | 07/16/2020 | added firmware versions to commands and table
0x0074  | 07/21/2020 | Updated beep tunes
0x0075  | 07/27/2020 | added in display ann table numbers, simple remote table numbers and
                     |   struct for simple remote status table
0x0076  | 09/01/2020 | Added TBL_AVENGER_MAG_CAL to read/write the magnetometer calibration
0x0077  | 09/15/2020 | Updated TBL_AVENGER_MAG_CAL to include version and flags
0x0078  | 09/17/2020 | Added fault flags to steering status
0x0079  | 09/22/2020 | Added a fault flags for no steering cal
0x0080	| 10/20/2020 | Added Display Annunciator Status Table. Updated Foot Pedal Status Table
0x0081	| 10/20/2020 | Remove hBridgeTemperature from Pedal status table.
0x0082  | 10/20/2020 | added diagnostic base download table
                     | added system current to table 0xa4
0x0083  | 10/29/2020 | Propulsion status table (0x41) updated to include status denoting OCL auto reset occurance
0x0084	| 11/16/2020 | cmd_steeringTimeout added to BaseCmdTable
========= to this point - matches the CM2 SPEC version 00.124
0x0085	| 11/20/2020 | Added externalSteerCmd to the steering cmd table
0x0086  | 01/18/2020 | Added "cmd_groundCase" + "groundCase" config bits to "PropCmdTable"
0x0087  | 01/29/2021 | added state of charge, battery temp to table 0xd0 
0x0088  | 03/18/2021 | Error reporting overhaul, line in the sand
0x0089  | 06/07/2021 | merged into MASTER for move to 00_39 versions. 
0x0090  | 06/10/2021 | Pairing mode flag and new tunes
0x0091  | 06/14/2021 | moved grounding config to prop config and added clearVoltageFault to prop cmd
*/

#define AVENGERS_VER 0x00000091
