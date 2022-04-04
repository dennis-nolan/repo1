
//******************************************************************************
// AVENGERS COMMUNICATIONS
// -----------------------------------------------------------------------------
// As a general rule, status tables will be sent out:
// 1. maximum refresh rate is once per second.
// 2. minimum refresh rate is 200msec (even on change thresholds, the table will
//    NOT be sent out any sooner than the last tranmission of 200msec).
//------------------------------------------------------------------------------
// Propulsion and steering status table will be sent as DIRECT to BOTH the head
// and base board.
//------------------------------------------------------------------------------
// For each of the status tables, the SENT ON CHANGE entries will be marked below.

#ifndef AVENGERS_CORE_HEADER
#define AVENGERS_CORE_HEADER

#ifndef AVENGER_MACROS_ONLY
#include "CM2CoreHeader.h"
#endif

#define MODEL_SCIZZOR   0
#define MODEL_PIVOT     1

/********** Product limits **********/
#define ABSOLUTE_ROTATION_MAX             300
#define ABSOLUTE_ROTATION_MIN             (0-ABSOLUTE_ROTATION_MAX)
#define RELATIVE_ROTATION_MAX             (2*ABSOLUTE_ROTATION_MAX)
#define RELATIVE_ROTATION_MIN             (2*ABSOLUTE_ROTATION_MIN)


/********** Angle conversion macros **********/
// Number of integer counts per 1 degree
#define COUNTS_PER_DEGREE                 10

/********** Defaults **********/
#define MIN_STATUS_REFRESH          200 //mS
#define MAX_STATUS_REFRESH          1000 //mS
    //---------------------------------
    // PLC power is
    // (-34dB TO 11.6 dB)
    // Value is added to -34dB with max being 0x2E Which is 11.6dB
    //------------------------------------
#define DEFAULT_PLC_PWR_PROP        0x2E // 46 - 34 = 12 = 11.6db
#define DEFAULT_PLC_PWR_STEER       0x18 // 24 - 34 = -10db
#define DEFAULT_PLC_PWR_HEAD        0x18 // 24 - 34 = -10db
#define DEFAULT_PLC_PWR_BASE        0x2E // 46 - 34 = 12 = 11.6db

#define DEFAULT_RETRIES_HIGH        8
#define DEFAULT_RETRIES_NORM        4
#define DEFAULT_RETRIES_LOW         1
#define	DEFAULT_HEAD_ANGLE_THRESH   5 //deg
#define	DEFAULT_STEER_ANGLE_THRESH  5 //deg

#define DEFUALT_PLC_CARRIER_MODE    0
#define DEFAULT_PLC_RXCARRIER_THRESHOLD 0x24

/********** Internal PLC device types **********/
enum
{
    DEV_TYPE_BASE  = 0x01,
    DEV_TYPE_HEAD  = 0x02,
    DEV_TYPE_STEER = 0x03,
    DEV_TYPE_PROP  = 0x04,
    DEV_TYPE_FOOT  = 0x05,
    DEV_TYPE_STEER_MOTOR = 0x06  //ONLY used INTERNALLY to the BASE BOARD
};

typedef enum
{
    BOOT_TUNE = 0,
    BASIC_BEEP,
    UP_TUNE,
    DOWN_TUNE,
    ON_TUNE,
    OFF_TUNE,
    ERROR_TUNE,
    NAV_HL_ON_TUNE,
    NAV_AL_ON_TUNE,
    NAV_VL_ON_TUNE,
    NAV_OFF_TUNE,
    PAIR_MODE_TUNE,
    PAIR_SUCCESS_TUNE,
    OCL_TUNE,
    DEPLOY_TUNE,
    DOWNLOAD_TUNE,
    INSTALL_TUNE,
    QUICK_TICK_TUNE,
    REV_ON_TUNE,
    REV_OFF_TUNE,

    //endpoint of normal tune numbers
    NUM_TUNES
} BeepSounds;

/********** Subopcodes used with OP_COMM **********/
typedef enum
{
    OP_DODA = 0x1B,
    OP_DODR = 0x1C,
    OP_STOP = 0x1D,
    OP_HOLD = 0x1E,
    OP_SPEED_MODE = 0x2D,
    OP_RELEASE = 0x2E,
    OP_PROPULSION = 0x2F,
    OP_ANCHOR_LOCK = 0x30,
    OP_REVERSE = 0x31,
    OP_NAV_MODE = 0x32,
    OP_JOG = 0x33,
    OP_ROUTE_POINT_REQ = 0x34,
    OP_ROUTE_POINT = 0x35,
    OP_NAV_TOGGLE = 0x37,
    OP_CALIBRATE = 0x20,
    OP_ECHO = 0x56,
    OP_BASEBOARD = 0x57,
    OP_SET_HOME = 0x36,
    OP_TM_ID = 0x10,
    OP_PROD_CAL = 0x60,
    OP_BRIGHTNESS = 0x61
} AvengerSubopcode;


/********** Nav Modes **********/
typedef enum
{
    NAV_OFF = 0,
    NAV_MANUAL,
    NAV_ANCHOR_LOCK,
    NAV_GPS_VECTOR,
    NAV_HEADING_LOCK,
    NAV_ROUTE,
    //reserved 5-199
    NAV_TOGGLE = 200,
    NAV_DECREMENT,
    NAV_INCREMENT,
    //reserved 203-255
} NavCommand;

/********** Avengers Table IDs **********/
//Note: the PLC device types are used to organize these tables
enum
{
    //Common CM2 Tables
    TBL_PAIRING = 0,
    TBL_CM2_PROD_INFO,
    //RESERVED

    //Base Tables
    TBL_PWR_STATUS = 0x10,
    TBL_BASE_IMG_VERS,
    TBL_BASE_RUNTIME,
    TBL_BASE_STATUS,
    TBL_BASE_CONFIG,
    TBL_BASE_CMD,
    TBL_BASE_DOWNLOAD,
    //RESERVED

    //Head Tables
    TBL_NAVIGATION = 0x20,
    TBL_SENSOR_DATA,
    TBL_SENSOR_RAW_DATA,
    TBL_SENSOR_CAL,
    TBL_HEAD_STATUS,
    TBL_HEAD_CMD,
    TBL_HEAD_CONFIG,
    //RESERVED

    //Steering Tables
    TBL_STEERING_CTRL = 0x30,
    TBL_STEERING_STATUS,
    TBL_STEER_CMD,
    TBL_STEER_CONFIG,
    //RESERVED

    //Propulsion Tables
    TBL_PROP_CONFIG = 0x40,
    TBL_PROP_STATUS,
    TBL_PROP_CMD,
    //RESERVED

    //Avenger Product Tables
    TBL_AVENGER_FW_VERS = 0xA0,
    TBL_AVENGER_STATUS,
    TBL_AVENGER_PLC_STATUS,
    TBL_AVENGER_CONFIG,
    TBL_AVENGER_REALTIME_STATUS,
    TBL_AVENGER_MAG_CAL,
    //RESERVED

    //Foot Pedal Tables
    TBL_FOOT_PEDAL_CONFIG = 0xB0,
    TBL_FOOT_PEDAL_STATUS,
    //RESERVED

    //Display annunciator tables
    TBL_DISPLAY_ANN_CONFIG = 0xC0,
    TBL_DISPLAY_ANN_STATUS,
    //RESERVED

    //avenger remote Tables
    TBL_SIMPLE_REMOTE_STATUS = 0xD0,
    TBL_SIMPLE_REMOTE_STATE,
    TBL_SIMPLE_REMOTE_CONFIG,
    //RESERVED
};

#ifndef AVENGER_MACROS_ONLY

/********** Commonly Used Avengers Tables **********/
#ifndef __packed
#define __packed __attribute__ ((packed))
#endif

//----------------------------------------
// TBL_AVENGER_CONFIG TABLES
// these tables are saved by base board and
// sent out to sub-units during initialization
//---------------------------------
#define AVENGERSCORE_BASE_CONFIG_VER	8
#define DEFAULT_REL_MOVE_SINGLE       	5
#define DEFAULT_REL_MOVE_HELD         	20  //power of 10 change
#define DEFAULT_STEERING_RATE         	1
#define DEFAULT_REL_MOVE_RELEASE_HELD   5
#define DEFAULT_REL_MOVE_RELEASE_SINGLE 2
#define DEFAULT_PING_SIZE               50
#define DEFAULT_SESSION_SIZE            128
#define DEFAULT_RAMP1                   2
#define DEFAULT_RAMP2                   4
#define DEFAULT_RAMP3                   6
#define DEFAULT_RAMP4                   8
#define DEFAULT_MODEL                   MODEL_SCIZZOR

typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint8_t     plcPowerBase;
    uint8_t     plcCarrierMode;
    uint8_t     plcRXCarrierThreshold;
    uint8_t     relativeMoveSingleSteering;
    uint8_t     relativeMoveHeldSteering;
    uint8_t     relativeMoveReleaseHeldSteering;
    uint8_t     relativeModeReleaseSingleSteering;
    uint16_t    pingTestSize;
    uint8_t     demoMode;
    uint8_t     steeringRate;  //0=50 msec nonzero = 100msec
    uint16_t    sessionSize;
    uint8_t     ramp1;
    uint8_t     ramp2;
    uint8_t     ramp3;
    uint8_t     ramp4;
    uint8_t     model;
    uint8_t     reserved[2];
} __packed AvengerConfigBaseTable;


#define AVENGERSCORE_PROP_CONFIG_VER	11
#define DEFAULT_MOMENTARYTO             50
#define DEFAULT_MAX_APPLICATION_SPEED   8400
#define DEFAULT_RAMP_STEP_PREELBOW      1944
#define DEFAULT_RAMP_STEP_POSTELBOW     10000
#define DEFAULT_NUMBEROFSTEPS_FORWARD   20
#define DEFAULT_NUMBEROFSTEPS_REVERSE   20
#define DEFAULT_SPEED_ELBOW             300
#define DEFAULT_PROPULSION_VOLTAGE_UNDER_LIMIT 18
#define DEFAULT_PROPULSION_VOLTAGE_OVER_LIMIT   60
#define DEFAULT_PROP_STOWSTEPFORWARD    5
#define DEFAULT_PROP_STOWSTEPREVERSE    10
#define DEFAULT_PROP_MINFORWARD         1
#define DEFAULT_PROP_MINREVERSE         1
#define DEFAULT_GROUNDING_OFF           0

typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint8_t     plcPowerProp;
    uint8_t     plcCarrierMode;
    uint8_t     plcRXCarrierThreshold;
    uint8_t     stowStepForwardDefault; //this is the off to on default for stow-deploy
    uint8_t     stowStepReverseDefault; //this is the off to on default for stow-deploy
    uint16_t    speedElbow;
    uint16_t    rampStepPreElbow;
    uint16_t    rampStepPostElbow;
    uint16_t    piLowSetPoint;
    uint16_t    maxApplicationSpeed;
    uint8_t     numSpeedStepsForward;
    uint16_t    stepSize;
    uint8_t     voltageUnderLimit;
    uint8_t     voltageOverLimit;
    uint8_t     momentaryTimeOut;  //in 10msec *
    uint8_t     minStepForward;
    uint8_t     minStepReverse;
    uint8_t     numSpeedStepsReverse;
    uint8_t     configBitGroundingOn:1;  //if 1, then ground the case on
    uint8_t     configBitsReserved:7;
    uint8_t     reserved[5];
} __packed AvengerConfigPropTable;

typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint8_t     plcPowerHead;
    uint8_t     plcCarrierMode;
    uint8_t     plcRXCarrierThreshold;
    uint8_t     model;
    uint8_t     reserved[11];
} __packed AvengerConfigHeadTable;

typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint8_t     plcPowerSteer;
    uint8_t     plcCarrierMode;
    uint8_t     plcRXCarrierThreshold;
    uint8_t     reserved[12];
} __packed AvengerConfigSteerTable;

//--------------------------------
// TBL_AVENGER_STATUS
// this table is complied by the base board and sent out
// over OTA radio only.
//---------------------------------
typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint16_t    systemVoltage;
    uint16_t    systemCurrent;
    uint8_t     systemMode;
    uint8_t     headNavMode;
    int16_t     headHeading;
    float       headLatitude;
    float       headLongitude;
    uint32_t    headGPSaccuracy;
    int32_t     headGPSspeed;
    uint16_t    headGPSheading;
    uint16_t    headHDOP;
    uint8_t     headNumSatellites;
    int16_t     steeringTargetAngle;
    int16_t     steeringCurrentAngle;
    uint16_t    steeringTemp;
    uint8_t     steeringLastCmd;
    int8_t      propulsionStep;  //EVENT
    uint8_t     propulsionMotorOn:1; //EVENT
    uint8_t     steeringReleased:1; //EVENT
    uint8_t     stowed:1;
    uint8_t     reserved:5;
    uint16_t    sessionSize;
    uint8_t    sessionHealthHeadCommands;
    uint8_t    sessionHealthSteerCommands;
    uint8_t    sessionHealthPropCommands;
} __packed AvengerStatusTable;

//common error flags structure
typedef struct {
    //-------byte 1
    uint8_t     errorHeadCheckin:1;     //base internal
    uint8_t     errorPropCheckin:1;     //base internal
    uint8_t     errorSteerCheckin:1;    //base internal
    uint8_t     errorPicCheckin:1;
    uint8_t     errorDisplayCheckin:1;
    uint8_t     errorPedalCheckin:1;
    uint8_t     errorHeadHealth:1;      //base internal
    uint8_t     errorPropHealth:1;      //base internal
    //-------byte 2
    uint8_t     errorSteerHealth:1;     //base internal
    uint8_t     errorPicHealth:1;
    uint8_t     errorDisplayHealth:1;
    uint8_t     errorHeadPing:1;        //base internal
    uint8_t     errorPropPing:1;        //base internal
    uint8_t     errorSteerPing:1;       //base internal
    uint8_t     errorDisplayPing:1;
    uint8_t     errorSteerNoCal:1;      //SteeringStatusTable.nocal
    //-------byte 3
    uint8_t     errorHeadNoCal:1;
    uint8_t     errorBaseNoCal:1;
    uint8_t     errorSteerPoorCal:1;    //SteeringStatusTable.badcal
    uint8_t     errorHeadPoorCal:1;
    uint8_t     errorBasePoorCal:1;
    uint8_t     errorShaftSlip:1;
    uint8_t     errorPropStart:1;
    uint8_t     errorSteerNoCurrent:1;
    //-------byte 4
    uint8_t     errorSteerOvercurrent:1;//SteeringStatusTable.overcurren
    uint8_t     errorSteerStall:1;      //SteeringStatusTable.stall
    uint8_t     errorSteerOvertemp:1;   //SteeringStatusTable.overtemp
    uint8_t     errorAnchorModeStart:1;
    uint8_t     errorHeadingModeStart:1;
    uint8_t     errorRouteModeStart:1;
    uint8_t     errorVbatOperational:1; //base internal vBatt<15 but in Active state
    uint8_t     errorVbatPowerup:1;
    //------ byte 5
    uint8_t     errorVbatShutdown:1;    //base internal vBatt < 10V
    uint8_t     OCLAutoResetOccurred:1; //PropulsionStatusTable.OCLAutoResetOccured
    uint8_t     ssRelayOff:1;
    uint8_t     v17Low:1;
    uint8_t     errorElectrolysis:1;    //PropulsionStatusTable.FaultElectrolysis
    uint8_t     reservedBits:3;
    uint8_t     reserved[3];
} __packed ErrorFlags; //8 bytes

//--------------------------------
// TBL_REALTIME_AVENGER_STATUS
// this table is compiled by base board and sent OTA
// this table will be sent as a acknowledgment to footpedal commands
//---------------------------------
typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint8_t     systemMode;
    uint8_t     headNavMode;
    int16_t     headHeading;
    float       headLatitude;
    float       headLongitude;
    uint16_t    headHDOP;
    int16_t     steeringTargetAngle;
    int16_t     steeringCurrentAngle;
    int8_t      propulsionStep;
    uint8_t     propulsionMotorOn:1;
    uint8_t     steeringReleased:1;
    uint8_t     stowed:1;
    uint8_t     reservedBits:5;
    int16_t     propulsionRPMMeasured;
    uint16_t    systemCurrent;
    uint8_t     headGPSConfidence;
    ErrorFlags  errors;
    uint8_t     reserved[4];
} __packed AvengerRealtimeStatusTable;

//TBL_BASE_CMD
typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint8_t     sequence;
    uint8_t     txHoldoff; //1-254ms of holdoff, 0  for immediate and 0xFF for normal TX operation
    uint8_t     cmd_motorOn:1;                     //used with motorOn to set the prop motor on/off state. If motorOn is a 0, prop motor is turned off
    uint8_t     cmd_targetAngle:1;
    uint8_t     cmd_targetRPM:1;
    uint8_t     cmd_limitSpeed:1;
    uint8_t     cmd_minimumTime:1;
    uint8_t 	cmd_steeringTimeout:1;
    uint8_t     cmd_reserved:2;
    uint8_t     motorOn:1;
    uint8_t     reserved:7;
    int16_t     targetAngle; //always absolute
    int16_t     targetRPM;   //always absolute
    uint16_t    speedLimit;
    uint16_t    minTime;
    uint32_t    firmwareVersion;
} __packed BaseCmdTable;





//TBL_PROP_CMD
//note: does not include tblVer, tblFlags, or sequence
typedef struct {
    uint8_t     cmd_systemMode:1;
    uint8_t     cmd_action:1;
    uint8_t     cmd_notUsed:1;
    uint8_t     cmd_stowed:1;
    uint8_t     cmd_goToRPM:1;
    uint8_t     cmd_clearVoltageFault:1;
    uint8_t     cmd_reserved:2;
    uint8_t     systemMode;
    uint8_t     action;     //0-200 (0-100%), 201=toggle, 202=dec, 203=inc
    uint8_t     notUsed:1;
    uint8_t     stow:1;     //set to "1" when stowed
    uint8_t     clearVoltageFault:1;   // set to "1"  
    uint8_t     reserved:5; //unused flags
    int16_t     goToRPM;
    int16_t     reserved1;
    int16_t     reserved2;
} __packed PropCmdTable;

//TBL_STEER_CMD
//note: does not include tblVer, tblFlags, or sequence
typedef struct {
    uint8_t     cmd_systemMode:1;
    uint8_t     cmd_targetAngle:1;
    uint8_t     cmd_released:1;
    uint8_t     cmd_setHome:1;
    uint8_t     cmd_stowed:1;
    uint8_t     cmd_limitSpeed:1;
    uint8_t     cmd_minimumTime:1;
    uint8_t     cmd_downloadMotor:1;
    uint8_t     systemMode;
    int16_t     targetAngle;
    int16_t     setHome;
    uint8_t     targetRelative:1;
    uint8_t     released:1;
    uint8_t     setHomeRelative:1;
    uint8_t     stow:1;             //set to "1" when stowed
    uint8_t     returnStatus:1;     //if 1- steering will not respond with status
    uint8_t     downloadMotor:1;    //if 1-starts a download to steering motor if versions differ
    uint8_t     externalSteerCmd:1; //0 if from head, 1 if from an external device
    uint8_t     reserved:1;
    uint16_t    speedLimit;
    uint16_t    minTime;
} __packed SteerCmdTable;

//TBL_HEAD_CMD
//note: does not include tblVer, tblFlags, or sequence
typedef struct {
    uint8_t     cmd_systemMode:1;
    uint8_t     cmd_navMode:1;
    uint8_t     cmd_latitude:1;
    uint8_t     cmd_longitude:1;
    uint8_t     cmd_heading:1;
    uint8_t     cmd_beep:1;
    uint8_t     cmd_speed:1;
    uint8_t     cmd_reserved:1;
    uint8_t     systemMode;
    uint8_t     navMode;
    uint8_t     beepMode;
    int16_t     speed;
    float       latitude;
    float       longitude;
    uint16_t    heading;
} __packed HeadCmdTable;

//--------------------------------
// TBL_AVENGER_PLC_STATUS
// this table is compiled  by the base board and sent out
// over PLC.
//---------------------------------
typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint8_t     sequence;
    int16_t     propSystemMode;
    int16_t     propSpeedRPM;
    int16_t     steerSystemMode;
    int16_t     steerCurrentAngle;
    int16_t     steerTargetAngle;
    uint8_t     steerReleased:1;
    uint8_t     propMotorOn:1;
    uint8_t     externalSteerCmd:1;
    uint8_t     propMotorTurningOff:1;
    uint8_t     propMotorRamping:1;
    uint8_t     basePairingMode:1;
    uint8_t     reservedBits:2;
    HeadCmdTable headCommands;
    uint16_t    headLastMac;
    uint16_t    baseHeading;
    float       baseAccel[3];
    ErrorFlags  errors;
    uint8_t     reserved[4];
} __packed AvengerPLCStatusTable;


//TBL_PROP_STATUS
typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint8_t     TimingAdjustMSB;
    uint8_t     TimingAdjustLSB;
    int16_t     MotorTemp;
    uint16_t    BusVoltage;
    int16_t     RPMSetting;
    int16_t     RPMMeasured;
    //uint8_t     OCLReading;
	uint8_t     OCLReading:1;			/*!< OCL pin status */
	uint8_t		OCLAutoResetOccured:1;	/*!< OCL reset occured within last 5 seconds */
	uint8_t		OCLReserved:6;
    uint8_t     MotorLibState;
    uint8_t     MotorOn:1;  //EVENT
    uint8_t     avengerConfigTableGood:1;
    uint8_t     reverse:1;
    uint8_t     stow:1;
    uint8_t     turningOff:1;
    uint8_t     ramping:1;
    uint8_t     reserved:2;
    uint8_t     OccurredFaultsMSB;  //EVENT
    uint8_t     OccurredFaultsLSB;  //EVENT
    uint8_t     CurrentFaultsMSB;   //EVENT
    uint8_t     CurrentFaultsLSB;   //EVENT
    uint8_t     PropulsionBoardStatus;
    uint8_t     VoltageOverLimitCount;
    uint8_t     CurrentOverLimitCount;
    uint8_t     FaultStateUnderVoltage:1;   //EVENT
    uint8_t     FaultStateOverVoltage:1;    //EVENT
    uint8_t     FaultStateOverCurrentLimit:1;   //EVENT
    uint8_t     FaultStatePreOverTemp:1;    //EVENT
    uint8_t     FaultStateOverTemp:1;      //EVENT
    uint8_t     FaultElectrolysis:1;
    uint8_t     FaultReserved:2;
    uint8_t     SystemMode;              //EVENT
    uint8_t     motorStartFailureCount;
    uint8_t     motorStopFailureCount;
    uint8_t     lastSequence;
    uint8_t     motorMode;
    uint16_t    angleDifference;
} __packed PropulsionStatusTable;



//TBL_HEAD_STATUS
typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint16_t    boardVoltage;
    uint8_t     systemMode;
    uint8_t     navMode;
    uint8_t     headingConfidence;
    int16_t     heading;
    uint8_t     GPSConfidence;
    float       latitude;
    float       longitude;
    uint32_t    GPSaccuracy;
    int32_t     GPSspeed;
    uint16_t    GPSheading;
    uint16_t    HDOP;
    uint8_t     numSatellites;
    uint16_t    year;
    uint8_t     month;
    uint8_t     day;
    uint8_t     hour;
    uint8_t     minute;
    uint8_t     second;
    uint8_t     avengerConfigTableGood:1;
    uint8_t     stowed:1;
    uint8_t     atCoord:1;
    uint8_t     errorAnchorModeStart:1;
    uint8_t     errorHeadingModeStart:1;
    uint8_t     errorRouteModeStart:1;
    uint8_t     errorHeadNoCal:1;
    uint8_t     errorHeadPoorCal:1;
    uint8_t     lastSequence;

    //command items
    PropCmdTable propCommands;
    SteerCmdTable steerCommands;

    uint8_t     errorShaftSlip:1;
    uint8_t     reservedBits:7;
    uint8_t     reserved[2];
} __packed HeadStatusTable;

//TBL_STEERING_STATUS
typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint16_t    boardVoltage;
    uint8_t     systemMode;
    uint16_t    motorFwVerMajor;
    uint16_t    motorFwVerMinor;
    uint16_t    hwRevision;
    int16_t     current;
    int16_t     currentAngle;
    int16_t     targetAngle;
    int16_t     temperature;
    uint8_t     lastCmdDevice;
    uint8_t     steeringReleased:1;
    uint8_t     avengerConfigTableGood:1;
    uint8_t     downloadingMotor:1;
    uint8_t     lastMotorDownloadFailure:1;
    uint8_t     lastMotorDownloadSuccess:1;
    uint8_t     lastMotorDownloadFailureChecksum:1;
    uint8_t     lastMotorDownloadFailureGetInfo:1;
    uint8_t     reservedBits:1;
    uint8_t     lastSequence;
    uint32_t    storedMotorFWVersion;
    uint16_t    downloadChunkOffset;
    uint16_t    downloadChunkTotal;
    uint16_t    overtemp:1;
    uint16_t    overcurrent:1;
    uint16_t    nocurrent:1;
    uint16_t    undervolt:1;
    uint16_t    badcal:1;
    uint16_t    stall:1;
    uint16_t    nocal:1;
    uint16_t    reservedFaults:9;
} __packed SteeringStatusTable;

//TBL_FOOT_PEDAL_STATUS
typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint16_t    boardVoltage;
    uint8_t     mode;
    int16_t     angle;
    uint8_t     input_values;
      //----------------------
      // bit 0: prop button pressed
      // bit 1: hall sw triggered
      //-----------------------------
    uint8_t     health_byte;
      //----------------------
      // bit 0: eeprom good
      // bit 1: crypto good
      // bit 2: pedal encoder good
      // bit 3: pedal magnet detected
      // bit 4: pedal calibrated
      // bit 5: roller encoder good
      // bit 6: roller magnet detected
      // bit 7: Spirit OTA Init good
      //-----------------------------
    int8_t      converterTemperature;
    int16_t     hBridgeCurrent;
    uint8_t     masterMode:1;      //EVENT
    uint8_t     reserved:7;
} __packed FootPedalStatusTable;

//TBL_BASE_STATUS
typedef struct {
    uint8_t  tblVer;
    uint8_t  tblFlags;
    uint8_t  checkinFaults;
    uint8_t  checkinIDRequest;
    uint16_t checkinVersion;
    uint16_t readingCalculatedVbs;
    uint16_t readingCalculatedVb;
    uint16_t readingCalculatedTemp;
    int16_t readingCalculatedCurrent;
    uint8_t  currentMode;
      //----------------------
      // bit 0: eeprom good
      // bit 1: serial flash good
      // bit 2: mag good
      // bit 3: accelerometer good
      // bit 4: hall sw input value
      // bit 5: crypto good
      // bit 6: Spirit OTA Init good
      // bit 7: Spirit PLC Init good
      //-----------------------------
    uint8_t  boardStatusLSB;
    uint8_t  boardStatusMSB;
    uint8_t  appState;         //EVENT
    uint16_t  pingResultHead;
    uint16_t  pingResultSteering;
    uint16_t  pingResultPropulsion;
    uint16_t  pingTestSize;
    uint16_t    baseHeading;
    float       baseAccel[3];
} __packed BaseStatusTable;

//TBL_BASE_DOWNLOAD
typedef struct {
    uint8_t  tblVer;
    uint8_t  tblFlags;
    uint8_t  appDownloadNeeded;
    uint8_t  appDownloadRequested;
    uint8_t  spare[3];
} __packed BaseDownloadTable;

typedef struct {
    uint8_t tblVer;
    uint8_t tblFlags;
    uint8_t mode;
    uint8_t light_sensor;       /* 0-255 percent of raw adc value (0-4095)*/
    uint8_t master_mode:1;
    uint8_t i2c_error:1;
    uint8_t reserved:6;
} __packed DisplayStatusTable;

typedef struct {
    uint8_t     tblVer;
    uint8_t     tblFlags;
    uint8_t     sequenceByte;
    uint8_t     buttonState0;
    uint8_t     buttonState1;
    uint8_t     buttonState2;
    uint8_t     buttonState3;
    int8_t      batteryTemp;      //int8_t value denoting degrees Celsius
    uint16_t    button0HoldTime;
    uint16_t    button1HoldTime;
    uint16_t    button2HoldTime;
    uint16_t    button3HoldTime;
        //---------state of charge
        //0 == Charge state undefined (initial state until battery is sampled)
        //1 == BATTERY_LOW
        //2 == BATTERY_MED
        //3 == BATTERY_FULL
    uint8_t    stateOfCharge;
    uint8_t    reserved;
    uint32_t   firmwareVersionSimpleRemote;
} __packed SimpleRemoteStatusTable;

//TBL_AVENGER_MAG_CAL
typedef struct {
    uint8_t tblVer;
    uint8_t tblFlags;
    float   HI_Bias[3];   //default 0.0
    float   SF_Matrix[3]; //default 1.0
    bool    calibrated;    //default false
} __packed MagCalibrationTable;



/********** Commonly Used Avengers Packets **********/

//OP_COMM + OP_DODR/DODA
typedef struct {
    uint8_t     opcode;     //OP_COMM
    uint8_t     subOpcode;  //OP_DODR/DODA
    uint8_t     sequence;   //sequence number
    uint8_t     speed;      //0-200 (0-100%)
    uint8_t     torque;     //0-200 (0-100%)
    int16_t     angle;      //-3276.8 to 3276.7 degrees (0.1 degree increments)
} __packed DODR_DODA_Payload;

//OP_COMM + OP_REVERSE
typedef struct {
    uint8_t     opcode;     //OP_COMM
    uint8_t     subOpcode;  //OP_PROPULSION
    uint8_t     sequence;   //sequence number
    uint8_t     action;     //0 = no change
                            //1 = toggle current setting
                            //2 = turn on reverse
                            //3 = turn off reverse
                            //4:255 not used.
    uint32_t    firmwareVersion;
} __packed Reverse_Payload;

//OP_COMM + OP_SPEED_MODE
typedef struct {
    uint8_t     opcode;     //OP_COMM
    uint8_t     subOpcode;  //OP_PROPULSION
    uint8_t     sequence;   //sequence number
    uint8_t     action;     //0 = no change
                            //1 = toggle speed mode
                            //2 = speed mode moved to mph
                            //3 = speed mode moved to kph
                            //4 = speed mode moved to step
                            //5:255 not used.
    uint32_t    firmwareVersion;
} __packed SpeedMode_Payload;

//OP_COMM + OP_PROPULSION
typedef struct {
    uint8_t     opcode;     //OP_COMM
    uint8_t     subOpcode;  //OP_PROPULSION
    uint8_t     sequence;   //sequence number
    uint8_t     action;     //0-200 (0-100%), 201=toggle, 202=dec, 203=inc
    uint32_t    firmwareVersion;
} __packed Propulsion_Payload;

//OP_COMM + OP_NAV_MODE
typedef struct {
    uint8_t     opcode;     //OP_COMM
    uint8_t     subOpcode;  //OP_NAV_MODE
    uint8_t     sequence;   //sequence number
    uint8_t     action;     //mode set, toggle, increment, decrement
    uint8_t     speed;      //0-200 (0-100%), sets max speed for current mode
    float       latitude;   //Used for anchor lock. 0 if anchoring at current position.
    float       longitude;  //Used for anchor lock. 0 if anchoring at current position.
    uint16_t    heading_numpoints;  //heading except for NAV_ROUTE where it is the num points. 0xFFFF if using the current heading.
} __packed Nav_Mode_Payload;

//OP_COMM + OP_SET_HOME
typedef struct {
    uint8_t     opcode;     //OP_COMM
    uint8_t     subOpcode;  //OP_SET_HOME
    uint8_t     sequence;   //sequence number
    int16_t     angle;      //-3276.8 to 3276.7 degrees (0.1 degree increments)
    uint8_t     absolute:1; //absolute/relative flag
    uint8_t     reserved:7; //unused flags
} __packed Set_Home_Payload;

//OP_COMM + OP_TM_ID
typedef struct {
    uint8_t     opcode;     //OP_COMM
    uint8_t     subOpcode;  //OP_ID_REQ
    uint8_t     sequence;   //sequence number
    bool        request;    //is this a request, or providing the ID?
    uint8_t     myUID[12];    //STM32 unique ID of requester
} __packed TM_ID_Payload;

//OP_COMM + OP_NAV_TOGGLE
typedef struct {
    uint8_t     opcode;     //OP_COMM
    uint8_t     subOpcode;  //OP_ID_REQ
    uint8_t     sequence;   //sequence number
    uint8_t     mode;    //nav mode
    uint32_t    firmwareVersion;
} __packed NAV_Toggle_Payload;

//OP_COMM + OP_BRIGHTNESS
typedef struct {
    uint8_t     opcode;     //OP_COMM
    uint8_t     subOpcode;  //OP_BRIGHTNESS
    uint8_t     sequence;   //sequence number
    uint8_t     action;     //0-100%
} __packed Brightness_Payload;

/********** Common Avengers Functions **********/
bool PlcPacketFilter(CM2Packet* packet);
void HandlePlcAcking_Common(CM2Packet* packet, uint8_t devType, Opcode ackType, uint8_t nackReason, bool blocking);
#endif

#endif
