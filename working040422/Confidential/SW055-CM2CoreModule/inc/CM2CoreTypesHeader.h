/******************************** VERSION INFO ********************************
	Created by Brad Higgins
	08/01/2016: Syncing CM2 & JL Spirit code between projects and added version info.
	08/28/2015: Initial creation.
******************************* END VERSION INFO ******************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CM2CORETYPES_HEADER
#define CM2CORETYPES_HEADER

#include "CommonTypes.h"
#include "RawCircularBufferHeader.h"

/*** Packet stuff ***/
#define DIRECT_ADDRESSING 0x00
#define BROADCAST_ADDRESSING 0xFF

#define IS_TYPE_ADDRESSING(x) ((x)!=DIRECT_ADDRESSING && (x)!=BROADCAST_ADDRESSING)
#define IS_PACKET_VALID(x) ((x)!=0 && ((x)->command)!=0 && ((x)->commandLen)>0)
//#define IS_ACK_REQUIRED_RX(x) ((x)->deviceSource != PORT_INTERNAL && (x)->command[0]!=OP_LOW_LEVEL_ACK && (x)->command[0]!=OP_ACK && (x)->command[0]!=OP_NACK && (x)->destType==DIRECT_ADDRESSING)
//#define IS_ACK_REQUIRED_TX(x) ((x)->deviceDest   != PORT_INTERNAL && (x)->command[0]!=OP_LOW_LEVEL_ACK && (x)->command[0]!=OP_ACK && (x)->command[0]!=OP_NACK && (x)->destType==DIRECT_ADDRESSING)
//#define IS_ACK_REQUIRED_RX(x) ((x)->deviceSource != PORT_INTERNAL && (x)->command[0]!=OP_LOW_LEVEL_ACK && (x)->command[0]!=OP_ACK && (x)->command[0]!=OP_NACK && (x)->destType==DIRECT_ADDRESSING)
//#define IS_ACK_REQUIRED_TX(x) ((x)->deviceDest   != PORT_INTERNAL && (x)->command[0]!=OP_LOW_LEVEL_ACK && (x)->command[0]!=OP_ACK && (x)->command[0]!=OP_NACK && (x)->destType==DIRECT_ADDRESSING)
//EMH#define IS_ACK_REQUIRED_RX(x) ((x)->deviceSource != PORT_INTERNAL && (x)->command[0]!=OP_LOW_LEVEL_ACK && (x)->command[0]!=OP_ACK && (x)->command[0]!=OP_NACK && (x)->destType==DIRECT_ADDRESSING)
//EMH#define IS_ACK_REQUIRED_TX(x) ((x)->deviceDest != PORT_INTERNAL && (x)->command[0]!=OP_LOW_LEVEL_ACK && (x)->command[0]!=OP_ACK && (x)->command[0]!=OP_NACK && (x)->destType==DIRECT_ADDRESSING)
#define IS_ACK_REQUIRED_RX(x) ((x)->deviceSource != PORT_INTERNAL && (x)->command[0]!=OP_EX_PACKET_START && (x)->command[0]!=OP_EX_PACKET_DATA  && (x)->command[0]!=OP_LOW_LEVEL_ACK   && (x)->destType==DIRECT_ADDRESSING)
#define IS_ACK_REQUIRED_TX(x) ((x)->deviceDest != PORT_INTERNAL && (x)->command[0]!=OP_LOW_LEVEL_ACK  && (x)->destType==DIRECT_ADDRESSING)



#define IS_ADMIN_COMMAND(x)			((x)->command[0]<=0x0F)
#define IS_STANDARD_COMMAND(x)		((x)->command[0]>=0x10 && (x)->command[0]<=0x4F)
#define IS_APPLICATION_COMMAND(x)	((x)->command[0]>=0x50 && (x)->command[0]<=0xF9)
#define IS_RESERVED_COMMAND(x)		((x)->command[0]>=0xFA && (x)->command[0]<=0xFF)
//#define IS_PAPI_COMMAND(x)			//////TODO

#define IS_SCRIPTING_COMMAND(x)		((x)->command[0]==OP_SCRIPT_SET || (x)->command[0]==OP_SCRIPT_RUN || (x)->command[0]==OP_SCRIPT_STOP)

//------------- needs to be moved into CM2
#define NACK_SUPPORT_READALLONLY        0x05    /* Response when attempting to write to a read only table */
#define NACK_TABLENOTSUPPORTED          0x06    /* Response when attempting to write to a CM2 table not supported */
#define NACK_TABLEBADOFFSET             0x07    /* Offset of write command exceeds table bounds */
#define NACK_TABLEBADWRITECODE          0x08    /* Response to a write command formating error */
#define NACK_TABLETOOLONG               0x09    /* Response to a write command that exceeds table bounds */
#define NACK_FLASHERASEFAIL             0x0A    /* Denotes application's failure to successfully erase flash memory */
#define NACK_FLASHWRITEFAIL             0x0B    /* Denotes application's failure to successfully write to flash memory */
#define NACK_OPCODENOTSUPPORTED         0x0C    /* Denotes CM2 opcode is not supported for the current device */

#define NACK_TABLEENTRYBAD              0x0D    /* LEGACY - Denotes one or more parameters in attempted table write is out of bounds */
#define NACK_WRONGPARAMETER             0x0F    /* Denotes one or more parameters in attempted table write is out of bounds */
#define NACK_WRONGLENGTH                0x16    /* LEGACY - used on MIB OP_COMM response */ 
#define NACK_MIBNOTINDEMO               0x17    /* Response when receiving a demo mode specific command while not in demo mode */
#define NACK_BADTABLEVERSION            0x18    /* response to table write using incorrect table version */
#define NACK_STOPDOWNLOAD               0x19    /* way for a receiving device to STOP a cm2 download session */
#define NACK_DONWLOADFAIL               0x1a    /* error with downlaoded image */

#define NACK_UNDEFINED                  0xFF  
 
//--------------------------------
// cm2 DEVICE TYPE
#define SRC_WOLVERINE_TYPE      0x08
#define SRC_VISION_TYPE         0x09
#define SRC_SIMPLE_REMOTE_TYPE  0x1A
#define SRC_FOOT_STEERING_TYPE  0x1B
#define SRC_PROP_ONOFF_TYPE     0x1C
#define SRC_STEERLEFT_TYPE      0x1D
#define SRC_STEERRIGHT_TYPE     0x1E
#define SRC_HEADINGLOCK_TYPE    0x1F
#define SRC_PROPREVERSE_TYPE    0x20
#define SRC_ANCHORLOCK_TYPE     0x21
#define SRC_PUCK_6CHANNEL       0x29
#define SRC_MIB_TYPE            0x31
#define SRC_AVENGER_BASE_TYPE   0x38
#define SRC_NMEA_TYPE           0xC2
#define SRC_DISPLAY_TYPE        0x22

#define MAX_CMD_LEN	128
#define PORT_PACKET_BUFFER_LEN	4

typedef enum {
	PORT_INTERNAL,
	PORT_SPIRIT,
	PORT_SPIRIT_ALT,
	PORT_SPIRIT_PLC,
	PORT_BT,
	PORT_BLE,
	PORT_USB,
	PORT_UART,
	PORT_CAN,
	PORT_ALL=0xFE,
	PORT_UNKNOWN=0xFF
} CM2PortType;

//NOTE: struct has been rearranged here for memory efficiency
typedef struct
{
	CM2PortType		deviceSource;			// For packet routing
	CM2PortType		deviceDest;				// For packet routing
	uint8_t			retries;				// For packet routing
	uint8_t			protocolVer;			// Protocol version used to make this packet
	uint8_t			srcType;				// Source device type
	uint8_t			destType;				// Destination device type
	uint32_t		srcID;					// Unique source ID
	uint32_t		destID;					// Unique Destination ID
	uint8_t			subaddress;				// Denotes a specific destination in this device (left pole, 2nd puck channel, etc)
	uint8_t			commandLen;				// Length of the following command
	uint16_t		mac;					// Message Authentication Code
	uint8_t			command[MAX_CMD_LEN];	// Opcode followed by any associated parameters
} CM2Packet;

typedef enum {
	TX_IDLE,
	TX_HOLDOFF,
	TX_ACTIVE,
	TX_SUCCESS,
	TX_FAIL,
	TX_FAIL_FATAL
} TxStatus;

typedef enum {
	RX_PACKET_UNHANDLED,
	RX_PACKET_HANDLED,
	RX_PACKET_ERR_OPCODE,
	RX_PACKET_ERR_SYNTAX,
	RX_PACKET_ERR_OTHER
} RxPacketResult;

typedef struct {
	bool macFiltering:1;
	bool destTypeFiltering:1;
	bool destIDFiltering:1;
	bool enforcePairing:1;
	bool rssiFiltering:1;
	bool crcFiltering:1;
	bool nidFiltering:1;
	bool subaddrFiltering:1;
	bool duplicateFiltering:1;
} FilteringFlags;

typedef struct {
	FilteringFlags flags;
	uint8_t deviceType;
	uint8_t subaddress;
	uint32_t ID;
	uint32_t NID;
	float rssiThresh;
} FilteringSettings;

typedef struct {
	uint32_t arrivalTime;
	uint16_t MAC;
} PacketLog;

typedef enum {
	////////// Administration /////////
	OP_NULL				= 0x00,
	OP_PING				= 0x01,
	OP_LOW_LEVEL_ACK	= 0x02,
	OP_ACK				= 0x03,
	OP_NACK				= 0x04,
	OP_JOIN_REQ			= 0x05,
	OP_JOIN_BROAD		= 0x06,
	OP_OPEN_PAIR		= 0x07,
	OP_EX_PACKET_START	= 0x08,
	OP_EX_PACKET_DATA	= 0x09,
	OP_CHALLENGE		= 0x0A,
	OP_CHALLENGE_RESP	= 0x0B,
	OP_PAIR				= 0x0C,
	OP_RANGE_TEST		= 0x0D,
//EMH	OP_PRODUCTION_TEST	= 0x0E,
	OP_SIGNAL       	= 0x0E,
        OP_PAIR_COMPLETE        = 0x0F,
	////////// Standard ///////////////
	OP_REQUEST			= 0x10,
	OP_NAME				= 0x11,
	OP_VERSION			= 0x12,
	OP_RESET			= 0x13,
	OP_COMM				= 0x14,
	//OP_EX_PACKET_START	= 0x15,
	//OP_EX_PACKET_DATA	= 0x16,
	OP_TIME				= 0x17,
	OP_TEST				= 0x18,
	OP_TEST_SET			= 0x19,
	OP_DO				= 0x1A,
	OP_SCRIPT_SET		= 0x1B,
	OP_SCRIPT_RUN		= 0x1C,
	OP_SCRIPT_STOP		= 0x1D,
	OP_SCRIPT_TOGGLE	= 0x1E,
	OP_DEFAULT_FW		= 0x1F,
	OP_PROG_START		= 0x20,
	OP_PROG_DATA		= 0x21,
	OP_PROG_ABORT		= 0x22,
	OP_DIAG				= 0x23,
	OP_DIAG_PROTECTED	= 0x24,
	OP_DIAG_MEM_READ	= 0x25,
	OP_DIAG_DATA		= 0x26,
	OP_DIAG_MEM_CLR		= 0x27,
	OP_FREQ_REQ			= 0x28,
	OP_FREQ_ACK			= 0x29,
	OP_FREQ_ANN			= 0x2A,
	OP_STREAM_DATA		= 0x2B,
	OP_STREAM_ABORT		= 0x2C,
	OP_STATUS			= 0x2D,
	OP_MODE				= 0x2E,
	OP_MODE_WRITE		= 0x2F,
	OP_METER			= 0x30,
	OP_CONFIG_DATA		= 0x31,
	OP_CAL_DATA			= 0x32,
	OP_CAL_DATA_WRITE	= 0x33,
	OP_IDLE_CALLBACK	= 0x34,
	OP_TABLE_READ_REQ	= 0x35,
	OP_TABLE_DATA		= 0x36,
	OP_DIG_INPUT_REPORT = 0x37,
	OP_ANA_INPUT_REPORT = 0x38,
	//Not assigned		= 0x37 to 0x4F

	////////// Application ////////////
	OP_BMC_SET			= 0x50,
	OP_BMC_READ			= 0x51,
	OP_BMC_METER		= 0x52,
	OP_BMC_METER_CLR	= 0x53,
	//Not assigned		= 0x54 to 0xFA
	
	////////// Reserved ///////////////
	//Not assigned		= 0xFA to 0xFE
	OP_UNKNOWN			= 0xFF
} Opcode;

typedef struct {
	CM2PortType portType;			//What source will be applied to packets coming from this port?
	RawCircularBuffer rxByteBuffer;//Holds bytes as they arrive
	uint8_t packetParseState;
	TxStatus (*Transmit)(CM2PortType, CM2Packet*, bool);	//Used to transmit a packet over this port
	bool (*Ready)(void);			//Used to ask if the driver is functioning & ready
	void (*Reinit)(CM2PortType);			//Used for re-init on fault
	//bool (*IsIdle)(void);			//Used when deciding whether to sleep
	void (*handleAckingCallback)(CM2Packet*, Opcode, uint8_t, bool);
	uint16_t macFailures;
	FilteringSettings filtering;
	bool automaticLLAcks;
	uint16_t lastPacketMAC;
	uint32_t lastPacketTick;
	RawCircularBuffer packetHistory;
} Port;

/*** Script stuff ***/
/*#define IS_SCRIPT_VALID(x) (x!=0 && x->numCommands!=0 && x->delays!=0 && x->commands!=0)

typedef struct
{
	uint8_t triggerIndex;	// Where this script lives in the table, and what trigger runs it
	uint8_t numCommands;	// How many commands are in this script
	uint8_t repetition;		// How many times should this repeat? 0=infinite/until stopped
	bool running;			// Currently running?
	uint8_t currentCommand;	// Current command index
	uint8_t runCount;		// Current count of how many times it has repeated
	uint16_t* delays;		// Delay before running the command of the same index (10ms increments)
	CM2Packet** commands;	// Command list, stored as pointers to complete CM2 packets
} CM2Script;*/

typedef enum {
	MODE_HIBERNATE			= 0x00, //Fully off, including radio
	MODE_STANDBY			= 0x01, //Radio on, able to be interrupted on wake command
	MODE_CONFIG				= 0x02, //Motor/outputs idle, no recurring broadcasts
	MODE_LOW_POWER_IDLE		= 0x03,
	MODE_IDLE				= 0x04,
	MODE_NORMAL				= 0x05, //Full power operation
	MODE_BOOT				= 0x06,
	MODE_REFLASHING			= 0x07,
	//Not assigned			= 0x08 to 0x1F
	//Application specific	= 0x20 to 0xF9
	//RESERVED				= 0xFA,
	//RESERVED				= 0xFB,
	MODE_FAULT				= 0xFC,
	MODE_ERROR				= 0xFE,
	MODE_DATA_NOT_AVAIL		= 0xFF
} Mode;

typedef enum {
	FAULT_NONE					= 0x00,
	FAULT_LOW_VOLTAGE			= 0x01, 
	FAULT_HIGH_VOLTAGE			= 0x02,
	FAULT_LOW_CURRENT			= 0x03,
	FAULT_HIGH_CURRENT			= 0x04,
	FAULT_HIGH_TEMPERATURE		= 0x05,
	FAULT_OVER_TEMP_SHUTDOWN	= 0x06,
	FAULT_OPEN_LOAD				= 0x07,
	FAULT_SHORTED_LOAD			= 0x08,
	FAULT_STARTUP				= 0x09,
	FAULT_SPEED_FREQ			= 0x0A,
	FAULT_SENSOR				= 0x0B,
	FAULT_INVALID_INPUT			= 0x0C,
	FAULT_WIRING				= 0x0D,
	FAULT_SOURCE_POLARITY		= 0x0E,
	FAULT_INTERMITTENT			= 0x0F,
	FAULT_WATER_HUMIDITY		= 0x10,
	FAULT_POSITION				= 0x11,
	FAULT_STUCK					= 0x12,
	FAULT_DRAGGING				= 0x13,
	FAULT_VIBRATION				= 0x14,
	FAULT_AUDIBLE				= 0x15,
	FAULT_ELEC_GENERAL			= 0x16,
	FAULT_MPROC_COMM_ERRORS		= 0x17,
	FAULT_MPROC_REP_FAULT		= 0x18,
	FAULT_MPROC_INOPERATIVE		= 0x19,
	FAULT_PROC_OVER_TEMP		= 0x1A,
	FAULT_PROC_WATCH_DOG		= 0x1B,
	FAULT_PROC_MEM				= 0x1C,
	FAULT_COMM_PORT_UNKNOWN		= 0x1D,
	FAULT_COMM_PORT1_SPIRIT		= 0x1E,
	FAULT_COMM_PORT2_BT			= 0x1F,
	FAULT_COMM_PORT3_BLE		= 0x20,
	FAULT_COMM_PORT4_USB		= 0x21,
	FAULT_COMM_PORT5_UART		= 0x22,
	FAULT_COMM_PORT6			= 0x23, //Reserved for future use
	//Not assigned				= 0x24 to 0xF9
	//RESERVED					= 0xFA to 0xFE
	FAULT_NOT_AVAIL				= 0xFF
} Fault;

/*** Utilities ***/
/*void ClearPortFlags(CM2PortFlags* portflags);
bool AnyFlagsSet(CM2PortFlags flags);
bool AnyFlagsCommon(CM2PortFlags flags1, CM2PortFlags flags2);*/
CM2Packet* CopyPacket(CM2Packet* packet);
bool CopyPacketStatic(CM2Packet* inputPacket, CM2Packet* outputPacket);
bool PacketsIdentical(CM2Packet* packet1, CM2Packet* packet2);
void FlattenCM2Packet(CM2Packet* inputPacket, uint8_t* length, uint8_t** data, bool flattenForSerial);
void FlattenCM2PacketStatic(CM2Packet* inputPacket, uint8_t* length, uint8_t* data, bool flattenForSerial);
//CM2Packet* UnflattenCM2Packet(uint8_t length, uint8_t* data, uint16_t source, bool unflattenFromSerial);
bool UnflattenCM2PacketStatic(CM2Packet* packetToPopulate, uint8_t length, uint8_t* data, CM2PortType source, bool unflattenFromSerial);
bool UnflattenCM2PacketStatic2(CM2Packet* packetToPopulate, RawCircularBuffer* buf, CM2PortType source);
bool UnflattenCM2PacketStatic3(CM2Packet* packetToPopulate, RawCircularBuffer* buf, CM2PortType source);

// close recursive include ifdef
#endif