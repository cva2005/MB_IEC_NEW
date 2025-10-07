#ifndef _CONFIG
#define _CONFIG

#include "mb_types.h"
#include "esp_err.h"
#include "esp_attr.h"

typedef enum
{
	SLAVE_IEC_104_TCP = 0,
	SLAVE_IEC_101_SERIAL = 1,
} slave_select_t;

typedef enum
{
	ASDU_ADDR_1B = 0,
	ASDU_ADDR_2B = 1,
} addr_size_t;

typedef enum {
	IOA_SIZE_1B = 0,
	IOA_SIZE_2B = 1,
	IOA_SIZE_3B = 2,
} ioa_size_t;

typedef enum {
	COT_SIZE_1B = 0,
	COT_SIZE_2B = 1,
} cot_size_t;

typedef enum {
	DISABLE = 0,
	ENABLE  = 1,
} select_t;

/* Settable Baud Rate */
typedef enum
{
	BAUD_1200 = 0,
	BAUD_2400 = 1,
	BAUD_4800 = 2,
	BAUD_9600 = 3,
	BAUD_19200 = 4,
	BAUD_38400 = 5,
	BAUD_57600 = 6,
	BAUD_115200 = 7,
	BAUD_128000 = 8,
	BAUD_230400 = 9,
	BAUD_256000 = 10,
	BAUD_460800 = 11,
	BAUD_921600 = 12,
} baudr_t;

extern const uint32_t BaudVal[];

/* Settable Data Bits */
typedef enum {
	DATABITS_7 = 0,
	DATABITS_8 = 1
} datab_t;

/* Settable Stop bits */
typedef enum {
	STOPBITS_1 = 0,
	/* One Stop bits */
	STOPBITS_2 = 1,
	/* Two Stop bits */
	STOPBITS_1_5 = 2,
	/* 1.5 Stop bits */
} stopb_t;

/* Settable Parity bits */
typedef enum {
	PARITY_NONE = 0,
	PARITY_ODD  = 1,
	PARITY_EVEN = 2,
} parity_t;

typedef enum
{
	UNBALANCED = 0,
	BALANCED = 1,
} ll_mode_t;

#define CFG_MAX_LEN     64  /* ToDo: compare with configurator */
#define TBL_RW          14
#define RW_REG_ADDR		0
#define RW_REG_TYPE		1
#define RW_IEC_TYPE		2
#define RW_SWAP_TYPE 	3
#define RW_TRESHOLD 	4
#define RW_OBJ_ADDR		5
#define RW_HIGH_LIM 	6
#define RW_SLAVE_ID		7
#define RW_LOW_LIM		8
#define RW_OBJ_NUM		9
#define RW_POLL_MS		10
#define RW_FAULT_VAL	11
#define RW_FAULT_TO		12
#define RW_GROUP_NUM	13
#define STR_MAX			24

typedef enum
{
	SINGLE_POINT = 0,	/* Single point */
	DOUBLE_POINT = 1,	/* Double point */
	STEP_POSITION = 2,	/* Step position */
	NORMALIZED_VAL = 3, /* Measured value (Normalized) */
	SCALED_VAL = 4,		/* Measured value (Scaled) */
	FLOATING_VAL = 5,	/* Measured value (Floating) */
	INTEGR_TOTALS = 6,	/* Integrated totals */
	BIT_STRING_32 = 7,	/* Bitstring of 32 bit */
} iec_data_t;

typedef enum
{
	TIME_NONE = 0,	  /* without time stamp */
	TIME_24_BITS = 1, /* time stamp 24 bits */
	TIME_56_BITS = 2, /* time stamp 56 bits */
} iec_time_t;

typedef enum
{
	SW_NONE = 0, /* Don't need to swap */
	SW_BYTE = 1, /* Byte swap */
	SW_WORD = 2, /* Word swap */
	SW_B_W = 3,	 /* Byte and Word swap */
} swap_t;

typedef enum
{
	COIL_STATUS_RW = 0,	 /* 01 Coil Status (0x) Read/Write Mode */
	COIL_STATUS_RO = 1,	 /* 01 Coil Status (0x) Read Only Mode */
	COIL_STATUS_WO = 2,	 /* 01 Coil Status (0x) Write Only Mode */
	INPUT_STATUS_RO = 3, /* 02 Input Status (1x) Read Only Permanent */
	HOLDING_REG_RW = 4,	 /* 03 Holding Register (4x) Read/Write Mode */
	HOLDING_REG_RO = 5,	 /* 03 Holding Register (4x) Read Only Mode */
	HOLDING_REG_WO = 6,	 /* 03 Holding Register (4x) Write Only Mode */
	INPUT_REG_RO = 7,	 /* 04 Input Registers (3x) Read Only Permanent */
} mb_func_t;

#define TRESHOLD_OFF 	(0)
#define LOW_VAL_MIN 	(-99999999)
#define HIGH_VAL_MAX 	(100000000)

#pragma pack(1)
typedef struct
{
	uint16_t mb_dev_id;		/* ModBus Slave ID + TCP mode signature (ID + 256) */
	uint8_t mb_function;	/* ModBus Function */
	uint8_t swap_data;		/* Swap data */
	uint8_t iec_data_type;	/* IEC Data Type */
	uint8_t iec_group;		/* Interrogation by staion/group 1~16 */
	uint16_t mb_reg_addr;	/* ModBus Registry Address */
	uint16_t fault_sec;		/* ModBus Fault Timeout */
	uint16_t iec_obj_num;	/* Number elements of IEC Object */
	uint32_t iec_obj_addr;	/* IEC first Object Address */
	uint32_t mb_poll_ms;	/* ModBus Poll interval */
	float fault_val;		/* Fault value */
	float low_limit;		/* Low Limit */
	float high_limit;		/* High Limit */
	float threshold;		/* Threshold of Insensibility */
} table_t;

typedef struct {
	/* Application Layer */
	uint16_t AsduAdr;	/* ASDU Address */
	uint16_t tPSt;		/* Point Status Timeout */
	uint8_t alAdrSize;	/* ASDU Address Size */
	uint8_t ioaSize;	/* IOA Size */
	uint8_t cotSize;	/* COT Size */
	uint8_t actTerm;	/* Active Termination */
	uint8_t spVal;		/* Enable/disable spontaneous feature */
	uint8_t tIrr;		/* General interrogation response time stamp */
	uint8_t tEvt;		/* Event with/without time stamp */
	uint8_t tMval;		/* Measured value cyclic time stamp */
	uint32_t tNorm;		/* Cyclic send measured value (normalized) 0-2073600 s; 0 for disable */
	uint32_t tScal;		/* Cyclic send measured value (scaled) */
	uint32_t tFloat;	/* Cyclic send measured value (short floating point number) */
	uint16_t tSel;		/* Select timeout 0-600 s; 0 for executing only */
	uint8_t orgAdr;		/* Originator Address 0-255 */
	/* IEC 870-5-104 */
	uint8_t t0;		  	/* Timeout 0 */
	uint16_t t1;		/* Timeout 1 */
	uint16_t t2;		/* Timeout 2 */
	uint16_t t3;		/* Timeout 3 */
	uint16_t K;			/* K Parameter */
	uint16_t W;		  	/* W Parameter */
	uint16_t tcpPort; 	/* TCP Port */
	uint32_t ipAdr;	  	/* IP Address */
	/* IEC 870-5-101 */
	uint8_t Baud;		/* Baud Rate */
	uint8_t Parity;		/* Parity */
	uint8_t stopBits;	/* Stop Bits */
	uint8_t dataBits;	/* Data Bits */
	uint16_t tRx;       /* Timeout Frames Rx */
	uint8_t chRetry;	/* Link layer retries */
	/* Link Layer [FT1.2] */
	uint8_t llMode;	   /* Link Layer Mode */
	uint8_t llAdrSize; /* Link Address Size */
	uint8_t usACK;	   /* Use Single Char ACK */
	uint16_t llAdr;	   /* Link Address */
	uint32_t tAck;	   /* Timeout for ACK,ms */
	uint32_t tRep;	   /* Timeout Repead,ms */
	/* ModBus Serial */
	uint8_t mbB0;		/* Baud Rate */
	uint8_t mbB1;		/* Baud Rate */
	uint8_t mbP0;		/* Parity */
	uint8_t mbP1;		/* Parity */
	uint8_t mbS0;		/* Stop Bits */
	uint8_t mbS1;		/* Stop Bits */
	uint8_t mbD0;		/* Data Bits */
	uint8_t mbD1;		/* Data Bits */
	uint8_t mbM0;		/* Mode of Serial MB */
	uint8_t mbM1;		/* Mode of Serial MB */
	uint16_t mbRetry;	/* Channel Retry */
	uint16_t mbInDel;	/* Initial delay,ms 0-30000 */
	uint16_t mbRespT;	/* Responde Timeout,ms */
	/* ModBus Ethernet */
	uint16_t eP0;		/* TCP Port */
	uint16_t eP1;		/* TCP Port */
	uint16_t eP2;		/* TCP Port */
	uint16_t eP3;		/* TCP Port */
	uint16_t eP4;		/* TCP Port */
	uint16_t eP5;		/* TCP Port */
	uint16_t eP6;		/* TCP Port */
	uint16_t eP7;		/* TCP Port */
	uint32_t iP0;		/* IP Address */
	uint32_t iP1;		/* IP Address */
	uint32_t iP2;		/* IP Address */
	uint32_t iP3;		/* IP Address */
	uint32_t iP4;		/* IP Address */
	uint32_t iP5;		/* IP Address */
	uint32_t iP6;		/* IP Address */
	uint32_t iP7;		/* IP Address */
	uint16_t eRespT;	/* Ethernet Responde Timeout,ms */
	uint16_t eRetry;	/* Channel Retry */
	/*  Gateway Configuration Table */
	uint16_t tLen; 		/* Confiure table lengt */
	table_t CfgA[CFG_MAX_LEN];
	uint16_t tFault;	/* Fault timeout 100 - 65535 ms 6000 */
	uint16_t tSync;		/* Sync time interval 0 - 65535 [min] default - 30 (0 - OFF) */
	uint16_t tSwith;	/* Swith Reserwed Channel delay time 0 - 65535 [sec] default - 10 (0 - OFF) */
	uint16_t tBetw;		/* Delay Between Polls, 0 - 65535 [ms] default - 100 */
	uint16_t SerN;		/* Serial Number of Device [Read Only] */
	uint16_t VerFW;		/* Version of FirmWare [Read Only] */
	uint16_t crc16;		/* Control Summ */
} config_t;
#pragma pack()

typedef struct {
	char* key;
	void *ptr;
	uint8_t size;
} parsed_t;

#define PARSE_TAB_LEN 		73
#define FIRST_DIG			1
#define PARSE_ARR_LEN		1024
#define PARSE_BUF_LEN		1280
#define EEPROM_CONFIG_ADDR	0xfff0

extern config_t RamCfg;
extern parsed_t ParseTab[PARSE_TAB_LEN];
extern slave_select_t slave_select;

esp_err_t read_config(void);
esp_err_t write_config(void);
esp_err_t save_serial_key(uint16_t value);
esp_err_t write_config_default(void);
bool is_web_server(void);

#define IEC_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define QUEUE_NO_WAIT (0)
#define GTW_QUEUE_TIMEOUT 	(200)
#define WAIT_GTW_TIME_MS 	(200)
#define GTW_CHECK_TIME_MS 	(1)

#endif // !defined(_CONFIG)
