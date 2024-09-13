/*
 * ProjectConfig.h
 *
 *  Created on: Aug 7, 2024
 *      Author: Owner
 */

#ifndef INC_PROJECTCONFIG_H_
#define INC_PROJECTCONFIG_H_

#include <stdbool.h>

// Firmware Version Control
#define FirmwareName "NCR ATM Monitoring System"
#define VERSION_MAJOR 1
#define VERSION_MID   3
#define VERSION_MINOR 8

#define MAX_LORA_PAYLOAD_BUFFER_SIZE 50
#define MAX_UART_BUFFER_SIZE 500

/* MCP23008 Definitions */
#define MCP23008_ADDR 0x21

// MCP23008 Register Addresses
#define MCP23008_IODIR    0x00  // I/O Direction Register
#define MCP23008_GPIO     0x09  // GPIO Register
#define MCP23008_GPINTEN  0x02  // GPIO Interrupt Enable Register
#define MCP23008_DEFVAL   0x03  // Default Compare Register
#define MCP23008_INTCON   0x04  // Interrupt Control Register
#define MCP23008_INTCAP   0x08  // Interrupt Capture Register
#define MCP23008_IOCON    0x05  // IO Expander Control Register

/* Private macro for Accelerometer -------------------------------------------------------------*/
#define    BOOT_TIME            20 //ms

/* Self-test recommended samples */
#define SELF_TEST_SAMPLES  5

/* Self-test positive difference */
#define ST_MIN_POS      70.0f
#define ST_MAX_POS      1500.0f

// Thresholds for SHT40
#define TEMP_HIGH 31.0
#define TEMP_LOW  28.0
#define TEMP_HYS  0.0
#define RH_HYS    0.0
#define RH_HIGH   70.0
#define RH_LOW    30.0

#define WATER_LEAK_TH 1500

#define QUEUE_MAX_SIZE 5

// Structure Definitions

// Define the enum for data types
typedef enum {
    TYPE_UINT8,
    TYPE_UINT16,
    TYPE_UINT32,
    TYPE_INT8,
    TYPE_INT16,
    TYPE_INT32,
    TYPE_FLOAT,
	TYPE_UINT8_ARRAY
} DataType;

typedef enum {
	IDLE = 0,
	DRY_CONTACT,
	SMOKE_SENSOR,
	SHT
}UnscheduledTxTriggers;
// Structure to hold Dry Contact Channel Status
typedef struct {
	uint8_t value;
    bool DC1;
    bool DC2;
    bool DC3;
    bool DC4;
    bool DC5;
    bool DC6;
    bool DC7;
    bool DC8;
} DryContactStatus;



// Structure to hold Smoke Sensor Status
typedef struct {
    bool pinA;
    bool pinB;
    const char* status;
    uint8_t level;
} SmokeStatus;

// Structure to hold LTC4015 parameters
typedef struct {
    float VIN;
    float VBAT;
    float IIN;
    float IBAT;
    float VSYS;
} LTCStatus;

// Structure to help Power Meter parameters
typedef struct {
	uint16_t voltage;
	uint16_t current;
	uint16_t power;
	uint16_t powerFactor;
	uint16_t frequency;
	uint32_t energy;
} ADL100Status;


typedef enum{
	TEST_PORT = 1,
	HEARTBEAT_PORT = 2,
	INTERRUPT_PORT = 3,
	TEST_UPLINK_PORT = 4,
	POWER_PORT = 5
}UplinkPorts;

enum{
	NORMAL = 0,
	ABOVE_THRESHOLD,
	BELOW_THRESHOLD
}AlarmStates;

typedef enum{
	CONFIRMED_UPLINK = 1,
	UNCONFIRMED_UPLINK = 0,
}UplinkTransmissionState;

typedef struct {
	float temperature;
	float humidity;
	struct{
		float temp_low;
		float temp_high;
		float temp_hys;
		float rel_low;
		float rel_high;
		float rel_hys;
	}thresholds;
	struct{
		uint8_t temperature;
		uint8_t humidity;
	}alarmState;
}SHT40;

typedef enum{
	STATIONARY = 1,
	SMASHED = 2,
	MOVING = 3
}AccelState;

typedef struct {
	AccelState status;
}Accel;

typedef enum{
	DRY = 0,
	WET = 1
}LeakState;

typedef struct{
	float raw;
	LeakState state;
}WaterLeak;

typedef struct{

	SHT40 sht40;
	SmokeStatus smoke;
	DryContactStatus dryContact;
	LTCStatus ltc4015;
	Accel accel;
	WaterLeak leak;
}Sensors;

typedef enum {
    HEARTBEAT = 0,
    UNSCHEDULED_TRANSMISSION,
    DIAGNOSTICS,
    ACKNOWLEDGEMENT,
    DOWNLINK_QUERY_RESPONSE,
	POWER_PARAMS
} MessageType;

typedef struct{
	MessageType msgType;
	uint8_t buffer[MAX_LORA_PAYLOAD_BUFFER_SIZE];
	uint8_t length;
}TxPayload;

typedef union {
  int16_t i16bit[3];
} axis3bit16_t;


typedef struct {
    TxPayload queue[QUEUE_MAX_SIZE];
    uint8_t front;
    uint8_t rear;
    uint8_t size;
} TxPayloadQueue;


// System Behavior Macros
#define SERIAL_DEBUG_SHT
//#define SERIAL_DEBUG_SENSORS
//#define SERIAL_DEBUG_INTERRUPT
//#define SCAN_I2C_DEVICES
//#define SERIAL_DEBUG_PAYLOADCHECK
//#define SERIAL_DEBUG_LTC

#define SHT_READ_INTERVAL   500
#define DEVICE_HEARTBEAT    30000
#define WDT_RESET_INTERVAL  5000
#define MCU_REST_INTERVAL   180000
#define QUEUE_SEND_INTERVAL 15000
#define WARM_UP_TIME        10000
#define ACREL_READ_INTERVAL 30000


// Global Variables
// ModBus Command to get data from Meter



#define GET_METER_ERG_CMD   {0x01, 0x03, 0x00, 0x3c, 0x00, 0x04, 0x84, 0x05}
#define GET_METER_BASIC_CMD {0x01, 0x03, 0x00, 0x0B, 0x00, 0x07, 0x75, 0xCA}

#define GetMeterData_LEN 8

#define METER_CMD_ON  {0x01,0x10,0x00,0x57,0x00,0x02,0x04,0x00,0x01,0x00,0x00,0xe6,0xb5}
#define METER_CMD_ON_LEN  13

#define METER_CMD_OFF {0x01,0x10,0x00,0x57,0x00,0x02,0x04,0x00,0x01,0x00,0x01,0x27,0x75}
#define METER_CMD_OFF_LEN 13

#define METER_CMD_PREPAID {0x01,0x10,0x00,0x57,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0xb7,0x75}
#define METER_CMD_PREPAID_LEN 13

#endif /* INC_PROJECTCONFIG_H_ */
