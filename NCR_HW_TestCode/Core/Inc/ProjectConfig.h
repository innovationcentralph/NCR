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
#define VERSION_MID   1
#define VERSION_MINOR 4

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


// Structure Definitions

// Define the enum for data types
typedef enum {
    TYPE_UINT8,
    TYPE_UINT16,
    TYPE_UINT32,
    TYPE_INT8,
    TYPE_INT16,
    TYPE_INT32,
    TYPE_FLOAT
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
	TEST_UPLINK_PORT = 4
}UplinkPorts;

typedef enum{
	CONFIRMED_UPLINK = 1,
	UNCONFIRMED_UPLINK = 2,
}UplinkTransmissionState;

typedef struct {
	float temperature;
	float humidity;
}SHT40;

typedef enum{
	STATIONARY = 1,
	SMASHED = 2,
	MOVING = 3
}AccelState;

typedef struct {
	AccelState status;
}Accel;

typedef struct{

	SHT40 sht40;
	SmokeStatus smoke;
	DryContactStatus dryContact;
	LTCStatus ltc4015;
	Accel accel;
}Sensors;

typedef enum {
    HEARTBEAT = 0,
    UNSCHEDULED_TRANSMISSION,
    DIAGNOSTICS,
    ACKNOWLEDGEMENT,
    DOWNLINK_QUERY_RESPONSE
} MessageType;

typedef struct{
	MessageType msgType;
	uint8_t buffer[MAX_LORA_PAYLOAD_BUFFER_SIZE];
	uint8_t length;
}TxPayload;

typedef union {
  int16_t i16bit[3];
} axis3bit16_t;


// System Behavior Macros
//#define SERIAL_DEBUG_SHT
#define SERIAL_DEBUG_SENSORS
//#define SERIAL_DEBUG_INTERRUPT
#define SCAN_I2C_DEVICES
//#define SERIAL_DEBUG_PAYLOADCHECK

#define SHT_READ_INTERVAL 1000
#define DEVICE_HEARTBEAT  10000
#define WDT_RESET_INTERVAL 5000




// Global Variables
// ModBus Command to get data from Meter
#define GET_METER_CMD {0x00, 0x01, 0x02, 0x03, 0x04}
#define GetMeterData_LEN 5

#endif /* INC_PROJECTCONFIG_H_ */
