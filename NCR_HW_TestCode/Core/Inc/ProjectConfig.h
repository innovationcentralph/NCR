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
#define VERSION_MINOR 0


/* MCP23008 Definitions */
#define MCP23008_ADDR 0x20

// MCP23008 Register Addresses
#define MCP23008_IODIR    0x00  // I/O Direction Register
#define MCP23008_GPIO     0x09  // GPIO Register
#define MCP23008_GPINTEN  0x02  // GPIO Interrupt Enable Register
#define MCP23008_DEFVAL   0x03  // Default Compare Register
#define MCP23008_INTCON   0x04  // Interrupt Control Register
#define MCP23008_INTCAP   0x08  // Interrupt Capture Register
#define MCP23008_IOCON    0x05  // IO Expander Control Register



// Structure Definitions

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




typedef struct{

	struct {
		float temperature;
		float humidity;
	}sht20;

	SmokeStatus smoke;
	DryContactStatus dryContact;

}Sensors;




// System Behavior Macros
#define SERIAL_DEBUG_SHT
#define SERIAL_DEBUG_SENSORS
#define SCAN_I2C_DEVICES

#define SHT_READ_INTERVAL 1000
#define DEVICE_HEARTBEAL  5000




// Global Variables
// ModBus Command to get data from Meter
#define GET_METER_CMD {0x00, 0x01, 0x02, 0x03, 0x04}
#define GetMeterData_LEN 5

#endif /* INC_PROJECTCONFIG_H_ */
