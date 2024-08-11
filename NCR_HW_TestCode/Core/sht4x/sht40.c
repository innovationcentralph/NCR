/*
 * sht40.c
 * Author: Kevin Zheng
 */

#include "sht40.h"

/*
 * Static function prototypes
 */
static bool verify_checksum(uint16_t data, uint8_t checksum);
static HAL_StatusTypeDef i2c_communicate(I2C_HandleTypeDef* i2cHandle, uint8_t* command, uint8_t* response);
static HAL_StatusTypeDef read_temp_humidity(uint8_t* response, SHT40_Measurement* result);
static float hardware_to_celsius(uint16_t reading);
static float hardware_to_humidity_percentage(uint16_t reading);

/*
 * LUT for CRC8 with polynomial 0x31
 */
static const uint8_t CRC8_TABLE[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC,
};

/*
 * Measurement-Taking
 */
HAL_StatusTypeDef SHT40_Measure(I2C_HandleTypeDef* i2cHandle, SHT40_Measurement* result, SHT40_Precision precision) {
    uint8_t response[6];
    if (i2c_communicate(i2cHandle, (uint8_t*)&precision, response) == HAL_OK) {
        return read_temp_humidity(response, result);
    }
    return HAL_ERROR;
}

/*
 * Heater Control
 */
HAL_StatusTypeDef SHT40_Heat(I2C_HandleTypeDef* i2cHandle, SHT40_Measurement* result, SHT40_HeaterOption heat_option) {
    uint8_t response[6];
    if (i2c_communicate(i2cHandle, (uint8_t*)&heat_option, response) == HAL_OK) {
        return read_temp_humidity(response, result);
    }
    return HAL_ERROR;
}

/*
 * Reset
 */
HAL_StatusTypeDef SHT40_SoftReset(I2C_HandleTypeDef* i2cHandle) {
    static uint8_t COMMAND = SHT40_SOFT_RESET;

    if( HAL_I2C_Master_Transmit(i2cHandle, SHT40_I2C_ADDR, &COMMAND, 1, SHT40_I2C_TIMEOUT) != HAL_OK ) {
        return HAL_ERROR;
    } else {
        return HAL_OK;
    }
}

/*
 * Serial
 */
HAL_StatusTypeDef SHT40_ReadSerial(I2C_HandleTypeDef* i2cHandle, uint32_t* result) {
    static uint8_t COMMAND = SHT40_READ_SERIAL;
    uint8_t serial_response[6];

    if( i2c_communicate(i2cHandle, &COMMAND, serial_response) == HAL_OK ) {
        uint16_t serial_msb = serial_response[0] << 8 | serial_response[1];
        uint16_t serial_lsb = serial_response[3] << 8 | serial_response[4];

        if( verify_checksum(serial_msb, serial_response[2]) && verify_checksum(serial_lsb, serial_response[5]) ) {
            *result = ((uint32_t)serial_msb << 16) | serial_lsb;
            return HAL_OK;
        }
    }
    return HAL_ERROR;
}

/*
 * Returns whether the provided CRC8 checksum matches that
 * of the provided 16-bit data
 */
static bool verify_checksum(uint16_t data, uint8_t checksum) {
    uint8_t calculated_crc = 0xFF;

    calculated_crc = CRC8_TABLE[calculated_crc ^ (data >> 8)];
    calculated_crc = CRC8_TABLE[calculated_crc ^ (data & 0xFF)];

    return calculated_crc == checksum;
}

/*
 * Common I2C communication function
 */
static HAL_StatusTypeDef i2c_communicate(I2C_HandleTypeDef* i2cHandle, uint8_t* command, uint8_t* response) {
//    if(
//        HAL_I2C_Master_Transmit(i2cHandle, SHT40_I2C_ADDR, command, 1, SHT40_I2C_TIMEOUT) != HAL_OK ||
//
//		HAL_I2C_Master_Receive(i2cHandle, SHT40_I2C_ADDR, response, SHT40_I2C_RESP_LEN, SHT40_I2C_TIMEOUT) != HAL_OK
//    ) {
//        return HAL_ERROR;
//    }

    HAL_StatusTypeDef ret;

    HAL_Delay(100);

    ret = HAL_I2C_Master_Transmit(i2cHandle, SHT40_I2C_ADDR, command, 1, SHT40_I2C_TIMEOUT);

    if(ret != HAL_OK){
    	return HAL_ERROR;
    }

    HAL_Delay(100);

    ret = HAL_I2C_Master_Receive(i2cHandle, SHT40_I2C_ADDR, response, SHT40_I2C_RESP_LEN, SHT40_I2C_TIMEOUT);

    if(ret != HAL_OK){
		return HAL_ERROR;
	}

    HAL_Delay(100);

    return HAL_OK;
}

/*
 * Read temperature and humidity from the I2C response
 */
static HAL_StatusTypeDef read_temp_humidity(uint8_t* response, SHT40_Measurement* result) {
    uint16_t temperature = response[0] << 8 | response[1];
    uint16_t humidity = response[3] << 8 | response[4];

    if( verify_checksum(temperature, response[2]) && verify_checksum(humidity, response[5]) ) {
        result->temperature = hardware_to_celsius(temperature);
        result->rel_humidity = hardware_to_humidity_percentage(humidity);
        return HAL_OK;
    }
    return HAL_ERROR;
}

/*
 * Converts hardware representation of temperature to C
 */
static float hardware_to_celsius(uint16_t reading) {
    return -45.0 + 175.0 * (reading / 65535.0);
}

/*
 * Converts hardware representation of humidity to relative humidity as percentage
 */
static float hardware_to_humidity_percentage(uint16_t reading) {
    return -6.0 + 125.0 * (reading / 65535.0);
}
