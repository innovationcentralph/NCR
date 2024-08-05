/*
 * PWX_ST50H_Modbus.c
 *
 *  Created on: Aug 18, 2023
 *      Author: Charles Kim Kabiling
 */

#include "PWX_ST50H_Modbus.h"

// CRC lookup table for Modbus CRC calculation
static const uint16_t wCRCTable[] =
{
  0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
  0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
  0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
  0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
  0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
  0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
  0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
  0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
  0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
  0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
  0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
  0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
  0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
  0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
  0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
  0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
  0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
  0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
  0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
  0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
  0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
  0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
  0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
  0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
  0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
  0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
  0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
  0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
  0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
  0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
  0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
  0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};




Pin_t MODBUS_EN;
UART_HandleTypeDef modbusSerial;
ModBus_t ModbusResponse;

// Function prototypes
void Serial_Transmit(uint8_t *data, uint16_t length);



/** @brief Initialize Modbus configuration.
 *
 * @param serialPort Pointer to the UART handle for Modbus communication.
 * @param EN_GPIOPort GPIO port for MODBUS_EN pin.
 * @param EN_GPIOPin GPIO pin for MODBUS_EN.
 */
void initModbus(UART_HandleTypeDef *serialPort, GPIO_TypeDef * EN_GPIOPort, uint16_t EN_GPIOPin){

	modbusSerial = *serialPort;
	MODBUS_EN.port = EN_GPIOPort;
	MODBUS_EN.pin = EN_GPIOPin;

}

/**
 * @brief Sends raw Modbus command data and handles the response.
 *
 * This function sends raw Modbus command data, receives the response using UART interrupts,
 * and handles the response in the provided `modbusResponse` structure.
 *
 * @param modbusCMD Pointer to the raw Modbus command data.
 * @param cmdLen Length of the Modbus command data.
 * @param modbusResponse Pointer to the ModBus response structure.
 */
void sendRaw(uint8_t *modbusCMD, uint16_t cmdLen, ModBus_t *modbusResponse) {

	// Clear response buffer and reset index
	memset(modbusResponse->buffer, '\0', sizeof(modbusResponse->buffer) * sizeof(modbusResponse->buffer[0]));
	modbusResponse->rxIndex = 0;

	// Enable MODBUS_EN
	HAL_GPIO_WritePin(MODBUS_EN.port, MODBUS_EN.pin, GPIO_PIN_SET);

	// Receive data using UART interrupt
	HAL_UART_Receive_IT(&modbusSerial, (uint8_t *)modbusResponse->buffer, 1);

//	printf("MODBUS CMD XXX (Hex): ");
//	for (int i = 0; i < cmdLen; i++) {
//		printf("%02X ", modbusCMD[i]);
//	}
//	printf("\n");

	// Transmit the raw data
	Serial_Transmit(modbusCMD, cmdLen);

	// Delay to ensure proper communication
	//HAL_Delay(1);

	// Disable MODBUS_EN
	HAL_GPIO_WritePin(MODBUS_EN.port, MODBUS_EN.pin, GPIO_PIN_RESET);

	// Delay for stability
	HAL_Delay(1);


}

/**
 * @brief Sends raw Modbus command data with CRC computation and handles the response.
 *
 * This function calculates the CRC for the provided Modbus command data, appends the CRC to the data,
 * sends the combined data, receives the response using UART interrupts, and handles the response
 * in the provided `modbusResponse` structure.
 *
 * @param modbusCMD Pointer to the raw Modbus command data.
 * @param cmdLen Length of the Modbus command data.
 * @param modbusResponse Pointer to the ModBus response structure.
 */
void sendRaw_CRC(uint8_t *modbusCMD, uint16_t cmdLen, ModBus_t *modbusResponse) {

	// Calculate CRC for modbusCMD
	uint16_t crc = calculateModbusCRC(modbusCMD, sizeof(modbusCMD));

	// Convert CRC to two bytes
	uint8_t crcBytes[2];
	crcBytes[0] = (uint8_t)(crc & 0xFF);
	crcBytes[1] = (uint8_t)((crc >> 8) & 0xFF);

	// Calculate the total length of data (modbusCMD + CRC)
	uint16_t totalLength = cmdLen + sizeof(crcBytes);

	// Allocate memory for the combined data (modbusCMD + CRC)
	uint8_t combinedData[totalLength];

	// Copy modbusCMD to the beginning of combinedData
	memcpy(combinedData, modbusCMD, cmdLen);

	// Copy CRC bytes to the end of combinedData
	memcpy(combinedData + sizeof(modbusCMD), crcBytes, sizeof(crcBytes));

	// Clear response buffer and reset index
	memset(modbusResponse->buffer, 0, sizeof(modbusResponse->buffer));
	modbusResponse->rxIndex = 0;

	// Enable MODBUS_EN
	HAL_GPIO_WritePin(MODBUS_EN.port, MODBUS_EN.pin, GPIO_PIN_SET);

	// Receive data using UART interrupt
	HAL_UART_Receive_IT(&modbusSerial, modbusResponse->buffer, totalLength);

	// Transmit the combined data
	Serial_Transmit(combinedData, totalLength);

	// Delay to ensure proper communication
	HAL_Delay(10);

	// Disable MODBUS_EN
	HAL_GPIO_WritePin(MODBUS_EN.port, MODBUS_EN.pin, GPIO_PIN_RESET);

	// Delay for stability
	HAL_Delay(10);

}

// Transmit data through UART
void Serial_Transmit(uint8_t *data, uint16_t length) {
    // Assuming UART1 has been initialized
    // Transmit data
    HAL_UART_Transmit(&modbusSerial, data, length, HAL_MAX_DELAY);
}

/**
 * @brief Callback function to handle Modbus response data reception.
 *
 * This function is called when Modbus response data is received. It stores the received character in the
 * `buffer` of the `_ModbusResponse` structure, updates the `rxIndex`, and continues to receive the next character.
 * If the buffer overflows, the `rxIndex` is reset to prevent data loss.
 *
 * @param _ModbusResponse Pointer to the ModBus response structure.
 */
void Modbus_RxCallback(ModBus_t *_ModbusResponse) {
    if (_ModbusResponse->rxIndex < sizeof(_ModbusResponse->buffer)) {
        _ModbusResponse->buffer[_ModbusResponse->rxIndex] = modbusSerial.Instance->RDR;  // Store the received character
        _ModbusResponse->rxIndex++;


    } else {
        // Buffer overflow, reset the index
        _ModbusResponse->rxIndex = 0;
    }

    // Continue to receive the next character
    HAL_UART_Receive_IT(&modbusSerial, (uint8_t *)(_ModbusResponse->buffer + _ModbusResponse->rxIndex), 1);
}

/**
 * @brief Calculates the length of a null-terminated array.
 *
 * This function calculates the length of a null-terminated (string) array, which is the number of characters in the array
 * until the null terminator is encountered.
 *
 * @param array Pointer to the null-terminated array.
 * @return The length of the array (number of characters excluding the null terminator).
 */
size_t getArrayLength(const uint8_t *array) {
    size_t length = 0;
    while (array[length] != '\0') {
        length++;
    }
    return length;
}

/**
 * @brief Calculates the Modbus CRC for the given data.
 *
 * This function calculates the Modbus CRC (Cyclic Redundancy Check) for the specified data.
 *
 * @param nData Pointer to the data for which CRC needs to be calculated.
 * @param wLength Length of the data in bytes.
 * @return The calculated CRC value.
 */
uint16_t calculateModbusCRC(const uint8_t *nData, uint16_t wLength)
{
  uint8_t nTemp;
  uint16_t wCRCWord = 0xFFFF;
  while (wLength--)
  {
    nTemp = *nData++ ^ wCRCWord;
    wCRCWord >>= 8;
    wCRCWord  ^= wCRCTable[nTemp];
  }
  return wCRCWord;
}




