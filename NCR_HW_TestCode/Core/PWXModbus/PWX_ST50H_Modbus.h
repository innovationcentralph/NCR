/*
 * PWX_ST50H_Modbus.h
 *
 *  Created on: Aug 16, 2023
 *      Author: Charles Kim Kabiling
 */

#ifndef INC_PWX_ST50H_MODBUS_H_
#define INC_PWX_ST50H_MODBUS_H_

#include "main.h"
#include  <stdbool.h>
#include  <string.h>
#include  <stdio.h>
#include <stddef.h>


// Define a macro for delay using the CMSIS RTOS delay function
#define   modbus_delay(x)  osDelay(x)


// Structure to represent a GPIO pin
typedef struct {
    GPIO_TypeDef *port; // Pointer to GPIO port (e.g., GPIOA, GPIOB, ...)
    uint8_t pin;       // Pin number (0 - 15)
}Pin_t;


// Modbus RX buffer size
#define MODBUS_RX_BUFFER 150

// External variables
extern Pin_t MODBUS_EN; // Declare MODBUS_EN as an external variable
extern UART_HandleTypeDef modbusSerial; // Declare modbusSerial as an external variable


// Enumeration of Modbus command types
typedef enum
{
  ModbusCMD_ReadCoilStatus = 1,
  ModbusCMD_ReadDiscreteInputs = 2,
  ModbusCMD_ReadHoldingRegisters = 3,
  ModbusCMD_ReadInputRegisters = 4,
  ModbusCMD_WriteSingleCoil = 5,
  ModbusCMD_WriteSingleRegister = 6,
  ModbusCMD_WriteMultipleCoils = 15,
  ModbusCMD_WriteMultipleRegisters = 16

}ModbusCMD_t;

// Enumeration of 16-bit order options for Modbus
typedef enum
{
  ModBus_16bitOrder_AB = 0,
  ModBus_16bitOrder_BA,

}ModBus_16bitOrder_t;


// Enumeration of 32-bit order options for Modbus
typedef enum
{
  ModBus_32bitOrder_ABCD = 0,
  ModBus_32bitOrder_DCBA,
  ModBus_32bitOrder_BADC,
  ModBus_32bitOrder_CDAB,

}ModBus_32bitOrder_t;

typedef enum
{
  Modbus_Float_ABCD = 0,
  Modbus_Float_DCBA,
  Modbus_Float_BADC,
  Modbus_Float_CDAB,
  Modbus_uInt32_ABCD,
  Modbus_uInt32_DCBA,
  Modbus_uInt32_BADC,
  Modbus_uInt32_CDAB,
  Modbus_Int32_ABCD,
  Modbus_Int32_DCBA,
  Modbus_Int32_BADC,
  Modbus_Int32_CDAB,
  Modbus_Int16_AB,
  Modbus_Int16_BA,
  Modbus_uInt16_AB,
  Modbus_uInt16_BA,
  Modbus_Int8,
  Modbus_uInt8,
}VarData_BitOrder_t;

// Structure to hold Modbus response data
typedef struct
{
  uint16_t              rxIndex;
  uint8_t               buffer[MODBUS_RX_BUFFER];

}ModBus_t;

/** @brief Initialize Modbus configuration.
 *
 * @param serialPort Pointer to the UART handle for Modbus communication.
 * @param EN_GPIOPort GPIO port for MODBUS_EN pin.
 * @param EN_GPIOPin GPIO pin for MODBUS_EN.
 */
void initModbus(UART_HandleTypeDef *serialPort, GPIO_TypeDef * EN_GPIOPort, uint16_t EN_GPIOPin);


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
void sendRaw(uint8_t *modbusCMD, uint16_t cmdLen, ModBus_t *modbusResponse);

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
void sendRaw_CRC(uint8_t *modbusCMD, uint16_t cmdLen, ModBus_t *modbusResponse);

/**
 * @brief Callback function to handle Modbus response data reception.
 *
 * This function is called when Modbus response data is received. It stores the received character in the
 * `buffer` of the `_ModbusResponse` structure, updates the `rxIndex`, and continues to receive the next character.
 * If the buffer overflows, the `rxIndex` is reset to prevent data loss.
 *
 * @param _ModbusResponse Pointer to the ModBus response structure.
 */
void Modbus_RxCallback(ModBus_t *_ModbusResponse);

/**
 * @brief Calculates the Modbus CRC for the given data.
 *
 * This function calculates the Modbus CRC (Cyclic Redundancy Check) for the specified data.
 *
 * @param nData Pointer to the data for which CRC needs to be calculated.
 * @param wLength Length of the data in bytes.
 * @return The calculated CRC value.
 */
uint16_t calculateModbusCRC(const uint8_t *nData, uint16_t wLength);

/**
 * @brief Calculates the length of a null-terminated array.
 *
 * This function calculates the length of a null-terminated (string) array, which is the number of characters in the array
 * until the null terminator is encountered.
 *
 * @param array Pointer to the null-terminated array.
 * @return The length of the array (number of characters excluding the null terminator).
 */
size_t getArrayLength(const uint8_t *array);


#endif /* INC_PWX_ST50H_MODBUS_H_ */
