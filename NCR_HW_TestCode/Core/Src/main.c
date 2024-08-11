/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "lis2dw12_reg.h"
#include <string.h>
#include "sht2x_for_stm32_hal.h"
#include "PWX_ST50H_Modbus.h"
#include "ATCommands.h"
#include <inttypes.h>
#include "sht40.h"
#include "ProjectConfig.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
UnscheduledTxTriggers TxTriggers = IDLE;
ModBus_t ModbusResp;
Sensors sensors;
LPUARTState lpuartState = UART_IDLE;
LTCStatus LTC4015;

bool hasJoinedNetwork = false;

uint32_t shtReadMillis = 0;
uint32_t sensorsReadMillis = 0;
SHT40_Measurement sht40;

DryContactStatus dryContact;

uint8_t getMeterDataCmd[] = GET_METER_CMD;

// LPUART 1 Variables
static bool responseReceived = false;
static char responseBuffer[100];
static uint16_t bufferIndex = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void WDTReset(void);
void MCP23008_Init(void);
void MCP23008_ConfigureInterrupts(void);
void readSHT40(SHT40 *sht40);
void scanI2CDevices(void);
void printLineMarker(char marker); // for debugging
void handleInterruptTriggers(UnscheduledTxTriggers trigger);


//Lora FPF
bool setLoraCredentials(void);
bool sendATCommand(char *command, uint32_t responseWaitTime, LPUARTState _lpuartState);
bool sendToLora(uint8_t portNumber, bool isConfirmedUplink, TxPayload payload);
bool joinNetwork(void);
bool generatePayload(void **inputs, DataType *types, uint8_t itemCount, MessageType msgType, TxPayload *payload);
bool generateHeartbeatTxPayload(Sensors sensors, TxPayload *payload);

DryContactStatus MCP23008_ReadInputs(void);
SmokeStatus ReadSmokeStatus(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Initialize MCP23008
  MCP23008_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize MCP23008
  MCP23008_Init();

  // Configure interrupts on MCP23008
  MCP23008_ConfigureInterrupts();

  // Initialize SHT20 Sensor
    uint32_t sht40_serial;
    if( SHT40_ReadSerial(&hi2c1, &sht40_serial) != HAL_ERROR ) {
		printf("I2C connection established to SHT40 with serial %" PRIu32 "\r\n", sht40_serial);
	} else {
		printf("Failed to read serial from SHT40; check connections and reset MCU\r\n");
	}



  // Initialize Modbus
  initModbus(&huart1, MODBUS_EN_GPIO_Port, MODBUS_EN_Pin);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)(ModbusResp.buffer + ModbusResp.rxIndex), 1);
  uint8_t rxBuffer;
  HAL_UART_Receive_IT(&hlpuart1, &rxBuffer, 1);
  // Initialize timers;
  shtReadMillis = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf(" # # # # # # # # # # -> %s\r\n ", FirmwareName);
  printf("# # # # # # # # # # -> Firmware Version %d.%d.%d\r\n ", VERSION_MAJOR, VERSION_MID, VERSION_MINOR);

#ifdef SCAN_I2C_DEVICES
  scanI2CDevices();
#endif

  WDTReset();


  printf("Setting LoRa Credentials \r\n");
//  if(!setLoraCredentials()){
//	  printf("Error setting LoRa credentials \r\n");
//  }else{
//	  printf("Success setting LoRa credentials \r\n");
//  }

  printf("Joining to Network\r\n");

//  while(hasJoinedNetwork == false){
//	  joinNetwork();
//	  printf("Retrying Joining Lora\r\n");
//  }
  printf("Success Joining Lora \r\n");

  TxPayload initPayload;
  initPayload.buffer[0] = 0x00;
  initPayload.buffer[1] = 0x01;
  initPayload.buffer[2] = 0x02;
  initPayload.length = 3;
  initPayload.msgType = DIAGNOSTICS;

  //printf("Sending Test Lora Payload\r\n");
  sendToLora(TEST_UPLINK_PORT, CONFIRMED_UPLINK, initPayload);

  sendToLora(TEST_UPLINK_PORT, CONFIRMED_UPLINK, initPayload);

  sendToLora(TEST_UPLINK_PORT, CONFIRMED_UPLINK, initPayload);



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Check for Unscheduled Transmission Requests
	  handleInterruptTriggers(TxTriggers);

  	  // Check Temperature Reading Every X Interval
  	  if(HAL_GetTick() - shtReadMillis > SHT_READ_INTERVAL){

  		readSHT40(&sensors.sht40);

#ifdef SERIAL_DEBUG_SHT
  		printLineMarker('*');
  		printf("SHT20 Reading ->Temperature: %.02f \t Humidity: %.02f\r\n", sensors.sht40.temperature, sensors.sht40.humidity);
  		printLineMarker('*');
#endif
  		/// @TODO: Insert Threshold Control here for Unscheduled TX

  		shtReadMillis = HAL_GetTick();
  	  }

  	  // Read All Sensors every Y Interval
  	  if(HAL_GetTick() - sensorsReadMillis > DEVICE_HEARTBEAL){

  		// Read SHT20
  		readSHT40(&sensors.sht40);

	    // Read Smoke Sensor
	    sensors.smoke = ReadSmokeStatus();

	    // Read DryContacts
	    sensors.dryContact = MCP23008_ReadInputs();

	    // PLaceholder for LTC4015
	    sensors.ltc4015.VIN = 24.123;
	    sensors.ltc4015.VBAT = 12.456;
	    sensors.ltc4015.VSYS = 5.789;

	    // Read ModBus Device
	    //sendRaw(getMeterDataCmd, GetMeterData_LEN, &ModbusResp);
	    //HAL_Delay(2000); // Give time to receive response

#ifdef SCAN_I2C_DEVICES
  		scanI2CDevices();
#endif

#ifdef SERIAL_DEBUG_SENSORS
	    printLineMarker('-');
	    printf("SHT20 Reading ->Temperature: %.02f \t Humidity: %.02f\r\n", sensors.sht40.temperature, sensors.sht40.humidity);
	    printf("Smoke Level -> Level %d \r\n", sensors.smoke.level);
	    printf("Dry Contact States: %d %d %d %d %d %d %d %d \r\n",
				sensors.dryContact.DC1, sensors.dryContact.DC2,
				sensors.dryContact.DC3, sensors.dryContact.DC4,
				sensors.dryContact.DC5, sensors.dryContact.DC6,
				sensors.dryContact.DC7, sensors.dryContact.DC8);
	    printf("MODBUS RESPONSE (Hex): ");
	    for (int x = 0; x < ModbusResp.rxIndex; x++) {
	        printf("%02X ", ModbusResp.buffer[x]);
	    }
	    printf(" \r\n");
	    printLineMarker('-');
#endif

	    // Generate HEARTBEAT Payload
	    TxPayload _heartBeatPayload;
	    generateHeartbeatTxPayload(sensors, &_heartBeatPayload);

	    sendToLora(HEARTBEAT_PORT, CONFIRMED_UPLINK, _heartBeatPayload);

  		sensorsReadMillis = HAL_GetTick();
  	  }

  	WDTReset();


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, WDT_DONE_Pin|MODBUS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SMOKE_B_Pin SMOKE_A_Pin */
  GPIO_InitStruct.Pin = SMOKE_B_Pin|SMOKE_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : WDT_DONE_Pin MODBUS_EN_Pin */
  GPIO_InitStruct.Pin = WDT_DONE_Pin|MODBUS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MCP23008_Init(void)
{
    uint8_t data[2];

    // Set all GPIO pins as inputs
    data[0] = MCP23008_IODIR;
    data[1] = 0xFF;  // Set all pins as input
    HAL_I2C_Master_Transmit(&hi2c1, MCP23008_ADDR << 1, data, 2, HAL_MAX_DELAY);
}

DryContactStatus MCP23008_ReadInputs(void)
{
    uint8_t data[1];
    uint8_t gpioState;

    // Request to read GPIO register
    data[0] = MCP23008_GPIO;

    HAL_I2C_Master_Transmit(&hi2c1, MCP23008_ADDR << 1, data, 1, HAL_MAX_DELAY);
    //HAL_Delay(50);
    // Read GPIO register
    HAL_I2C_Master_Receive(&hi2c1, MCP23008_ADDR << 1, &gpioState, 1, HAL_MAX_DELAY);

    DryContactStatus dryContact;
    dryContact.value = gpioState;
	dryContact.DC1 = (gpioState & (1 << 0)) ? true : false;
	dryContact.DC2 = (gpioState & (1 << 1)) ? true : false;
	dryContact.DC3 = (gpioState & (1 << 2)) ? true : false;
	dryContact.DC4 = (gpioState & (1 << 3)) ? true : false;
	dryContact.DC5 = (gpioState & (1 << 4)) ? true : false;
	dryContact.DC6 = (gpioState & (1 << 5)) ? true : false;
	dryContact.DC7 = (gpioState & (1 << 6)) ? true : false;
	dryContact.DC8 = (gpioState & (1 << 7)) ? true : false;

	return dryContact;
}

DryContactStatus MCP23008_ReadCapturedINT(void){
    uint8_t data[1];
    uint8_t gpioState;

    // Request to read GPIO register
    data[0] = MCP23008_INTCAP;

    HAL_I2C_Master_Transmit(&hi2c1, MCP23008_ADDR << 1, data, 1, HAL_MAX_DELAY);
    //HAL_Delay(10);
    // Read GPIO register
    HAL_I2C_Master_Receive(&hi2c1, MCP23008_ADDR << 1, &gpioState, 1, HAL_MAX_DELAY);

//    DryContactStatus dryContact;
    dryContact.value = gpioState;
//	dryContact.DC1 = (gpioState & (1 << 0)) ? true : false;
//	dryContact.DC2 = (gpioState & (1 << 1)) ? true : false;
//	dryContact.DC3 = (gpioState & (1 << 2)) ? true : false;
//	dryContact.DC4 = (gpioState & (1 << 3)) ? true : false;
//	dryContact.DC5 = (gpioState & (1 << 4)) ? true : false;
//	dryContact.DC6 = (gpioState & (1 << 5)) ? true : false;
//	dryContact.DC7 = (gpioState & (1 << 6)) ? true : false;
//	dryContact.DC8 = (gpioState & (1 << 7)) ? true : false;

	return dryContact;
}

void MCP23008_ConfigureInterrupts(void)
{
    uint8_t data[2];

    // Enable interrupts for all GPIO pins
    data[0] = MCP23008_GPINTEN;
    data[1] = 0xFF; // Enable interrupts on all pins
    HAL_I2C_Master_Transmit(&hi2c1, MCP23008_ADDR << 1, data, 2, HAL_MAX_DELAY);

    // Configure interrupt on change (default value) for all pins
    data[0] = MCP23008_INTCON;
    data[1] = 0x00; // Compare register is used for all pins
    HAL_I2C_Master_Transmit(&hi2c1, MCP23008_ADDR << 1, data, 2, HAL_MAX_DELAY);

    // Set default values for comparison (not strictly necessary)
    data[0] = MCP23008_DEFVAL;
    data[1] = 0x00; // No specific default value comparison
    HAL_I2C_Master_Transmit(&hi2c1, MCP23008_ADDR << 1, data, 2, HAL_MAX_DELAY);

    // Configure interrupt control to generate an interrupt on change
    // Assuming here the default configuration is sufficient

    // Read the current value of the IOCON register
	data[0] = MCP23008_IOCON;
	HAL_I2C_Master_Receive(&hi2c1, MCP23008_ADDR << 1, &data[1], 1, HAL_MAX_DELAY);

	// Modify Bit 1 and Bit 2
	// Bit 1 (INTPOL) = 1 (Active High)
	// Bit 2 (Driver Output) = 0 (Keep as 0)
	data[1] = (data[1] & ~(1 << 2)) | (1 << 1);

	// Write the updated value back to the IOCON register
	data[0] = MCP23008_IOCON;
	HAL_I2C_Master_Transmit(&hi2c1, MCP23008_ADDR << 1, data, 2, HAL_MAX_DELAY);

}

void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)
{
  /* EXTI line interrupt detected */
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) != 0x00u)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

    switch(GPIO_Pin)
    {
      case GPIO_PIN_15:
    	  MCP23008_ReadCapturedINT();
    	  TxTriggers = DRY_CONTACT;
          break;
      case SMOKE_A_Pin:
      case SMOKE_B_Pin:
    	  TxTriggers = SMOKE_SENSOR;
          break;
      default:
        break;
    }

  }
}

SmokeStatus ReadSmokeStatus(void)
{
    SmokeStatus smokeStatus;
    smokeStatus.pinA = HAL_GPIO_ReadPin(GPIOB, SMOKE_A_Pin);
    smokeStatus.pinB = HAL_GPIO_ReadPin(GPIOB, SMOKE_B_Pin);

    if (!smokeStatus.pinA && !smokeStatus.pinB) {
        smokeStatus.status = "Clean";
        smokeStatus.level = 1;
    } else if (!smokeStatus.pinA && smokeStatus.pinB) {
        smokeStatus.status = "Light Pollution";
        smokeStatus.level = 2;
    } else if (smokeStatus.pinA && !smokeStatus.pinB) {
        smokeStatus.status = "Moderate Pollution";
        smokeStatus.level = 3;
    } else if (smokeStatus.pinA && smokeStatus.pinB) {
        smokeStatus.status = "Severe Pollution";
        smokeStatus.level = 4;
    } else {
        smokeStatus.status = "Unknown"; // Fallback case, should not occur
        smokeStatus.level = 0;
    }

    return smokeStatus;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	  if (huart->Instance == USART1)
	  {
		  Modbus_RxCallback(&ModbusResp);
		  HAL_UART_Receive_IT(&huart1, (uint8_t *)(ModbusResp.buffer + ModbusResp.rxIndex), 1);
	  }

	  if (huart->Instance == LPUART1) {
		  // Handle received data
		  uint8_t receivedData = huart->pRxBuffPtr[-1]; // Last received byte

		  if(lpuartState != UART_IDLE){
			  // Store received data in buffer
			  responseBuffer[bufferIndex++] = receivedData;

			  // Check if the buffer is full or if the received character is \r or \n
			  if (receivedData == '\r' || receivedData == '\n' || bufferIndex >= MAX_UART_BUFFER_SIZE) {
				  //responseBuffer[bufferIndex] = '\r'; // Null-terminate the string
				  //printf("Received: %s\n", responseBuffer);
				  switch(lpuartState){
				  case AT_RESPONSE_CAPTURE_OK:
					  if (strstr(responseBuffer, "OK") != NULL) {
						  printf("OK RECEIVED\r\n");
						  responseReceived = true;
					  }
					  break;
				  case AT_RESPONSE_CAPTURE_NVM_STORED:
					  if (strstr(responseBuffer, "NVM DATA STORED") != NULL) {
						  responseReceived = true;
						  //printf("STORE OK RECEIVED\r\n");
					  }
					  else if (strstr(responseBuffer, "NVM DATA UP TO DATE") != NULL) {
						  responseReceived = true;
						  //printf("STORE OK RECEIVED\r\n");
					  }
					  break;
				  case AT_RESPONSE_CAPTURE_RESET:
					  if (strstr(responseBuffer, "APPLICATION_VERSION") != NULL) {
						  responseReceived = true;
						  //printf("RESET RECEIVED\r\n");
					  }
					  break;
				  case AT_RESPONSE_CAPTURE_JOIN:
					  if (strstr(responseBuffer, "EVT:JOINED") != NULL) {
						  responseReceived = true;
						  hasJoinedNetwork = true;
						  printf("EVENT JOINED\r\n");
						  //memset(responseBuffer, '\0', MAX_UART_BUFFER_SIZE);
						  bufferIndex = 0;
					  }
					  else if (strstr(responseBuffer, "EVT:JOIN FAILED") != NULL) {
						  //responseReceived = true;
						  hasJoinedNetwork = false;
						  //printf("EVENT JOIN FAILED\r\n");
						  bufferIndex = 0;
						  //memset(responseBuffer, '\0', MAX_UART_BUFFER_SIZE);
					  }
					  break;
				  case AT_RESPONSE_CAPTURE_SEND_OK:
					  if (strstr(responseBuffer, "NO_NETWORK_JOINED") != NULL) {
						  //responseReceived = true;

						  //printf("ERROR JOIN\r\n");
						  //memset(responseBuffer, '\0', MAX_UART_BUFFER_SIZE);
						  bufferIndex = 0;
						  //lpuartState = UART_IDLE;
					  }
					  else if (strstr(responseBuffer, "OK") != NULL) {
						  responseReceived = true;
						  printf("SEND PARAM OK\r\n");
						  //hasJoinedNetwork = false;
						  //printf("EVENT JOIN FAILED\r\n");
						  bufferIndex = 0;
						  //lpuartState = UART_IDLE;
						  //memset(responseBuffer, '\0', MAX_UART_BUFFER_SIZE);
					  }
					  break;
				  case UART_IDLE:
					  // do nothing for now
					  break;
				  }

				  // Clear the buffer and reset index
				  bufferIndex = 0;
			  }
		  }

//		  else if(lpuartState == AT_RESPONSE_CAPTURE_JOIN){
//			  // Store received data in buffer
//			  responseBuffer[bufferIndex++] = receivedData;
//
//
//			  if (strstr(responseBuffer, "EVT:JOINED") != NULL) {
//				  // Handle joined event
//				  // Do something, e.g., indicate success or set a flag
//				  printf("Event: JOINED\r\n");
//				  hasJoinedNetwork = true;
//				  responseReceived = true;
//				  for (int i = 0; i < MAX_UART_BUFFER_SIZE; i++) {
//				      responseBuffer[i] = '\0';
//				  }
//				  bufferIndex = 0; // Reset buffer after handling event
//			  } else if (strstr(responseBuffer, "EVT:JOIN FAILED") != NULL) {
//				  // Handle join failed event
//				  // Do something, e.g., retry connection or set an error flag
//				  printf("Event: JOIN FAILED\r\n");
//				  memset(responseBuffer, 0, MAX_UART_BUFFER_SIZE);
//				  responseReceived = true;
//				  hasJoinedNetwork = false;
//				  for (int i = 0; i < MAX_UART_BUFFER_SIZE; i++) {
//					  responseBuffer[i] = '\0';
//				  }
//				  bufferIndex = 0; // Reset buffer after handling event
//			  }
//		  }

		  // Restart reception
		  HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, 1);
	  }

}

void WDTReset(void){
	HAL_GPIO_WritePin(WDT_DONE_GPIO_Port, WDT_DONE_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(WDT_DONE_GPIO_Port, WDT_DONE_Pin, GPIO_PIN_RESET);
}

void scanI2CDevices(void){
	for (uint8_t addr = 0; addr < 128; addr++) {
	  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 5, 5000); // Adjust timeout as needed

	  if (status == HAL_OK) {
		printf(" ---- > Device found at address 0x%02X\n", addr);
	  }
	  else{
		// printf("No device found at address 0x%02X\n", addr);
	  }
	}
}

void printLineMarker(char marker) {
    for (int i = 0; i < 25; i++) {
        printf("%c ", marker);
    }
    printf("\r\n");
}

// Function to convert data to byte array
uint8_t dataToByteArray(void *input, uint8_t *output, DataType type) {
    uint8_t size = 0;
    switch (type) {
        case TYPE_UINT8:
            output[0] = *((uint8_t*)input);
            size = sizeof(uint8_t);
            break;
        case TYPE_UINT16:
            memcpy(output, input, sizeof(uint16_t));
            size = sizeof(uint16_t);
            break;
        case TYPE_UINT32:
            memcpy(output, input, sizeof(uint32_t));
            size = sizeof(uint32_t);
            break;
        case TYPE_INT8:
            output[0] = *((int8_t*)input);
            size = sizeof(int8_t);
            break;
        case TYPE_INT16:
            memcpy(output, input, sizeof(int16_t));
            size = sizeof(int16_t);
            break;
        case TYPE_INT32:
            memcpy(output, input, sizeof(int32_t));
            size = sizeof(int32_t);
            break;
        case TYPE_FLOAT:
            memcpy(output, input, sizeof(float));
            size = sizeof(float);
            break;
    }
    return size;
}


// Function to generate the payload
bool generatePayload(void **inputs, DataType *types, uint8_t itemCount, MessageType msgType, TxPayload *payload) {
    if (inputs == NULL || types == NULL || itemCount == 0 || itemCount > MAX_LORA_PAYLOAD_BUFFER_SIZE) {
        return false;
    }

    payload->msgType = msgType;
    payload->buffer[0] = msgType;
    uint8_t index = 1;

    for (uint8_t i = 0; i < itemCount; ++i) {
        uint8_t size = dataToByteArray(inputs[i], &payload->buffer[index], types[i]);
        if (index + size > MAX_LORA_PAYLOAD_BUFFER_SIZE) {
            return false; // Exceeds maximum buffer size
        }
        index += size;
    }

    payload->length = index;
    return true;
}

bool generateUnscheduledTxPayload(Sensors sensors, TxPayload *payload){

	MessageType msgType = UNSCHEDULED_TRANSMISSION;

	void *inputs[] = {&sensors.sht40.temperature, &sensors.sht40.humidity, &sensors.dryContact.value, &sensors.smoke.level};
	DataType types[] = { TYPE_FLOAT, TYPE_FLOAT, TYPE_UINT8, TYPE_UINT8 };

	bool ret = generatePayload(inputs, types, sizeof(inputs) / sizeof(void*), msgType, payload);

#ifdef SERIAL_DEBUG_PAYLOADCHECK
	if(ret){
		printf("HEARTBEAT PAYLOAD: ");
		for (uint8_t i = 0; i < payload->length; ++i) {
			printf("%02X ", payload->buffer[i]);
		}
		printf("\r\n ");
	}
	else{
		printf("Failed to generate payload\n");
	}
#endif

	return ret;
}

bool generateHeartbeatTxPayload(Sensors sensors, TxPayload *payload){

	MessageType msgType = HEARTBEAT;

	void *inputs[] = {&sensors.sht40.temperature, &sensors.sht40.humidity, &sensors.dryContact.value,
					  &sensors.smoke.level, &sensors.ltc4015.VIN, &sensors.ltc4015.VBAT, &sensors.ltc4015.VSYS};

	DataType types[] = { TYPE_FLOAT, TYPE_FLOAT, TYPE_UINT8, TYPE_UINT8, TYPE_FLOAT, TYPE_FLOAT, TYPE_FLOAT };

	bool ret = generatePayload(inputs, types, sizeof(inputs) / sizeof(void*), msgType, payload);

#ifdef SERIAL_DEBUG_PAYLOADCHECK
	if(ret){
		printf("HEARTBEAT PAYLOAD: ");
		for (uint8_t i = 0; i < payload->length; ++i) {
			printf("%02X ", payload->buffer[i]);
		}
		printf("\r\n ");
	}
	else{
		printf("Failed to generate payload\n");
	}
#endif

	return ret;
}

void handleInterruptTriggers(UnscheduledTxTriggers trigger){

	// clear remaining interrupts
	//__HAL_GPIO_EXTI_CLEAR_IT(SMOKE_A_Pin);
	//__HAL_GPIO_EXTI_CLEAR_IT(SMOKE_B_Pin);
	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);


	//DryContactStatus dryContact;
	//SmokeStatus smokeStatus;
	switch(trigger){
	case IDLE:
		// do nothing
		break;
	case DRY_CONTACT:
	case SMOKE_SENSOR:
	case SHT:

		sensors.dryContact = MCP23008_ReadInputs();

	    sensors.smoke = ReadSmokeStatus();

	    readSHT40(&sensors.sht40);


#ifdef SERIAL_DEBUG_INTERRUPT
		printLineMarker('!');
		printf("\nDRY CONTACT: %d %d %d %d %d %d %d %d \r\n",
				sensors.dryContact.DC8, sensors.dryContact.DC7, sensors.dryContact.DC6, sensors.dryContact.DC5,
				sensors.dryContact.DC4, sensors.dryContact.DC3, sensors.dryContact.DC2, sensors.dryContact.DC1);
		printf("\nSMOKE_A: %d, SMOKE_B: %d, Status: %s \r\n",
				sensors.smoke.pinA, sensors.smoke.pinB, sensors.smoke.status);
		printf("\nSHT20 Reading ->Temperature: %.02f \t Humidity: %.02f\r\n", sensors.sht40.temperature, sensors.sht40.humidity);
		printLineMarker('!');
#endif

		// Create Payload
		// Generate Unscheduled Payload
		TxPayload _heartBeatPayload;
		generateUnscheduledTxPayload(sensors, &_heartBeatPayload);

		sendToLora(INTERRUPT_PORT, CONFIRMED_UPLINK, _heartBeatPayload);

		TxTriggers = IDLE;

		break;
	default:
		// do nothing
		break;
	}

}

bool setLoraCredentials(void){
	if(!sendATCommand(AT_SET_DEVEUI, 2000, AT_RESPONSE_CAPTURE_OK)){
		return false;
	}
	if(!sendATCommand(AT_SET_APPEUI, 2000, AT_RESPONSE_CAPTURE_OK)){
		return false;
	}
	if(!sendATCommand(AT_SET_APPKEY, 2000, AT_RESPONSE_CAPTURE_OK)){
		return false;
	}
	if(!sendATCommand(AT_SET_NWKKEY, 2000, AT_RESPONSE_CAPTURE_OK)){
		return false;
	}
	if(!sendATCommand(AT_SET_APPSKEY, 2000, AT_RESPONSE_CAPTURE_OK)){
		return false;
	}
	if(!sendATCommand(AT_SET_NWKSKEY, 2000, AT_RESPONSE_CAPTURE_OK)){
		return false;
	}
	if(!sendATCommand(AT_SET_ADR, 2000, AT_RESPONSE_CAPTURE_OK)){
		return false;
	}
	if(!sendATCommand(AT_SAVE_TO_FLASH, 2000, AT_RESPONSE_CAPTURE_NVM_STORED)){
		return false;
	}
	return true;
}

bool joinNetwork(void){
	if(!sendATCommand(AT_JOIN_OTAA, 20000, AT_RESPONSE_CAPTURE_JOIN)){
		return false;
	}
	return true;
}

char* buildATCommand(const char *AT_SEND_, TxPayload payload) {
    // Calculate the total length needed for the new string
    // Each byte in payload becomes 2 hex characters, +2 for \r\n, +1 for null terminator
    int total_length = strlen(AT_SEND_) + (payload.length * 2) + 2 + 1;

    // Allocate memory for the new string
    char *result = (char *)malloc(total_length * sizeof(char));

    if (result == NULL) {
        // Handle memory allocation failure
        return NULL;
    }

    // Copy the initial string to the result
    strcpy(result, AT_SEND_);

    // Concatenate the payload as hex string
    for (int x = 0; x < payload.length; x++) {
        char temp[3]; // Buffer to hold hex representation (2 digits + null terminator)
        snprintf(temp, sizeof(temp), "%02X", payload.buffer[x]); // Convert to hex
        strcat(result, temp); // Append hex string to result
    }

    // Append \r\n at the end
    strcat(result, "\r\n");

    return result;
}


bool sendToLora(uint8_t portNumber, bool isConfirmedUplink, TxPayload payload){
	// Base command string
	const char *AT_SEND_ = "AT+SEND=";

	// Convert portNumber and isConfirmedUplink to strings
	char portNumberStr[4]; // Assuming portNumber won't exceed 3 digits
	snprintf(portNumberStr, sizeof(portNumberStr), "%d", portNumber);

	char confirmedUplinkStr[2]; // Single digit for true (1) or false (0)
	snprintf(confirmedUplinkStr, sizeof(confirmedUplinkStr), "%d", isConfirmedUplink);

	// Calculate the total length needed
	int total_length = strlen(AT_SEND_) + strlen(portNumberStr) + 1 + strlen(confirmedUplinkStr) + 1 + (payload.length * 2) + 2 + 1;

	// Allocate memory for the full command string
	char *result = (char *)malloc(total_length * sizeof(char));

	if (result == NULL) {
		// Handle memory allocation failure
		return false;
	}

	// Construct the command string
	strcpy(result, AT_SEND_);
	strcat(result, portNumberStr);
	strcat(result, ":");
	strcat(result, confirmedUplinkStr);
	strcat(result, ":");

	// Concatenate the payload
	for (int x = 0; x < payload.length; x++) {
		char temp[3];
		snprintf(temp, sizeof(temp), "%02X", payload.buffer[x]);
		strcat(result, temp);
	}

	// Append \r\n at the end
	strcat(result, "\r\n");

	// Send the command
	bool commandSent = sendATCommand(result, 2000, AT_RESPONSE_CAPTURE_SEND_OK);

	// Free the allocated memory
	free(result);

	return commandSent;

}

bool sendATCommand(char *command, uint32_t responseWaitTime, LPUARTState _lpuartState) {

    lpuartState = _lpuartState;
    //bufferIndex = 0;
    responseReceived = false;
    //memset(responseBuffer, 0, MAX_UART_BUFFER_SIZE);
    printf(command);

    // Wait for response or timeout
    uint32_t startTick = HAL_GetTick();
    while ((HAL_GetTick() - startTick) < responseWaitTime) {
        if (responseReceived) {
            return true; // "OK" response received
        }
    }

    lpuartState = UART_IDLE;

    return false; // Timeout occurred without receiving "OK"
}

void readSHT40(SHT40 *_sht40){
	HAL_StatusTypeDef ret = SHT40_Measure(&hi2c1, &sht40, MED_PRECISION);

	if(ret == HAL_ERROR){
		_sht40->temperature  = 0;
		_sht40->humidity     = 0;
	}else{
		_sht40->temperature = sht40.temperature;
		_sht40->humidity    = sht40.rel_humidity;
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
