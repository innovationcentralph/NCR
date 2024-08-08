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

uint32_t shtReadMillis = 0;
uint32_t sensorsReadMillis = 0;

uint8_t getMeterDataCmd[] = GET_METER_CMD;

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
void scanI2CDevices(void);
void printLineMarker(char marker); // for debugging
void handleInterruptTriggers(UnscheduledTxTriggers trigger);

bool generatePayload(void **inputs, DataType *types, uint8_t itemCount, MessageType msgType, TxPayload *payload);

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
  SHT2x_Init(&hi2c1);
  SHT2x_SetResolution(RES_14_12);

  // Initialize Modbus
  initModbus(&huart1, MODBUS_EN_GPIO_Port, MODBUS_EN_Pin);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)(ModbusResp.buffer + ModbusResp.rxIndex), 1);

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

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Check for Unscheduled Transmission Requests
	  handleInterruptTriggers(TxTriggers);

  	  // Check Temperature Reading Every X Interval
  	  if(HAL_GetTick() - shtReadMillis > SHT_READ_INTERVAL){
  		  sensors.sht20.temperature = SHT2x_GetTemperature(1);
  		  sensors.sht20.humidity = SHT2x_GetRelativeHumidity(1);

#ifdef SERIAL_DEBUG_SHT
  		printLineMarker('*');
  		printf("SHT20 Reading ->Temperature: %.02f \t Humidity: %.02f\r\n", sensors.sht20.temperature, sensors.sht20.humidity);
  		printLineMarker('*');
#endif
  		/// @TODO: Insert Threshold Control here for Unscheduled TX

  		shtReadMillis = HAL_GetTick();
  	  }

  	  // Read All Sensors every Y Interval
  	  if(HAL_GetTick() - sensorsReadMillis > DEVICE_HEARTBEAL){

  		// Read SHT20
  		sensors.sht20.temperature = SHT2x_GetTemperature(1);
	    sensors.sht20.humidity = SHT2x_GetRelativeHumidity(1);

	    // Read Smoke Sensor
	    sensors.smoke = ReadSmokeStatus();

	    // Read DryContacts
	    sensors.dryContact = MCP23008_ReadInputs();

	    // Read ModBus Device

	    sendRaw(getMeterDataCmd, GetMeterData_LEN, &ModbusResp);
	    HAL_Delay(2000); // Give time to receive response

#ifdef SCAN_I2C_DEVICES
  		scanI2CDevices();
#endif

#ifdef SERIAL_DEBUG_SENSORS
	    printLineMarker('-');
	    printf("SHT20 Reading ->Temperature: %.02f \t Humidity: %.02f\r\n", sensors.sht20.temperature, sensors.sht20.humidity);
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

}

void WDTReset(void){
	HAL_GPIO_WritePin(WDT_DONE_GPIO_Port, WDT_DONE_Pin, GPIO_PIN_SET);
	HAL_Delay(300);
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

bool generateUnscheduledTxPayload(Sensors sensors){

	TxPayload payload;

	MessageType msgType = UNSCHEDULED_TRANSMISSION;

	void *inputs[] = { &sensors.sht20.temperature, &sensors.sht20.humidity, &sensors.dryContact.value, &sensors.smoke.level };
	DataType types[] = { TYPE_FLOAT, TYPE_FLOAT, TYPE_UINT8, TYPE_UINT8 };

	bool ret = generatePayload(inputs, types, sizeof(inputs) / sizeof(void*), msgType, &payload);

#ifdef SERIAL_DEBUG_PAYLOADCHECK
	if(ret){
		printf("UNSCHEDULED PAYLOAD: ");
		for (uint8_t i = 0; i < payload.length; ++i) {
			printf("%02X ", payload.buffer[i]);
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
	__HAL_GPIO_EXTI_CLEAR_IT(SMOKE_A_Pin);
	__HAL_GPIO_EXTI_CLEAR_IT(SMOKE_B_Pin);
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);


	DryContactStatus dryContact;
	SmokeStatus smokeStatus;
	switch(trigger){
	case IDLE:
		// do nothing
		break;
	case DRY_CONTACT:
	case SMOKE_SENSOR:
	case SHT:
		smokeStatus = ReadSmokeStatus();
		sensors.sht20.temperature = SHT2x_GetTemperature(1);
	    sensors.sht20.humidity = SHT2x_GetRelativeHumidity(1);

	    HAL_Delay(100); // Give time for the pin signal to settle down
	    dryContact = MCP23008_ReadInputs();


#ifdef SERIAL_DEBUG_INTERRUPT
		printLineMarker('!');
		printf("DRY CONTACT: %d %d %d %d %d %d %d %d \r\n",
				 dryContact.DC1, dryContact.DC2, dryContact.DC3, dryContact.DC4,
				 dryContact.DC5, dryContact.DC6, dryContact.DC7, dryContact.DC8);
		printf("SMOKE_A: %d, SMOKE_B: %d, Status: %s \r\n",
				 smokeStatus.pinA, smokeStatus.pinB, smokeStatus.status);
		printf("SHT20 Reading ->Temperature: %.02f \t Humidity: %.02f\r\n", sensors.sht20.temperature, sensors.sht20.humidity);
		printLineMarker('!');
#endif

		// Create Payload
		generateUnscheduledTxPayload(sensors);

		break;
	default:
		// do nothing
		break;
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
