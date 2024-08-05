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
#define STM32WB

#define MCP23008_ADDR 0x20

// MCP23008 Register Addresses
#define MCP23008_IODIR    0x00  // I/O Direction Register
#define MCP23008_GPIO     0x09  // GPIO Register
#define MCP23008_GPINTEN  0x02  // GPIO Interrupt Enable Register
#define MCP23008_DEFVAL   0x03  // Default Compare Register
#define MCP23008_INTCON   0x04 // Interrupt Control Register
#define MCP23008_INTCAP   0x08 // Interrupt Capture Register
#define MCP23008_IOCON    0x05  // IO Expander Control Register


#define    BOOT_TIME            20 //ms
#define SENSOR_BUS hi2c1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef struct {
    bool DC1;
    bool DC2;
    bool DC3;
    bool DC4;
    bool DC5;
    bool DC6;
    bool DC7;
    bool DC8;
} DryContactStatus;

typedef struct {
    bool pinA;
    bool pinB;
    const char* status;
} SmokeStatus;


static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

ModBus_t ModbusResp;
uint8_t modbus_buffer[200];

uint8_t charRx;
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


void MCP23008_Init(void);
DryContactStatus MCP23008_ReadInputs(void);
void MCP23008_ConfigureInterrupts(void);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);

SmokeStatus ReadSmokeStatus(void);

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);


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

  SHT2x_Init(&hi2c1);
  SHT2x_SetResolution(RES_14_12);


  initModbus(&huart1, MODBUS_EN_GPIO_Port, MODBUS_EN_Pin);

  HAL_UART_Receive_IT(&huart1, &charRx, 1);


  // HAL_UART_Receive_IT(&huart1, (uint8_t *)(ModbusResp.buffer + ModbusResp.rxIndex), 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("NCR Test Code \r\n ");
  printf("Firmware Version 1.0.5 \r\n ");

//  printf("Decimals: %d %ld\n", 1977, 650000L);
//  printf("Preceding with blanks: %10d\n", 1977);
//  printf("Preceding with zeros: %010d\n", 1977);
//  printf("Some different radices: %d %x %o %#x %#o\n", 100, 100, 100, 100, 100);
//  printf("floats: %4.2f %+.0e %E\n", 3.1416, 3.1416, 3.1416);
//  printf("Width trick: %*d\n", 5, 10);
//  printf("%s\n", "A string");

  // I2C Scanner

//  printf("Scanning I2C bus...\r\n");
//
//  HAL_StatusTypeDef result;
//  uint8_t i;
//  for (i = 1; i < 128; i++)
//  {
//	  /*
//	   * The HAL_I2C_IsDeviceReady function checks if the device with the address `i << 1` is ready.
//	   * The address is shifted left by 1 because the 7-bit address is left-aligned in the 8-bit register.
//	   */
//	  result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 5);
//	  if (result == HAL_OK)
//	  {
//		  printf("I2C device found at address 0x%02X\r\n", i);
//	  }
//  }
//  printf("I2C scanning complete.\r\n");

//
//  /* Initialize mems driver interface */
//    stmdev_ctx_t dev_ctx;
//    dev_ctx.write_reg = platform_write;
//    dev_ctx.read_reg = platform_read;
//    dev_ctx.mdelay = platform_delay;
//    dev_ctx.handle = &SENSOR_BUS;
//    /* Initialize platform specific hardware */
//    platform_init();
//    /* Wait sensor boot time */
//    //platform_delay(BOOT_TIME);
//    /* Check device ID */
//    lis2dw12_device_id_get(&dev_ctx, &whoamI);
//
//    if (whoamI != LIS2DW12_ID)
//      while (1) {
//        /* manage here device not found */
//      }
//
//    /* Restore default configuration */
//    lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);
//
//    do {
//      lis2dw12_reset_get(&dev_ctx, &rst);
//    } while (rst);
//
//    /* Enable Block Data Update */
//    lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
//    /* Set full scale */
//    //lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_8g);
//    lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);
//    /* Configure filtering chain
//     * Accelerometer - filter path / bandwidth
//     */
//    lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);
//    lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);
//    /* Configure power mode */
//    lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
//    //lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit);
//    /* Set Output Data Rate */
//    lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_25Hz);
//

  	uint8_t addr;

  	for (addr = 0; addr < 128; addr++) {
  	  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 5, 5000); // Adjust timeout as needed

  	  if (status == HAL_OK) {
  		printf(" ---- > Device found at address 0x%02X\n", addr);
  	  }
  	  else{
  	    //printf("No device found at address 0x%02X\n", addr);
  	  }
  	}

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  uint8_t reg;
//	  /* Read output only if new value is available */
//	  lis2dw12_flag_data_ready_get(&dev_ctx, &reg);
//
//	  if (reg) {
//		/* Read acceleration data */
//		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
//		lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
//		//acceleration_mg[0] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[0]);
//		//acceleration_mg[1] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[1]);
//		//acceleration_mg[2] = lis2dw12_from_fs8_lp1_to_mg(data_raw_acceleration[2]);
//		acceleration_mg[0] = lis2dw12_from_fs2_to_mg(
//							   data_raw_acceleration[0]);
//		acceleration_mg[1] = lis2dw12_from_fs2_to_mg(
//							   data_raw_acceleration[1]);
//		acceleration_mg[2] = lis2dw12_from_fs2_to_mg(
//							   data_raw_acceleration[2]);
//		sprintf((char *)tx_buffer,
//				"Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
//				acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
//		tx_com(tx_buffer, strlen((char const *)tx_buffer));
//	  }
//	  // Read GPIO

	uint8_t addr;

	for (addr = 0; addr < 128; addr++) {
	  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 5, 5000); // Adjust timeout as needed

	  if (status == HAL_OK) {
		printf(" ---- > Device found at address 0x%02X\n", addr);
	  }
	  else{
	    // printf("No device found at address 0x%02X\n", addr);
	  }
	}

////	unsigned char buffer[100] = { 0 };
	/* Gets current temperature & relative humidity. */
	float cel = SHT2x_GetTemperature(1);
	/* Converts temperature to degrees Fahrenheit and Kelvin */
	float fah = SHT2x_CelsiusToFahrenheit(cel);
	float kel = SHT2x_CelsiusToKelvin(cel);
	float rh = SHT2x_GetRelativeHumidity(1);
	/* May show warning below. Ignore and proceed. */
	printf("%02fºC, %02fºF, %02f RH\n", cel, fah, rh);

	uint8_t cmdRaw[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
//
//	// Clear response buffer and reset index
//	memset(ModbusResp.buffer, '\0', sizeof(ModbusResp.buffer));
//	ModbusResp.rxIndex = 0;
//
//	// Enable MODBUS_EN
//	HAL_GPIO_WritePin(MODBUS_EN.port, MODBUS_EN.pin, GPIO_PIN_SET);
//
//		// Receive data using UART interrupt
//	HAL_UART_Receive_IT(&huart1, &charRx, 1);
//
//
//		// Transmit the raw data
//	HAL_UART_Transmit(&huart1, cmdRaw, 5, HAL_MAX_DELAY);
//
//	// Delay to ensure proper communication
//	//HAL_Delay(1);
//
//	// Disable MODBUS_EN
//	HAL_GPIO_WritePin(MODBUS_EN.port, MODBUS_EN.pin, GPIO_PIN_RESET);
//	HAL_Delay(3000);
//
	sendRaw(cmdRaw, 5, &ModbusResp);
	HAL_Delay(2000);



	printf("MODBUS RESPONSE (Hex): ");

	for (int x = 0; x < ModbusResp.rxIndex; x++){
		printf("%02X ", ModbusResp.buffer[x]);
	}
	printf(" \r\n");
	ModbusResp.rxIndex = 0;


	HAL_Delay(100);


	//HAL_UART_Receive_IT(&huart1, (uint8_t *)ModbusResp.buffer, 1);


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
  HAL_GPIO_WritePin(MODBUS_EN_GPIO_Port, MODBUS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SMOKE_B_Pin SMOKE_A_Pin */
  GPIO_InitStruct.Pin = SMOKE_B_Pin|SMOKE_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MODBUS_EN_Pin */
  GPIO_InitStruct.Pin = MODBUS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MODBUS_EN_GPIO_Port, &GPIO_InitStruct);

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
      {
          DryContactStatus dryContact = MCP23008_ReadInputs();
		  printf("DRY CONTACT: %d %d %d %d %d %d %d %d \r\n",
				 dryContact.DC1, dryContact.DC2, dryContact.DC3, dryContact.DC4,
				 dryContact.DC5, dryContact.DC6, dryContact.DC7, dryContact.DC8);
          break;
      }

      case SMOKE_A_Pin:
      case SMOKE_B_Pin:
      {
    	  SmokeStatus smokeStatus = ReadSmokeStatus();
		  printf("SMOKE_A: %d, SMOKE_B: %d, Status: %s \r\n",
				 smokeStatus.pinA, smokeStatus.pinB, smokeStatus.status);

          break;
      }

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
    } else if (!smokeStatus.pinA && smokeStatus.pinB) {
        smokeStatus.status = "Light Pollution";
    } else if (smokeStatus.pinA && !smokeStatus.pinB) {
        smokeStatus.status = "Moderate Pollution";
    } else if (smokeStatus.pinA && smokeStatus.pinB) {
        smokeStatus.status = "Severe Pollution";
    } else {
        smokeStatus.status = "Unknown"; // Fallback case, should not occur
    }

    return smokeStatus;
}



/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{

	HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_L, reg,
	                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);

#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  LIS2DW12_I2C_ADD_H & 0xFE, reg, (uint8_t*) bufp, len);
#endif
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_L, reg,
	                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

#if defined(NUCLEO_F401RE)
  HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, LIS2DW12_I2C_ADD_H & 0xFE, reg, bufp, len);
#endif
  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
	HAL_UART_Transmit(&hlpuart1, tx_buffer, len, 1000);
#if defined(NUCLEO_F401RE)
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
#elif defined(STEVAL_MKI109V3)
  CDC_Transmit_FS(tx_buffer, len);
#elif defined(SPC584B_DIS)
  sd_lld_write(&SD2, tx_buffer, len);
#endif
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
#if defined(NUCLEO_F401RE) | defined(STEVAL_MKI109V3) | defined(SWT32WB)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	  if (huart->Instance == USART1)
	  {
		  Modbus_RxCallback(&ModbusResp);
		  HAL_UART_Receive_IT(&huart1, (uint8_t *)(ModbusResp.buffer + ModbusResp.rxIndex), 1);
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
