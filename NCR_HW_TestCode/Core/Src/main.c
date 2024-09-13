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
#include <sht40.h>
#include "ProjectConfig.h"
#include "stdlib.h"
#include "ads1115.h"
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

IWDG_HandleTypeDef hiwdg;

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
uint32_t wdtResetMillis = 0;
uint32_t mcuResetMillis = 0;
uint32_t payloadQueueMilis = 0;
SHT40_Measurement sht40;

DryContactStatus dryContact;

uint8_t getMeterDataCmd1[] = GET_METER_ERG_CMD;
uint8_t getMeterDataCmd2[] = GET_METER_BASIC_CMD;

uint8_t setMeterOff[]     = METER_CMD_ON;
uint8_t setMeterOn[]      = METER_CMD_OFF;
uint8_t setMeterPrepaid[] = METER_CMD_PREPAID;

// LPUART 1 Variables
static bool responseReceived = false;
static char responseBuffer[100];
static char downlinkBuffer[100];
static uint16_t bufferIndex = 0;
bool isConfirmedUplinkReceived = false;
bool isDownlinkReceived = false;
uint8_t downlinkBufferLen = 0;
bool isWarmedUp = false;
uint32_t warmUpMillis = 0;
uint32_t acrelReadMillis = 0;

// Accelerometer related
//static axis3bit16_t data_raw_acceleration[SELF_TEST_SAMPLES];
//static float acceleration_mg[SELF_TEST_SAMPLES][3];
static uint8_t whoamI, rst;
//static uint8_t tx_buffer[1000];
stmdev_ctx_t dev_ctx;
lis2dw12_reg_t int_route;

bool isTapDetected = false;
bool isMovementDetected = false;
uint32_t initTapMillis = 0;

uint8_t prev_dryContact    = 0;
uint8_t current_dryContact = 0;
uint8_t prev_smoke         = 0;
uint8_t current_smoke      = 0;
LeakState current_leak = 0;
LeakState prev_leak = DRY;

TxPayloadQueue payLoadQueue;

float adcRead;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
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
void readLTC4015(LTCStatus *ltc4015);
bool generatePowerTxPayload(Sensors sensors, TxPayload *payload);
void appendModbusToPayload(TxPayload *payload, ModBus_t *modbus);
void mockModbusResponse(ModBus_t *modbus, uint8_t *data, uint16_t length);
void queueUnscheduledPayload(void);
void queueHeartbeatPayload(void);
void queuePayload(TxPayload Payload);

void readWaterLeak(WaterLeak *leak);


//Lora FPF
bool setLoraCredentials(void);
bool sendATCommand(char *command, uint32_t responseWaitTime, LPUARTState _lpuartState);
bool sendToLora(uint8_t portNumber, bool isConfirmedUplink, TxPayload payload);
bool joinNetwork(void);
bool generatePayload(void **inputs, DataType *types, uint8_t itemCount, MessageType msgType, TxPayload *payload);
bool generateHeartbeatTxPayload(Sensors sensors, TxPayload *payload);
bool generatePowerTxPayload(Sensors sensors, TxPayload *payload);
bool generateUnscheduledTxPayload(Sensors sensors, TxPayload *payload);

DryContactStatus MCP23008_ReadInputs(void);
SmokeStatus ReadSmokeStatus(void);


// Accelerometer PFP
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
//static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);
bool initAccelerometer(void);
void readAccelerometer(Accel *_accel);

void initQueue(TxPayloadQueue *q);
int enqueue(TxPayloadQueue *q, TxPayload element);
int dequeue(TxPayloadQueue *q, TxPayload *element);
int isQueueEmpty(TxPayloadQueue *q);
int isQueueFull(TxPayloadQueue *q);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline float ABSF(float _x)
{
  return (_x < 0.0f) ? -(_x) : _x;
}

//static int flush_samples(stmdev_ctx_t *dev_ctx)
//{
//  lis2dw12_reg_t reg;
//  axis3bit16_t dummy;
//  int samples = 0;
//  /*
//   * Discard old samples
//   */
//  lis2dw12_status_reg_get(dev_ctx, &reg.status);
//
//  if (reg.status.drdy) {
//    lis2dw12_acceleration_raw_get(dev_ctx, dummy.i16bit);
//    samples++;
//  }
//
//  return samples;
//}
//
//static void test_self_test_lis2dw12(stmdev_ctx_t *dev_ctx)
//{
//  lis2dw12_reg_t reg;
//  float media[3] = { 0.0f, 0.0f, 0.0f };
//  float mediast[3] = { 0.0f, 0.0f, 0.0f };
//  uint8_t match[3] = { 0, 0, 0 };
//  uint8_t j = 0;
//  uint16_t i = 0;
//  uint8_t k = 0;
//  uint8_t axis;
//  /* Restore default configuration */
//  lis2dw12_reset_set(dev_ctx, PROPERTY_ENABLE);
//
//  do {
//    lis2dw12_reset_get(dev_ctx, &rst);
//  } while (rst);
//
//  lis2dw12_block_data_update_set(dev_ctx, PROPERTY_ENABLE);
//  lis2dw12_full_scale_set(dev_ctx, LIS2DW12_4g);
//  lis2dw12_power_mode_set(dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
//  lis2dw12_data_rate_set(dev_ctx, LIS2DW12_XL_ODR_50Hz);
//  HAL_Delay(100);
//  /* Flush old samples */
//  flush_samples(dev_ctx);
//
//  do {
//    lis2dw12_status_reg_get(dev_ctx, &reg.status);
//
//    if (reg.status.drdy) {
//      /* Read accelerometer data */
//      memset(data_raw_acceleration[i].i16bit, 0x00, 3 * sizeof(int16_t));
//      lis2dw12_acceleration_raw_get(dev_ctx,
//                                    data_raw_acceleration[i].i16bit);
//
//      for (axis = 0; axis < 3; axis++) {
//        acceleration_mg[i][axis] =
//          lis2dw12_from_fs4_to_mg(data_raw_acceleration[i].i16bit[axis]);
//      }
//
//      i++;
//    }
//  } while (i < SELF_TEST_SAMPLES);
//
//  for (k = 0; k < 3; k++) {
//    for (j = 0; j < SELF_TEST_SAMPLES; j++) {
//      media[k] += acceleration_mg[j][k];
//    }
//
//    media[k] = (media[k] / j);
//  }
//
//  /* Enable self test mode */
//  lis2dw12_self_test_set(dev_ctx, LIS2DW12_XL_ST_POSITIVE);
//  HAL_Delay(100);
//  i = 0;
//  /* Flush old samples */
//  flush_samples(dev_ctx);
//
//  do {
//    lis2dw12_status_reg_get(dev_ctx, &reg.status);
//
//    if (reg.status.drdy) {
//      /* Read accelerometer data */
//      memset(data_raw_acceleration[i].i16bit, 0x00, 3 * sizeof(int16_t));
//      lis2dw12_acceleration_raw_get(dev_ctx,
//                                    data_raw_acceleration[i].i16bit);
//
//      for (axis = 0; axis < 3; axis++)
//        acceleration_mg[i][axis] =
//          lis2dw12_from_fs4_to_mg(data_raw_acceleration[i].i16bit[axis]);
//
//      i++;
//    }
//  } while (i < SELF_TEST_SAMPLES);
//
//  for (k = 0; k < 3; k++) {
//    for (j = 0; j < SELF_TEST_SAMPLES; j++) {
//      mediast[k] += acceleration_mg[j][k];
//    }
//
//    mediast[k] = (mediast[k] / j);
//  }
//
//  /* Check for all axis self test value range */
//  for (k = 0; k < 3; k++) {
//    if ((ABSF(mediast[k] - media[k]) >= ST_MIN_POS) &&
//        (ABSF(mediast[k] - media[k]) <= ST_MAX_POS)) {
//      match[k] = 1;
//    }
//
//    sprintf((char *)tx_buffer, "%d: |%f| <= |%f| <= |%f| %s\r\n", k,
//            ST_MIN_POS, ABSF(mediast[k] - media[k]), ST_MAX_POS,
//            match[k] == 1 ? "PASSED" : "FAILED");
//    tx_com(tx_buffer, strlen((char const *)tx_buffer));
//  }
//
//  /* Disable self test mode */
//  lis2dw12_data_rate_set(dev_ctx, LIS2DW12_XL_ODR_OFF);
//  lis2dw12_self_test_set(dev_ctx, LIS2DW12_XL_ST_DISABLE);
//}

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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // Initialize MCP23008
  MCP23008_Init();

  // Configure interrupts on MCP23008
  MCP23008_ConfigureInterrupts();

  // Initialize SHT20 Sensor
  uint32_t sht40_serial;
  if( SHT40_ReadSerial(&hi2c1, &sht40_serial) != HAL_ERROR ) {
	  	sensors.sht40.alarmState.temperature = NORMAL;
	  	sensors.sht40.alarmState.humidity = NORMAL;
	  	sensors.sht40.thresholds.temp_high = TEMP_HIGH;
	  	sensors.sht40.thresholds.temp_low  = TEMP_LOW;
	  	sensors.sht40.thresholds.temp_hys  = TEMP_HYS;
	  	sensors.sht40.thresholds.rel_high  = RH_HIGH;
		sensors.sht40.thresholds.rel_low   = RH_LOW;
		sensors.sht40.thresholds.rel_hys   = RH_HYS;
		//printf("I2C connection established to SHT40 with serial %" PRIu32 "\r\n", sht40_serial);
  } else {
		//printf("Failed to read serial from SHT40; check connections and reset MCU\r\n");
  }


  // Initialize accelerometer
  if(initAccelerometer()){
	  //printf("Accelerometer Initialized \r\n ");
  }
  else{
	  //printf("Error Accelerometer Initialization \r\n ");
  }

  if(ADS1115_Init(&hi2c1, ADS1115_DATA_RATE_64, ADS1115_PGA_ONE) == HAL_OK){
	  HAL_Delay(1500);
	  //printf("ADC Present");
  } else{
  	  //printf("ADC Not Found");
  }



  // Initialize Modbus
  initModbus(&huart1, MODBUS_EN_GPIO_Port, MODBUS_EN_Pin);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)(ModbusResp.buffer + ModbusResp.rxIndex), 1);

  // Init ST50H AT Slave Communication
  uint8_t rxBuffer;
  HAL_UART_Receive_IT(&hlpuart1, &rxBuffer, 1);
  // Initialize timers;
  shtReadMillis = HAL_GetTick();
  payloadQueueMilis  = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf(" # # # # # # # # # # -> %s\r\n ", FirmwareName);
  printf("# # # # # # # # # # -> Firmware Version %d.%d.%d\r\n ", VERSION_MAJOR, VERSION_MID, VERSION_MINOR);

#ifdef SCAN_I2C_DEVICES
  scanI2CDevices();
#endif

  WDTReset();
  HAL_Delay(2000);

  sendATCommand(AT_RFS, 2000, AT_RESPONSE_CAPTURE_OK);
  sendATCommand(AT_RFS, 2000, AT_RESPONSE_CAPTURE_OK);
  sendATCommand(AT_RFS, 2000, AT_RESPONSE_CAPTURE_OK);

  HAL_Delay(2000);
  //printf("Setting LoRa Credentials \r\n");
  if(!setLoraCredentials()){

  }else{
	  //printf("Success setting LoRa credentials \r\n");
  }

  printf("Joining to Network \r\n");

  while(hasJoinedNetwork == false){
	  joinNetwork();
	  printf("Retrying Joining Lora\r\n");
  }
  printf("Success Joining Lora \r\n");

  TxPayload initPayload;
  initPayload.buffer[0] = 0x00;
  initPayload.buffer[1] = 0x01;
  initPayload.buffer[2] = 0x02;
  initPayload.length = 3;
  initPayload.msgType = DIAGNOSTICS;

  //printf("Sending Test Lora Payload\r\n");
  sendToLora(TEST_UPLINK_PORT, CONFIRMED_UPLINK, initPayload);

//  sendToLora(TEST_UPLINK_PORT, CONFIRMED_UPLINK, initPayload);
//
//  sendToLora(TEST_UPLINK_PORT, CONFIRMED_UPLINK, initPayload);
  mcuResetMillis = HAL_GetTick();
  warmUpMillis = HAL_GetTick();
  acrelReadMillis = HAL_GetTick();


  initQueue(&payLoadQueue);


  while (1)
  {

	  // Internal IWDT Feed
	  HAL_IWDG_Refresh(&hiwdg);

	  // External WDT Feed
	  if(HAL_GetTick() - wdtResetMillis > WDT_RESET_INTERVAL){
		  WDTReset();
		  wdtResetMillis = HAL_GetTick();
	  }

	  //Do Warm up to exclude false readings
	  if(isWarmedUp == false){
		  if(HAL_GetTick() - warmUpMillis > WARM_UP_TIME){
			  isWarmedUp = true;
		  }
	  }

//	  sendRaw(setMeterOn, METER_CMD_ON_LEN, &ModbusResp);
//	  HAL_Delay(1000);
//
//	  printf("MODBUS RESPONSE (Hex): ");
//	  for (int x = 0; x < ModbusResp.rxIndex; x++) {
//			printf("%02X ", ModbusResp.buffer[x]);
//	  }
//	  printf("\r\n");
//
//	  HAL_Delay(2000);
//
//	  sendRaw(setMeterOff, METER_CMD_OFF_LEN, &ModbusResp);
//	  HAL_Delay(1000);
//
//	  printf("MODBUS RESPONSE (Hex): ");
//	  for (int x = 0; x < ModbusResp.rxIndex; x++) {
//			printf("%02X ", ModbusResp.buffer[x]);
//	  }
//	  printf("\r\n");
//
//	  HAL_Delay(2000);




	  if(isDownlinkReceived == true){
		  isDownlinkReceived = false;
		  printf("DOWNLINK MESSAGE RECEIVED \r\n");
		  printf("Received Buffer: %.*s\r\n", downlinkBufferLen, downlinkBuffer);

		  // Check for Modbus Control
		  if(strstr(downlinkBuffer, "0101") != NULL){
			  printf("Modbus Command: Turn ON \r\n ");
			  sendRaw(setMeterOn, METER_CMD_ON_LEN, &ModbusResp);
			  HAL_Delay(1000);
		  }
		  if(strstr(downlinkBuffer, "0102") != NULL){
			  printf("Modbus Command: Turn OFF \r\n ");
			  sendRaw(setMeterOff, METER_CMD_OFF_LEN, &ModbusResp);
			  HAL_Delay(1000);
		  }
		  if(strstr(downlinkBuffer, "0103") != NULL){
			  printf("Modbus Command: Prepaid \r\n ");
			  sendRaw(setMeterPrepaid, METER_CMD_PREPAID_LEN, &ModbusResp);
			  HAL_Delay(1000);
		  }
	  }
	  if(isDownlinkReceived == true){
		  isDownlinkReceived = false;
		  printf("CONFIRMED UPLINK RECEIVED \r\n");
	  }



	  // Hanging / Freezing BAND AID Solution
	  if(HAL_GetTick() - mcuResetMillis > MCU_REST_INTERVAL){
		  //printf("RESTARTING MCU NOW... \r\n");
		  HAL_Delay(1000);
		  HAL_NVIC_SystemReset();
		  mcuResetMillis = HAL_GetTick();
	  }


	  // Handle Dequeue for Payloads
	  if(HAL_GetTick() - payloadQueueMilis > QUEUE_SEND_INTERVAL){
		  TxPayload payload;
		  if (dequeue(&payLoadQueue, &payload) == 0) {
		      //printf("Sending To Lora \r\n ");

		      if(payload.msgType == UNSCHEDULED_TRANSMISSION){
		    	  sendToLora(INTERRUPT_PORT, CONFIRMED_UPLINK, payload);
		      }

		      else if(payload.msgType == HEARTBEAT){
		    	  sendToLora(HEARTBEAT_PORT, UNCONFIRMED_UPLINK, payload);
		      }
		      else if(payload.msgType == POWER_PARAMS){
				  sendToLora(POWER_PORT, UNCONFIRMED_UPLINK, payload);
			  }

		  } else {
		      //printf("NOTHING TO SEND ... \r\n");
		  }
		  payloadQueueMilis = HAL_GetTick();
	  }





	  readAccelerometer(&sensors.accel);

	//test_self_test_lis2dw12(&dev_ctx);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Check for Unscheduled Transmission Requests
	  // handleInterruptTriggers(TxTriggers);






  	  // Check "INTERUPT EVENTS"
  	  if(HAL_GetTick() - shtReadMillis > SHT_READ_INTERVAL){

  		 // Read Water Leak
  		readWaterLeak(&sensors.leak);
  		current_leak = sensors.leak.state;
  		if(current_leak != prev_leak){
  			//printf("ALERT ON LEAK \r\n");
  			prev_leak = current_leak;

  			queueUnscheduledPayload();


  		}

  		// Read Temperature
  		readSHT40(&sensors.sht40);

  		// Read Smoke Sensor
  		sensors.smoke = ReadSmokeStatus();
  		current_smoke =  sensors.smoke.level;
  		if(current_smoke != prev_smoke){
  			//printf("ALERT ON SMOKE \r\n");
  			prev_smoke = current_smoke;

  			queueUnscheduledPayload();

  		}

  		// Read DryContacts
  	    sensors.dryContact = MCP23008_ReadInputs();
  	    current_dryContact = sensors.dryContact.value;
  	    if(current_dryContact != prev_dryContact){
  	    	//printf("ALERT ON DRY CONTACT \r\n");
  	    	prev_dryContact = current_dryContact;

  	    	queueUnscheduledPayload();

  	    }


#ifdef SERIAL_DEBUG_SHT
//  		printLineMarker('*');
//  		printf("\nSHT20 Reading ->Temperature: %.02f \t Humidity: %.02f\r\n", sensors.sht40.temperature, sensors.sht40.humidity);
//  		printf("\nSmoke Level -> Level %d \r\n", sensors.smoke.level);
//		printf("\nDry Contact States: %d %d %d %d %d %d %d %d \r\n\n",
//				sensors.dryContact.DC1, sensors.dryContact.DC2,
//				sensors.dryContact.DC3, sensors.dryContact.DC4,
//				sensors.dryContact.DC5, sensors.dryContact.DC6,
//				sensors.dryContact.DC7, sensors.dryContact.DC8);
//  		printLineMarker('*');
#endif


  		shtReadMillis = HAL_GetTick();
  	  }

  	  // Read Acrel Meter every Y Interval
  	  if(HAL_GetTick() - acrelReadMillis > ACREL_READ_INTERVAL){

  		  	TxPayload _powerPayload;
  		    _powerPayload.msgType = POWER_PARAMS;
  		    _powerPayload.buffer[0] = POWER_PARAMS;
  		    _powerPayload.length = 1;

			// First Modbus Command
			sendRaw(getMeterDataCmd1, GetMeterData_LEN, &ModbusResp);
			HAL_Delay(2000);

//			  printf("MODBUS RESPONSE (Hex): ");
//			  for (int x = 0; x < ModbusResp.rxIndex; x++) {
//					printf("%02X ", ModbusResp.buffer[x]);
//			  }
//			  printf("\r\n");

			appendModbusToPayload(&_powerPayload, &ModbusResp);

			// Second Modbus Command
			sendRaw(getMeterDataCmd2, GetMeterData_LEN, &ModbusResp);
			HAL_Delay(2000);

//			  printf("MODBUS RESPONSE (Hex): ");
//			  for (int x = 0; x < ModbusResp.rxIndex; x++) {
//					printf("%02X ", ModbusResp.buffer[x]);
//			  }
//			  printf("\r\n");

			appendModbusToPayload(&_powerPayload, &ModbusResp);

			queuePayload(_powerPayload);

			acrelReadMillis = HAL_GetTick();
  	  }



  	  // Read All Sensors every Y Interval - HEART BEAT
  	  if(HAL_GetTick() - sensorsReadMillis > DEVICE_HEARTBEAT){

  		// Read Water Leak
  		readWaterLeak(&sensors.leak);


  		// Read SHT20
  		readSHT40(&sensors.sht40);

	    // Read Smoke Sensor
	    sensors.smoke = ReadSmokeStatus();

	    // Read DryContacts
	    sensors.dryContact = MCP23008_ReadInputs();

	    // PLaceholder for LTC4015
	    readLTC4015(&sensors.ltc4015);


	    queueHeartbeatPayload();


//	    TxPayload _powerPayload;
//	    generatePowerTxPayload(sensors, &_powerPayload);
//
//
//	    // First Modbus Command
//	    sendRaw(getMeterDataCmd1, GetMeterData_LEN, &ModbusResp);
//	    HAL_Delay(2000);
//
//	    // Mock first Modbus Response
////	    uint8_t mockData1[] = {0x01, 0x03, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x64, 0x17};
////	    mockModbusResponse(&ModbusResp, mockData1, sizeof(mockData1));
//	    appendModbusToPayload(&_powerPayload, &ModbusResp);
//
//
//	    // Second Modbus Command
//	    sendRaw(getMeterDataCmd2, GetMeterData_LEN, &ModbusResp);
//	    HAL_Delay(2000);
//
//
//	    // Mock second Modbus Response
////	    uint8_t mockData2[] = {0x01, 0x03, 0x0E, 0x08, 0xD2, 0x00, 0x12, 0x00, 0x15, 0x00, 0x00, 0x00, 0x15, 0x03, 0xE8, 0x17, 0x61, 0x30, 0x4D};
////	    mockModbusResponse(&ModbusResp, mockData2, sizeof(mockData2));
//	    appendModbusToPayload(&_powerPayload, &ModbusResp);

//		sendToLora(POWER_PORT, CONFIRMED_UPLINK, _powerPayload);

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

//	    // Generate HEARTBEAT Payload
//	    TxPayload _heartBeatPayload;
//	    generateHeartbeatTxPayload(sensors, &_heartBeatPayload);
//
//	    sendToLora(HEARTBEAT_PORT, CONFIRMED_UPLINK, _heartBeatPayload);

  		sensorsReadMillis = HAL_GetTick();
  	  }



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
			  mcuResetMillis = HAL_GetTick();
			  // Check if the buffer is full or if the received character is \r or \n
			  if (receivedData == '\r' || receivedData == '\n' || bufferIndex >= MAX_UART_BUFFER_SIZE) {
				  //responseBuffer[bufferIndex] = '\r'; // Null-terminate the string
				  //printf("Received: %s\n", responseBuffer);
				  switch(lpuartState){
				  case AT_RESPONSE_CAPTURE_OK:
					  if (strstr(responseBuffer, "OK") != NULL) {
						  //printf("OK RECEIVED\r\n");
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
				  {
					  if (strstr(responseBuffer, "EVT:JOINED") != NULL) {
						  responseReceived = true;
						  hasJoinedNetwork = true;
						  //printf("EVENT JOINED\r\n");
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
				  }
				  case AT_RESPONSE_CAPTURE_SEND_OK:
					  if (strstr(responseBuffer, "NO_NETWORK_JOINED") != NULL) {
						 bufferIndex = 0;
					  }
					  else if (strstr(responseBuffer, "OK") != NULL) {
						  bufferIndex = 0;
					  }
					  else if (strstr(responseBuffer, "+EVT:5:") != NULL) {
						  isDownlinkReceived = true;
						  downlinkBufferLen = bufferIndex;
						  memcpy(downlinkBuffer, responseBuffer, bufferIndex);
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
	HAL_Delay(1);
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
        default:
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

	void *inputs[] = {&sensors.sht40.temperature, &sensors.sht40.humidity, &sensors.dryContact.value, &sensors.smoke.level, &sensors.leak.state, &sensors.accel.status};
	DataType types[] = { TYPE_FLOAT, TYPE_FLOAT, TYPE_UINT8, TYPE_UINT8, TYPE_UINT8, TYPE_UINT8};

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
					  &sensors.smoke.level, &sensors.ltc4015.VIN, &sensors.ltc4015.VBAT, &sensors.ltc4015.VSYS, &sensors.leak.state, &sensors.accel.status};

	DataType types[] = { TYPE_FLOAT, TYPE_FLOAT, TYPE_UINT8, TYPE_UINT8, TYPE_FLOAT, TYPE_FLOAT, TYPE_FLOAT, TYPE_UINT8, TYPE_UINT8};

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

bool generatePowerTxPayload(Sensors sensors, TxPayload *payload){
	MessageType msgType = POWER_PARAMS;

	void *inputs[] = {
			&sensors.ltc4015.VIN,
			&sensors.ltc4015.VBAT,
			&sensors.ltc4015.VSYS,
	};

	DataType types[] = {
			TYPE_FLOAT,
			TYPE_FLOAT,
			TYPE_FLOAT,
	};

	bool ret = generatePayload(inputs, types, sizeof(inputs) / sizeof(void*), msgType, payload);


#ifdef SERIAL_DEBUG_PAYLOADCHECK
	if(ret){
		printf("POWER PAYLOAD: ");
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
		HAL_NVIC_SystemReset();
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
	bool commandSent = sendATCommand(result, 4000, AT_RESPONSE_CAPTURE_SEND_OK);

	// Free the allocated memory
	free(result);

	return commandSent;

}

bool sendATCommand(char *command, uint32_t responseWaitTime, LPUARTState _lpuartState) {

    lpuartState = _lpuartState;
    //bufferIndex = 0;
    responseReceived = false;
    printf(command);

    // Wait for response or timeout
    uint32_t startTick = HAL_GetTick();
    while ((HAL_GetTick() - startTick) < responseWaitTime) {
    	// Internal IWDT Feed
        HAL_IWDG_Refresh(&hiwdg);
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

		// check for threshold breach


		if(_sht40->temperature > _sht40->thresholds.temp_high + _sht40->thresholds.temp_hys){
			//Anti Spam
			if(_sht40->alarmState.temperature != ABOVE_THRESHOLD){
				_sht40->alarmState.temperature = ABOVE_THRESHOLD;
				//printf("Threshold breach! -> Temperature High \r\n");
				queueUnscheduledPayload();

			}
		}

		if(_sht40->temperature < _sht40->thresholds.temp_low - _sht40->thresholds.temp_hys){
			//Anti Spam
			if(_sht40->alarmState.temperature != BELOW_THRESHOLD){
				_sht40->alarmState.temperature = BELOW_THRESHOLD;
				//printf("Threshold breach! -> Temperature Low \r\n");
				queueUnscheduledPayload();
			}
		}

		if(_sht40->temperature > (_sht40->thresholds.temp_low + _sht40->thresholds.temp_hys)
			&& _sht40->temperature < (_sht40->thresholds.temp_high - _sht40->thresholds.temp_hys)){
			//Anti Spam
			if(_sht40->alarmState.temperature != NORMAL){
				_sht40->alarmState.temperature = NORMAL;
				//printf("Temperature Now Normal \r\n");
				queueUnscheduledPayload();
			}
		}

		if(_sht40->humidity > _sht40->thresholds.rel_high + _sht40->thresholds.rel_hys){
			//Anti Spam
			if(_sht40->alarmState.humidity != ABOVE_THRESHOLD){
				_sht40->alarmState.humidity = ABOVE_THRESHOLD;
				//printf("Threshold breach! -> Humidity High \r\n");
				queueUnscheduledPayload();
			}
		}

		if(_sht40->humidity < _sht40->thresholds.rel_low - _sht40->thresholds.rel_hys){
			//Anti Spam
			if(_sht40->alarmState.humidity != BELOW_THRESHOLD){
				_sht40->alarmState.humidity = BELOW_THRESHOLD;
				//printf("Threshold breach! -> Humidity Low \r\n");
				queueUnscheduledPayload();
			}
		}

		if(_sht40->humidity > (_sht40->thresholds.rel_low + _sht40->thresholds.rel_hys)
			&& _sht40->humidity < (_sht40->thresholds.rel_high - _sht40->thresholds.rel_hys)){
			//Anti Spam
			if(_sht40->alarmState.humidity != NORMAL){
				_sht40->alarmState.humidity = NORMAL;
				//printf("Humidity Now Normal \r\n");
				queueUnscheduledPayload();
			}
		}


	}




}


static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
	HAL_I2C_Mem_Write(handle, LIS2DW12_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
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
	HAL_I2C_Mem_Read(handle, LIS2DW12_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

  return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
//static void tx_com(uint8_t *tx_buffer, uint16_t len)
//{
//	HAL_UART_Transmit(&hlpuart1, tx_buffer, len, 1000);
//}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
	HAL_Delay(ms);

}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{

}

bool initAccelerometer(void){
	// Initialize Accelerometer
	/* Initialize mems driver interface */

	  dev_ctx.write_reg = platform_write;
	  dev_ctx.read_reg = platform_read;
	  dev_ctx.mdelay = platform_delay;
	  dev_ctx.handle = &hi2c1;
	  /* Initialize platform specific hardware */
	  platform_init();
	  /* Wait sensor boot time */
	  platform_delay(BOOT_TIME);
	  /* Check device ID */
	  lis2dw12_device_id_get(&dev_ctx, &whoamI);

	  if (whoamI != LIS2DW12_ID){
		  return false;
	  }

	  /* Restore default configuration */
	   lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);

	   do {
		 lis2dw12_reset_get(&dev_ctx, &rst);
	   } while (rst);


     /* Set full scale */
	 lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);
	 /* Configure filtering chain
	  * Accelerometer - filter path / bandwidth
	  */
	 lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);
	 lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);
	 /* Configure power mode */
	 lis2dw12_power_mode_set(&dev_ctx,
							 LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit);

//	 /* Enable Tap detection on X, Y, Z */
//	  lis2dw12_tap_detection_on_z_set(&dev_ctx, PROPERTY_ENABLE);
//	  lis2dw12_tap_detection_on_y_set(&dev_ctx, PROPERTY_ENABLE);
//	  lis2dw12_tap_detection_on_x_set(&dev_ctx, PROPERTY_ENABLE);
//	  /* Set Tap threshold on all axis */
//	  lis2dw12_tap_threshold_x_set(&dev_ctx, 15);
//	  lis2dw12_tap_threshold_y_set(&dev_ctx, 15);
//	  lis2dw12_tap_threshold_z_set(&dev_ctx, 15);
//
//	  /* Configure Single Tap parameter */
//	    lis2dw12_tap_quiet_set(&dev_ctx, 1);
//	    lis2dw12_tap_shock_set(&dev_ctx, 3);
//	    /* Enable Single Tap detection only */
//	    lis2dw12_tap_mode_set(&dev_ctx, LIS2DW12_ONLY_SINGLE);
//	    /* Enable single tap detection interrupt */
//	    lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
//	    int_route.ctrl4_int1_pad_ctrl.int1_single_tap = PROPERTY_ENABLE;
//	    lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

	 /* Set wake-up duration
	  * Wake up duration event 1LSb = 1 / ODR
	  */
	 lis2dw12_wkup_dur_set(&dev_ctx, 2);
	 /* Set sleep duration
	  * Duration to go in sleep mode (1 LSb = 512 / ODR)
	  */
	 lis2dw12_act_sleep_dur_set(&dev_ctx, 3);
	 /* Set Activity wake-up threshold
	  * Threshold for wake-up 1 LSB = FS_XL / 64
	  */
	 lis2dw12_wkup_threshold_set(&dev_ctx, 1);
	 /* Data sent to wake-up interrupt function */
	 lis2dw12_wkup_feed_data_set(&dev_ctx, LIS2DW12_HP_FEED);
	 /* Config activity / inactivity or stationary / motion detection */
	 lis2dw12_act_mode_set(&dev_ctx, LIS2DW12_DETECT_ACT_INACT);
	 /* Enable activity detection interrupt */
	 lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
	 int_route.ctrl4_int1_pad_ctrl.int1_wu = PROPERTY_ENABLE;
	 lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
	 /* Set Output Data Rate */
	 lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_200Hz);

	 //init sensors struct
	 sensors.accel.status = STATIONARY;

	 return true;
}

void readAccelerometer(Accel *_accel){
	lis2dw12_all_sources_t all_source;
	  /* Read status register */
	  lis2dw12_all_sources_get(&dev_ctx, &all_source);


	  /* Check if Activity/Inactivity events */
	  if (all_source.wake_up_src.sleep_state_ia) {
		  //Inactivity
		  // Reset after sending Interrupt
//		  if(isTapDetected || isMovementDetected){
		      _accel->status = STATIONARY;
			  isTapDetected = false;
			  isMovementDetected = false;
//			  printf("Resetting flags \r\n ");
//		  }
	  }

	  if (all_source.wake_up_src.wu_ia) {

		// Activity
		if(isTapDetected == false){
			isTapDetected = true;
			_accel->status = SMASHED;
			//printf("Smash Detected \nX: %d \nY: %d \nZ: %d\r\n ", all_source.wake_up_src.x_wu, all_source.wake_up_src.y_wu, all_source.wake_up_src.z_wu);

			initTapMillis = HAL_GetTick();

			TxPayload _heartBeatPayload;
			generateUnscheduledTxPayload(sensors, &_heartBeatPayload);
			sendToLora(INTERRUPT_PORT, CONFIRMED_UPLINK, _heartBeatPayload);

     		//isTapDetected = false;
 			//isMovementDetected = false;
		}
		if(isTapDetected == true){
			if(HAL_GetTick() - initTapMillis > 3000){
				if(isMovementDetected == false){
					isMovementDetected = true;
					_accel->status = MOVING;
					//printf("Movement Detected \r\n ");

					TxPayload _heartBeatPayload;
					generateUnscheduledTxPayload(sensors, &_heartBeatPayload);
					sendToLora(INTERRUPT_PORT, CONFIRMED_UPLINK, _heartBeatPayload);

					isTapDetected = false;
					isMovementDetected = false;
				}
			}
		}
	  }
}

void readLTC4015(LTCStatus *ltc4015){

	 uint16_t i2c_data = 0;

	 HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3A, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&i2c_data, 2, HAL_MAX_DELAY);
	 ltc4015->VBAT= i2c_data * 0.000192264 * 4;

	 HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&i2c_data, 2, HAL_MAX_DELAY);
	 ltc4015->VIN = i2c_data * 0.001648;

	 HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3C, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&i2c_data, 2, HAL_MAX_DELAY);
	 ltc4015->VSYS = i2c_data * 0.001648;
#ifdef SERIAL_DEBUG_LTC
	 printf("LTC4015 Readings \r\n");
	 printf("VIN  = %.2f \r\n", ltc4015->VIN);
	 printf("VBAT = %.2f \r\n", ltc4015->VBAT);
	 printf("VSYS = %.2f \r\n", ltc4015->VSYS);
#endif
	}

// Function to mock Modbus response
void mockModbusResponse(ModBus_t *modbus, uint8_t *data, uint16_t length) {
    memcpy(modbus->buffer, data, length);
    modbus->rxIndex = length;
}

// Function to append Modbus response to payload
void appendModbusToPayload(TxPayload *payload, ModBus_t *modbus) {
    memcpy(&payload->buffer[payload->length], modbus->buffer, modbus->rxIndex);
    payload->length += modbus->rxIndex;
}


void initQueue(TxPayloadQueue *q) {
    q->front = 0;
    q->rear = 0;
    q->size = 0;
}

int enqueue(TxPayloadQueue *q, TxPayload element) {
    if (q->size == QUEUE_MAX_SIZE) {
        // Queue is full
        return -1;
    }

    q->queue[q->rear] = element;
    q->rear = (q->rear + 1) % QUEUE_MAX_SIZE;
    q->size++;
    return 0;
}

int dequeue(TxPayloadQueue *q, TxPayload *element) {
    if (q->size == 0) {
        // Queue is empty
        return -1;
    }

    *element = q->queue[q->front];
    q->front = (q->front + 1) % QUEUE_MAX_SIZE;
    q->size--;
    return 0;
}

int isQueueEmpty(TxPayloadQueue *q) {
    return (q->size == 0);
}

int isQueueFull(TxPayloadQueue *q) {
    return (q->size == QUEUE_MAX_SIZE);
}


void queueUnscheduledPayload(void){

	TxPayload unscheduledPayload;
	generateUnscheduledTxPayload(sensors, &unscheduledPayload);
	if(isWarmedUp == true){
		if (enqueue(&payLoadQueue, unscheduledPayload) == 0) {
			//printf("Added to Queue \r\n");
		} else {
			//printf("Queue is full \r\n");
		}
	}
}
void queueHeartbeatPayload(void){
	TxPayload heartbeatPayload;
	generateHeartbeatTxPayload(sensors, &heartbeatPayload);

	if(isWarmedUp == true){
		if (enqueue(&payLoadQueue, heartbeatPayload) == 0) {
			//printf("Added Heartbeat to Queue \r\n");
		} else {
			//printf("Queue is full \r\n");
		}
	}

}

void queuePayload(TxPayload Payload){

	if(isWarmedUp == true){
		if (enqueue(&payLoadQueue, Payload) == 0) {
			//printf("Added Heartbeat to Queue \r\n");
		} else {
			//printf("Queue is full \r\n");
		}
	}

}

void readWaterLeak(WaterLeak *leak){
	if(ADS1115_readSingleEnded(ADS1115_MUX_AIN0, &adcRead) == HAL_OK){
		leak->raw = adcRead;
		leak->state = (adcRead > WATER_LEAK_TH) ? WET : DRY;
	 }else{
		 //printf("Error Reading Water Leak");
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
