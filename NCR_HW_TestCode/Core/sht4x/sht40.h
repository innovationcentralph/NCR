/*
 * sht40.h
 * Author: Kevin Zheng
 */

#ifndef INC_SHT40_H_
#define INC_SHT40_H_

#include <stdbool.h>
#include "stm32wbxx_hal.h"

/*
 * I2C Communication
 */
#define SHT40_I2C_ADDR (0x44 << 1)
#define SHT40_I2C_TIMEOUT 500 /* 100 ms */
#define SHT40_I2C_RESP_LEN 6

/*
 * Commands
 */
#define SHT40_HI_PREC_MEASURE  0xFD
#define SHT40_MED_PREC_MEASURE 0xF6
#define SHT40_LOW_PREC_MEASURE 0xE0
#define SHT40_READ_SERIAL      0x89
#define SHT40_SOFT_RESET       0x94
#define SHT40_200mW_1s_HEAT    0x39
#define SHT40_200mW_100ms_HEAT 0x32
#define SHT40_110mW_1s_HEAT    0x2F
#define SHT40_110mW_100ms_HEAT 0x24
#define SHT40_20mW_1s_HEAT     0x1E
#define SHT40_20mW_100ms_HEAT  0x15

/*
 * Result/Measurement Struct
 */
typedef struct {

    /* Temperature in Celsius */
    float temperature;

    /* Percentage relative humidity */
    float rel_humidity;

} SHT40_Measurement;

/*
 * Measurement Precision Levels
 */
typedef enum {
    HIGH_PRECISION = SHT40_HI_PREC_MEASURE,  /* Low repeatability */
    MED_PRECISION  = SHT40_MED_PREC_MEASURE, /* Medium repeatability */
    LOW_PRECISION  = SHT40_LOW_PREC_MEASURE  /* High repeatability */
} SHT40_Precision;

/*
 * Heater Duration and Power Options
 */
typedef enum {
    HIGH_POWER_1s      = SHT40_200mW_1s_HEAT,
    HIGH_POWER_100ms   = SHT40_200mW_100ms_HEAT,
    MEDIUM_POWER_1s    = SHT40_110mW_1s_HEAT,
    MEDIUM_POWER_100ms = SHT40_110mW_100ms_HEAT,
    LOW_POWER_1s       = SHT40_20mW_1s_HEAT,
    LOW_POWER_100ms    = SHT40_20mW_100ms_HEAT
} SHT40_HeaterOption;

/*
 * Measurement-Taking
 */
HAL_StatusTypeDef SHT40_Measure(I2C_HandleTypeDef* i2cHandle, SHT40_Measurement* result, SHT40_Precision precision);

/*
 * Heater Control
 * SHT40_Measurement is optional (may be provided as NULL)
 */
HAL_StatusTypeDef SHT40_Heat(I2C_HandleTypeDef* i2cHandle, SHT40_Measurement* result, SHT40_HeaterOption heat_option);

/*
 * Reset/Serial
 */
HAL_StatusTypeDef SHT40_SoftReset(I2C_HandleTypeDef* i2cHandle);
HAL_StatusTypeDef SHT40_ReadSerial(I2C_HandleTypeDef* i2cHandle, uint32_t* result);

#endif /* INC_SHT40_H_ */
