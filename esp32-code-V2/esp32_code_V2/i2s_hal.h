/************************************************
*@file i2s_hal.h
*@created by Philip Abdalla
*@brief I2S Hardware Abstarction Layer
*/

#ifndef I2S_HAL_
#define I2S_HAL_

/*****************INCLUDES******************************/
#include "error_codes.h"
#include "config_hal.h"
#include <driver/i2s.h>

/********************MACRO DEFINITIONS***********************************/

/*************************Enumerations***********************************************/

typedef enum
{
  I2S_OK = 0x00000000U,
  I2S_FAIL = 0x00000001U,
  I2S_DRIVER_INSTALL_ERR = 0x00000002U,
  I2S_SET_PINS_ERR = 0x00000004U, 
  I2S_START_ERR = 0x00000008U, 
  I2S_PORT0_READ_ERR = 0x00000010U,
  I2S_PORT1_READ_ERR = 0x00000020U,
}I2sHalError_t;

/*************************Public Structs***********************************************/
typedef struct
{
  bool is_config;
  uint32_t i2s_error;
}I2sConfig_t;

/*************************Public Functions***********************************************/
extern capstoneErrorCode_t I2sHal_Init(void);
extern capstoneErrorCode_t I2sHal_Run(void);
extern void I2sHal_SetCalibrationGain(uint8_t mic_index, float gain);
extern capstoneErrorCode_t I2sHal_AutoCalibrate(uint32_t duration_ms);



#endif /*I2S_HAL_*/
