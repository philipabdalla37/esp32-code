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
  I2S_PORT0_READ_ERR = 0x00000002U,
  I2S_PORT1_READ_ERR = 0x00000004U,
}I2sHalError_t;

/*************************Public Structs***********************************************/
typedef struct
{
  bool is_config;
  uint32_t i2s_error;
}I2sConfig_t;

/*************************Public Functions***********************************************/
capstoneErrorCode_t I2sHal_Init(void);
capstoneErrorCode_t I2sHal_Read(void);



#endif /*I2S_HAL_*/
