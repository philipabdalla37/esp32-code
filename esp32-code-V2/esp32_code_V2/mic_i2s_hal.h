/************************************************
*@file mic_i2s_hal.h
*@created by Philip Abdalla
*@brief I2S Hardware Abstarction Layer
*/

#ifndef I2S_HAL_
#define I2S_HAL_

/*****************INCLUDES******************************/
#include "error_codes.h"
#include "config_hal.h"
#include <math.h>

/********************MACRO DEFINITIONS***********************************/

/*************************Enumerations***********************************************/

typedef enum
{
  I2S_OK = 0x00000000U,
  I2S_FAIL = 0x00000001U,
  I2S_CHANNEL_NEW_ERR = 0x00000002U,
  I2S_CHANNEL_A_INIT_ERR = 0x00000004U,
  I2S_CHANNEL_B_INIT_ERR = 0x00000008U,  
  I2S_CHANNEL_ENABLE_ERR = 0x00000010U, 
  I2S_CHANNEL_READ_ERR = 0x00000020U,
  I2S_CHANNEL_DISABLE_ERR = 0x00000040U,
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
extern int16_t* I2sHal_GetBuffer(void);
extern I2sMics_t I2sHal_GetClosestMic(void);
// extern void I2sHal_SetCalibrationGain(uint8_t mic_index, float gain);
// extern capstoneErrorCode_t I2sHal_AutoCalibrate(uint32_t duration_ms);



#endif /*I2S_HAL_*/
