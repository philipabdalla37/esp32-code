/************************************************
*@file led_hal.h
*@created by Philip Abdalla
*@brief LED Hardware Abstarction Layer
*/

#ifndef LED_HAL_
#define LED_HAL_

/*****************INCLUDES******************************/
#include "error_codes.h"
#include "config_hal.h"
#include <math.h>
/********************MACRO DEFINITIONS***********************************/

/*************************Enumerations***********************************************/

typedef enum
{
  LED_OK = 0x00000000U,
  LED_FAIL = 0x00000001U,
}LedHalError_t;

/*************************Public Structs***********************************************/

/*************************Public Functions***********************************************/
extern capstoneErrorCode_t LedHal_Init(void);
extern capstoneErrorCode_t LedHal_Run(I2sMics_t mic);
extern void LedHal_White();

#endif /*LED_HAL_*/
