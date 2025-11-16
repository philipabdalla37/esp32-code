/************************************************
*@file config_hal.h
*@created by Philip Abdalla
*@brief Error Codes
*/

#ifndef CONFIG_HAL_
#define CONFIG_HAL_

#include <driver/i2s.h>

/*******************I2S Config***************************/
// --- First pair of mics on I2S0 ---
#define I2S0_SCK 42   // BCLK
#define I2S0_WS  41   // LRCLK / WS
#define I2S0_SD  40   // DOUT (shared by left+right mics)

// --- Second pair of mics on I2S1 ---
#define I2S1_SCK I2S0_SCK //before it was 9
#define I2S1_WS  8  
#define I2S1_SD  7

#define SAMPLE_RATE 16000  // 16 kHz for speech

#define STEREO_FRAMES 64 // Each "frame" is 2 samples (R then L) with I2S_CHANNEL_FMT_RIGHT_LEFT



/***************************Enumerations*****************************************/
typedef enum{
  I2S_PORT_0 = I2S_NUM_0,
  I2S_PORT_1 = I2S_NUM_1,
  I2S_PORT_TOTAL = (I2S_NUM_1 + 1)
}
I2sPort_t;

#endif /*CONFIG_HAL_*/