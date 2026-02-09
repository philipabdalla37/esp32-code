/************************************************
*@file config_hal.h
*@created by Philip Abdalla
*@brief Error Codes
*/

#ifndef CONFIG_HAL_
#define CONFIG_HAL_

#include <driver/i2s_std.h>
#include <driver/gpio.h>

/*******************I2S Config***************************/
// --- First pair of mics on I2S0 ---
// Pair A (Mics 1 & 2) -> I2S_NUM_0
#define I2S0_SCK      GPIO_NUM_42   // Serial Clock (SCK)
#define I2S0_WS       GPIO_NUM_41   // Word Select (WS)
#define I2S0_SD       GPIO_NUM_40   // Serial Data (SD)

// Pair B (Mics 3 & 4) -> I2S_NUM_1
#define I2S1_SCK      GPIO_NUM_4   // Serial Clock (SCK)
#define I2S1_WS       GPIO_NUM_5   // Word Select (WS)
#define I2S1_SD       GPIO_NUM_6   // Serial Data (SD)


#define I2S_NUM_DMA 6
#define I2S_DMA_SIZE_IN_SAMPLES 512
#define SAMPLE_RATE 16000  // 16 kHz for speech (16000)
#define STEREO_FRAMES 64 // Each "frame" is 2 samples (R then L) with I2S_CHANNEL_FMT_RIGHT_LEFT


/*******************Light Ring Config***************************/
#define LED_DATA_PIN 18
#define LED_NUM_LEDS 16
#define LED_TYPE WS2812
#define LED_BRIGHTNESS 50

/*******************Button Config***************************/
#define BUTTON 8 //when prssed it is LOW

/***************************Enumerations*****************************************/
typedef enum{
  I2S_PORT_0 = I2S_NUM_0,
  I2S_PORT_1 = I2S_NUM_1,
  I2S_PORT_TOTAL = 2
}
I2sPort_t;

typedef enum{
  MIC_1 = 0,
  MIC_2,
  MIC_3,
  MIC_4,
  MIC_TOTAL
}
I2sMics_t;

typedef struct {
    I2sMics_t mic; 
    int start_led_index;
} MicLedMapping_t; // I renamed this slightly to make it clear it's a "Map"

//Added [MIC_TOTAL] to make it an array
static const MicLedMapping_t MIC_LED_MAP[MIC_TOTAL] = {
    {MIC_1, 14},   // Mic 0 starts at LED 0
    {MIC_2, 2},   // Mic 1 starts at LED 4
    {MIC_3, 10},   // Mic 2 starts at LED 8
    {MIC_4, 6}   // Mic 3 starts at LED 12
};

#endif /*CONFIG_HAL_*/