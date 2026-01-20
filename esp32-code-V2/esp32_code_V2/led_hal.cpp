/************************************************
*@file led_hal.c
*@created by Philip Abdalla
*@brief LED Hardware Abstarction Layer
*/

/**************************INCLUDES*************************************************/
#include "led_hal.h"
#include "config_hal.h"
#include <FastLED.h>
#include <Arduino.h>

/********************MACRO DEFINITIONS***********************************************/
/*************************Enumerations***********************************************/


/*************************Private Structs***********************************************/
typedef struct
{
  CRGB leds[LED_NUM_LEDS];
} LedData_t;


/*************************Local Functions***********************************************/
static LedData_t ledData = {0};

capstoneErrorCode_t LedHal_Init(void)
{
  FastLED.addLeds<WS2812, LED_DATA_PIN>(ledData.leds, LED_NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  return CAPSTONE_SUCCESS;
}

capstoneErrorCode_t LedHal_Run(I2sMics_t mic)
{
    // 1. RESET: Turn all lights off in the memory buffer first.
    // Without this, the previous winner's lights will stay on forever.
    fill_solid(ledData.leds, LED_NUM_LEDS, CRGB::Black);
    
    // 2. MATH: Calculate the slice size dynamically
    // 16 LEDs / 4 Mics = 4 LEDs per Mic
    int leds_per_mic = LED_NUM_LEDS / MIC_TOTAL; 
    
    int start_index = (int)mic * leds_per_mic;
    int end_index   = start_index + leds_per_mic;

    // 3. SET: Turn on only the winner's section
    for (int i = start_index; i < end_index; i++) {
        // Safety check: ensure we don't write outside the array
        if (i < LED_NUM_LEDS) {
            ledData.leds[i] = CRGB::White;   
        }
    }

    // 4. SHOW: Push the data to the strip
    FastLED.show();
    
    return CAPSTONE_SUCCESS;
}