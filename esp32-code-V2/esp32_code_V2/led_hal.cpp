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
    // 1. RESET: Turn all lights off first
    fill_solid(ledData.leds, LED_NUM_LEDS, CRGB::Black);
    
    // 2. LOOKUP: Get the starting LED index from your map
    // (Ensure MIC_LED_MAP is defined globally or in your header)
    int start_index = MIC_LED_MAP[mic].start_led_index;

    // 3. LOOP: Turn on 4 LEDs starting from that index
    for (int i = 0; i < 4; i++) {
        
        // --- THE MAGIC MATH (Modulo Operator) ---
        // We take the start_index, add 'i' (0, 1, 2, 3), 
        // and use '%' to wrap it around if it exceeds the total LEDs.
        int target_led = (start_index + i) % LED_NUM_LEDS;
        
        ledData.leds[target_led] = CRGB::White; // Or your preferred color
    }

    // 4. SHOW: Push data to strip
    FastLED.show();
    
    return CAPSTONE_SUCCESS;
}


void LedHal_White()
{
  fill_solid(ledData.leds, LED_NUM_LEDS, CRGB::White);
}