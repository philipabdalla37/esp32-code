#include <Arduino.h>
#include "mic_i2s_hal.h"
#include "led_hal.h"
#include "error_codes.h"
#include "config_hal.h"

void setup() {
  Serial.begin(921600);
  delay(500);  // Give serial time to stabilize
  LedHal_Init();
  Serial.println("ESP32-S3: 4-mic (2x I2S) reader - prints L1 R1 L2 R2");
  I2sHal_Init();
  delay(500);
  pinMode(BUTTON, INPUT_PULLUP);
  // I2sHal_AutoCalibrate(5000);
  // delay(50);
}

void loop() {
  // Read the state of the button
  int buttonState = digitalRead(BUTTON);

  if (I2sHal_Run() == CAPSTONE_SUCCESS && buttonState == LOW) {
    // 1. Get Audio
    int16_t* audio_buffer = I2sHal_GetBuffer();
    
    // 2. Send Raw Binary to Pi
    // 64 samples * 2 bytes = 128 bytes per packet
    Serial.write((uint8_t*)audio_buffer, STEREO_FRAMES * sizeof(int16_t));

    // 3. Example: Update LED Ring (Pseudo-code)
    LedHal_Run(I2sHal_GetClosestMic());
  } else {
    LedHal_White();
    delay(10);
  }

}
