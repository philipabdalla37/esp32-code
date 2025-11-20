#include "i2s_hal.h"
#include "freertos/queue.h"
#include <math.h>

void setup() {
  Serial.begin(9600);
  Serial.println("ESP32-S3: 4-mic (2x I2S) reader - prints L1 R1 L2 R2");
  I2sHal_Init();
  delay(500);
  // I2sHal_AutoCalibrate(20000); // measure for 3000 ms (3 seconds)
  // delay(500);
}

void loop() {
  I2sHal_Run();
}
