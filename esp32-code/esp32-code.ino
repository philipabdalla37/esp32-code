/*
  ESP32 I2S Microphone Sample
  esp32-i2s-mic-sample.ino
  Sample sound from I2S microphone, display on Serial Plotter
  Requires INMP441 I2S microphone
*/

// Include I2S driver
#include <driver/i2s.h>
#include <math.h>

// Connections to INMP441 I2S microphone
#define I2S_WS 41
#define I2S_SD 40
#define I2S_SCK 42

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

// Define input buffer length
#define bufferLen 64
int16_t sBuffer[bufferLen]; //holds 64 audio samples with each 16 bit

void i2s_install() {
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100, //Mic sends 44100 numbers per second
    .bits_per_sample = i2s_bits_per_sample_t(16),//each audio sample is 16 bits = 2bytes
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,//This Mono (1 side)
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

void setup() {

  // Set up Serial Monitor
  Serial.begin(115200);
  Serial.println(" ");

  delay(1000);

  // Set up I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);


  delay(500);
}

void loop() {

  // False print statements to "lock range" on serial plotter display
  // Change rangelimit value to adjust "sensitivity"
  int rangelimit = 3000;
  Serial.print(rangelimit * -1);
  Serial.print(" ");
  Serial.print(rangelimit);
  Serial.print(" ");

  // Get I2S data and place in data buffer
  size_t bytesIn = 0; //tells you how many bytes were actually received.
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

  if (result == ESP_OK)
  {
    // Read I2S data buffer
    int32_t samples_read = bytesIn / 8;
    if (samples_read > 0) {
      float rms = 0;
      for (int16_t i = 0; i < samples_read; ++i) {
        //mean += (sBuffer[i]);
        rms += (sBuffer[i] * sBuffer[i]);
      }

      // Average the data reading
      //mean /= samples_read;
      rms = sqrt(rms/samples_read);

      // Print to serial plotter
      //Serial.println(mean);
      Serial.println(rms);
    // float sumSquares = 0;
    // for (int i = 0; i < samples_read; i++) {
    //   int32_t shifted = sBuffer[i] >> 14;    // Convert 24-bit MSB audio to signed 16-bit
    //   float sample = shifted / 1024.0f;      // Normalize to -1.0 to +1.0 range
    //   sumSquares += sample * sample;
    // }
     //float rms = sqrt(sumSquares / samples_read);
    // Serial.println(rms);
    }
  }
}