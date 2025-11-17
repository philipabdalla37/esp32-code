#include "i2s_hal.h"
#include "freertos/queue.h"
#include <math.h>

// // --- First pair of mics on I2S0 ---
// #define I2S0_SCK 42   // BCLK
// #define I2S0_WS  41   // LRCLK / WS
// #define I2S0_SD  40   // DOUT (shared by left+right mics)
// #define I2S_PORT0 I2S_NUM_0

// // --- Second pair of mics on I2S1 ---
// #define I2S1_SCK 42 //9
// #define I2S1_WS  8  //8
// #define I2S1_SD  7
// #define I2S_PORT1 I2S_NUM_1

// #define SAMPLE_RATE 44100 //Hz. This is fWS; fSCK - 64 x FWS

// // Each "frame" is 2 samples (R then L) with I2S_CHANNEL_FMT_RIGHT_LEFT
// #define STEREO_FRAMES 64

// // Raw 32-bit frames (24-bit data left-justified)
// static int32_t buf_port_0[STEREO_FRAMES * 2];  // I2S0: interleaved R,L
// static int32_t buf_port_1[STEREO_FRAMES * 2];  // I2S1: interleaved R,L

// Downshift from 24-bit to ~16-bit for plotting
#define SHIFT_RIGHT 11

// Decimate/average so Serial can keep up
#define DECIMATE 16  // print one averaged sample per 16 frames
void setup() {
  Serial.begin(9600);
  // If your plot stutters, try a higher baud like 921600 (and set the Plotter to match)
  // Serial.begin(921600);
  Serial.println("ESP32-S3: 4-mic (2x I2S) reader - prints L1 R1 L2 R2");
  I2sHal_Init();
  delay(500);
  // I2sHal_AutoCalibrate(20000); // measure for 3000 ms (3 seconds)
}

void loop() {
  I2sHal_Run();
//   size_t byte_read_from_port_0 = 0, byte_read_from_port_1 = 0;//Number of bytes read
    
//   // Read a chunk from both ports
//   if (i2s_read(I2S_PORT0, buf_port_0, sizeof(buf_port_0), &byte_read_from_port_0, portMAX_DELAY) != ESP_OK ) return;//|| byte_read_from_port_0 == 0
//  // if (i2s_read(I2S_PORT1, buf_port_1, sizeof(buf_port_1), &byte_read_from_port_1, portMAX_DELAY) != ESP_OK ) return;//|| byte_read_from_port_1 == 0

//   // compute common full frames (each frame = 2 x int32_t)
//   //size_t frames_common = ((byte_read_from_port_0 < byte_read_from_port_1) ? byte_read_from_port_0 : byte_read_from_port_1) / (sizeof(int32_t) * 2);
//   size_t frames_common = 
//   if (frames_common == 0) return;

//   // Accumulators for RMS (sum of squares) for each channel (use 64-bit)
//   int64_t accMic1_sq = 0, accMic2_sq = 0, accMic3_sq = 0, accMic4_sq = 0;
//   int dec = 0;
//   /*
//     MIC3        MIC4
  

//     MIC2        MIC1
//   */
//   for (size_t i = 0; i < frames_common; i++) {
  //   // I2S_CHANNEL_FMT_RIGHT_LEFT => index 0 = Left, index 1 = Right
  //   int32_t raw_port0_left_mic_3 = buf_port_0[2*i + 0];
  //   int32_t raw_port0_right_mic_4 = buf_port_0[2*i + 1]; //32 bits per sample
  //   // int32_t raw_port1_left_mic_2 = buf_port_1[2*i + 0];
  //   // int32_t raw_port1_right_mic_1 = buf_port_1[2*i + 1];

  //   // Downshift 24-bit audio to ~16-bit range
  //   int16_t mic3_16_bit_value = (int16_t)(raw_port0_left_mic_3 >> SHIFT_RIGHT);
  //   int16_t mic4_16_bit_value = (int16_t)(raw_port0_right_mic_4 >> SHIFT_RIGHT);
  //   // int16_t mic2_16_bit_value = (int16_t)(raw_port1_left_mic_2 >> SHIFT_RIGHT);
  //   // int16_t mic1_16_bit_value = (int16_t)(raw_port1_right_mic_1 >> SHIFT_RIGHT);

  //   // accumulate sum of squares for RMS
  //   accMic3_sq += (int64_t)mic3_16_bit_value * (int64_t)mic3_16_bit_value;
  //   accMic4_sq += (int64_t)mic4_16_bit_value * (int64_t)mic4_16_bit_value;
  //   // accMic2_sq += (int64_t)mic2_16_bit_value * (int64_t)mic2_16_bit_value;
  //   // accMic1_sq += (int64_t)mic1_16_bit_value * (int64_t)mic1_16_bit_value;
  //   dec++;

  //   if (dec >= DECIMATE) {
  //     // Compute RMS for the block: sqrt(sumSquares / N)
  //     double rmsMic3 = sqrt((double)accMic3_sq / (double)DECIMATE);
  //     double rmsMic4 = sqrt((double)accMic4_sq / (double)DECIMATE);
  //     // double rmsMic2 = sqrt((double)accMic2_sq / (double)DECIMATE);
  //     // double rmsMic1 = sqrt((double)accMic1_sq / (double)DECIMATE);

  //     // False print statements to "lock range" on serial plotter display
  //     // Change rangelimit value to adjust "sensitivity"
  //     // int rangelimit = 3000;
  //     // Serial.print(rangelimit * -1);
  //     // Serial.print(" ");
  //     // Serial.print(rangelimit);
  //     // Serial.print(" ");

  //     // Print 4 channels per line: L1 R1 L2 R2 (as floats)
  //     // Serial.print(rmsMic1); Serial.print(' ');
  //     // Serial.print(rmsMic2); Serial.print(' ');
  //     Serial.print(rmsMic3); Serial.print(' ');
  //     Serial.println(rmsMic4);

  //     // Reset for next block
  //     //accMic1_sq = accMic2_sq = accMic3_sq = accMic4_sq = 0;
  //     accMic3_sq = accMic4_sq = 0;
  //     dec = 0;
  //   }
  // }
}
