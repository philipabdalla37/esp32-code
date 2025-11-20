/************************************************
*@file i2s_hal.c
*@created by Philip Abdalla
*@brief I2S Hardware Abstarction Layer
*/

/**************************INCLUDES*************************************************/
#include "i2s_hal.h"
#include <Arduino.h>
#include <math.h>

/********************MACRO DEFINITIONS***********************************************/
// ---- High-pass filter config ----
// #define HPF_ALPHA 0.97f   // tweak between 0.95–0.99 if needed
// // ---- Low-pass filter config ----
// #define LPF_ALPHA 0.1f   // 0 < alpha <= 1, smaller = smoother, lower cutoff
#define M_PI 3.14159265358979323846 // Define PI if not available

// ---- Band-pass filter config: 93–238 Hz ----
#define HP_CUTOFF_HZ   100.0f    // high-pass corner (93)
#define LP_CUTOFF_HZ  3000.0f   // low-pass corner (238)

// 3. Hysteresis: The "Sticky" Factor
// Challenger must be 20% louder for 3 frames to win
#define HYSTERESIS_FACTOR    1.30f    
#define HYSTERESIS_FRAMES    3   

// Add to your header file (i2s_hal.h)
#define SPEECH_RMS_THRESHOLD 20000.0f  // Adjust based on your calibration
#define MIN_SPEECH_FRAMES 3             // Require multiple frames above threshold

/*************************Enumerations***********************************************/


/*************************Private Structs***********************************************/
typedef struct
{
  int32_t buf_port[I2S_PORT_TOTAL][STEREO_FRAMES * I2S_PORT_TOTAL];
  size_t buf_bytes_read[I2S_PORT_TOTAL];//are 512 = 64 frames x 2 samples/frame x 4 bytes/sample 
  uint32_t i2s_error;
  float calibration_gain[4]; /* per-microphone gain factors (mic1..mic4) */
  uint8_t speech_frame_count[4];  // Track consecutive speech frames per mic
  int8_t prev_closest_mic;        // Previously-detected loud mic (-1 = none)
  int current_best_mic; // The Mic ID (0-3) currently selected as the "Winner"
  int hysteresis_count; // Counter: How many frames has a challenger been louder?
} I2sData_t;


/*************************Local Functions***********************************************/
void setup_i2s(I2sPort_t port, int sck, int ws, int sd);
static inline int32_t I2sHal_Extract24BitSample(int32_t raw32BitSample);
static inline int32_t I2sHal_ApplyCalibration(int32_t sample, float gain);

/* Extract 24-bit audio sample from MSB-aligned 32-bit I2S word */
static inline int32_t I2sHal_Extract24BitSample(int32_t raw32BitSample)
{
  /* INMP441 outputs 24-bit audio MSB-aligned, padded with 8 zero bits at LSB */
  /* Shift right by 8 to get the 24-bit value */
  return (raw32BitSample >> 8);
}

/* Apply calibration gain to a 24-bit sample and clamp to 24-bit signed range */
static inline int32_t I2sHal_ApplyCalibration(int32_t sample, float gain)
{
  float scaled = (float)sample * gain;
  if (scaled > 8388607.0f) scaled = 8388607.0f;
  if (scaled < -8388608.0f) scaled = -8388608.0f;
  return (int32_t)scaled;
}

/*************************Local Variables***********************************************/
static I2sConfig_t i2sConfig = {0};//Used Only for Init
static I2sData_t i2sData = {0};
// --- STATE VARIABLES ---
// We need memory for 4 microphones
// HP = High Pass, LP = Low Pass
static float hp_prev_output[4] = {0,0,0,0}; // To track the "DC Offset"
static float lp_prev_output[4] = {0,0,0,0}; // To track the "Smooth signal"

// --- COEFFICIENTS ---
static float alpha_hp = 0.0f;
static float alpha_lp = 0.0f;

//static float lpf_prev_out[4] = {0, 0, 0, 0};

// 2. The Filter Function: Call this inside your loop for EVERY sample
int32_t DSP_FilterSample(int mic_index, int32_t raw_sample) {
    float x = (float)raw_sample;

    // --- STAGE 1: HIGH PASS (DC REMOVAL) ---
    // Algorithm: Track the "average" (low pass) and subtract it from input.
    // y_avg[n] = y_avg[n-1] + alpha * (x[n] - y_avg[n-1])
    
    float prev_dc = hp_prev_output[mic_index];
    float current_dc = prev_dc + alpha_hp * (x - prev_dc);
    
    // Update state
    hp_prev_output[mic_index] = current_dc;

    // Output = Input - Average
    float x_no_dc = x - current_dc;

    // --- STAGE 2: LOW PASS (HISS REMOVAL) ---
    // Algorithm: Smooth out the result from Stage 1
    
    float prev_smooth = lp_prev_output[mic_index];
    float current_smooth = prev_smooth + alpha_lp * (x_no_dc - prev_smooth);

    // Update state
    lp_prev_output[mic_index] = current_smooth;

    // --- STAGE 3: CLAMPING ---
    // Ensure we don't overflow 24-bit integers
    if (current_smooth > 8388607.0f) current_smooth = 8388607.0f;
    if (current_smooth < -8388608.0f) current_smooth = -8388608.0f;

    return (int32_t)current_smooth;
}

// 1. Setup Function: Call this once in I2sHal_Init()
void DSP_CalculateCoefficients(uint32_t sample_rate) {
    float dt = 1.0f / (float)sample_rate;

    // Calculate Alpha for High Pass (100Hz)
    // Note: For HP, we calculate the Low-Pass equivalent first to subtract it
    float tau_hp = 1.0f / (2.0f * M_PI * HP_CUTOFF_HZ);
    alpha_hp = dt / (tau_hp + dt);

    // Calculate Alpha for Low Pass (3000Hz)
    float tau_lp = 1.0f / (2.0f * M_PI * LP_CUTOFF_HZ);
    alpha_lp = dt / (tau_lp + dt);

    // Debug Print
    // Serial.print("Alpha HP (100Hz): "); Serial.println(alpha_hp, 6);
    // Serial.print("Alpha LP (3000Hz): "); Serial.println(alpha_lp, 6);
}

capstoneErrorCode_t I2sHal_Init(void)
{
  setup_i2s(I2S_PORT_0, I2S0_SCK, I2S0_WS, I2S0_SD);
  setup_i2s(I2S_PORT_1, I2S1_SCK, I2S1_WS, I2S1_SD);

  // Calculate filter coefficients based on the sample rate
  DSP_CalculateCoefficients(SAMPLE_RATE);

  /* initialize calibration gains to 1.0 (no change) */
  for (int i = 0; i < 4; ++i) i2sData.calibration_gain[i] = 1.0f;
  
  // Initialize Logic State
  i2sData.current_best_mic = 0; // Default to Mic 1
  i2sData.hysteresis_count = 0;
  
  if (i2sConfig.i2s_error != I2S_OK)
  {
    return CAPSTONE_FAIL;
  }
  return CAPSTONE_SUCCESS;
}

capstoneErrorCode_t I2sHal_Run(void)
{
  // Read a chunk from both ports
  if (i2s_read((i2s_port_t)I2S_PORT_0, i2sData.buf_port[I2S_PORT_0], sizeof(i2sData.buf_port[I2S_PORT_0]), &i2sData.buf_bytes_read[I2S_PORT_0], portMAX_DELAY) != ESP_OK )
  {
    i2sData.i2s_error |= I2S_PORT0_READ_ERR;
    return CAPSTONE_FAIL;
  } 
  if (i2s_read((i2s_port_t)I2S_PORT_1, i2sData.buf_port[I2S_PORT_1], sizeof(i2sData.buf_port[I2S_PORT_1]), &i2sData.buf_bytes_read[I2S_PORT_1], portMAX_DELAY) != ESP_OK )
  {
    i2sData.i2s_error |= I2S_PORT1_READ_ERR;
    return CAPSTONE_FAIL;
  } 

  size_t frames_common = ( (i2sData.buf_bytes_read[I2S_PORT_0] < i2sData.buf_bytes_read[I2S_PORT_1]) ? i2sData.buf_bytes_read[I2S_PORT_0] : i2sData.buf_bytes_read[I2S_PORT_1]) / (sizeof(int32_t) * 2); // each frame = 2 x int32_t (stereo: Right + Left)
  if (frames_common == 0) return CAPSTONE_FAIL; //frames_common reads the full Buf length (64)

  // Accumulators for RMS: sum of squares for each mic
  // We use double to avoid overflowing a 32-bit integer
  double sum_sq[4] = {0, 0, 0, 0};
  int64_t max_value = 0;
  //int closest_mic = 0;

  for(size_t i = 0; i < frames_common; i++)
  {
    int32_t raw[4];
    raw[0] = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_0][i * 2 + 0]); // Left from Port 0 (24-bit)
    raw[1] = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_0][i * 2 + 1]); // Right from Port 0 (24-bit)
    raw[2] = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_1][i * 2 + 0]); // Left from Port 1 (24-bit)
    raw[3] = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_1][i * 2 + 1]); // Right from Port 1 (24-bit)

    for(int k=0; k<4; k++) {
        // A. FILTER
        int32_t clean = DSP_FilterSample(k, raw[k]);
        
        // B. SQUARE and ACCUMULATE
        // Energy = Amplitude * Amplitude
        sum_sq[k] += ((double)clean * clean);
    }

    // Serial.print(clean_mic1); Serial.print(' ');
    // Serial.print(clean_mic2); Serial.print(' ');
    // Serial.print(clean_mic3); Serial.print(' ');
    // Serial.println(clean_mic4);

    // int32_t mic1 = I2sHal_ApplyCalibration(raw1, i2sData.calibration_gain[0]);
    // int32_t mic2 = I2sHal_ApplyCalibration(raw2, i2sData.calibration_gain[1]);
    // int32_t mic3 = I2sHal_ApplyCalibration(raw3, i2sData.calibration_gain[2]);
    // int32_t mic4 = I2sHal_ApplyCalibration(raw4, i2sData.calibration_gain[3]);

    // High-pass filter to remove DC / low frequency noise
    // mic1 = I2sHal_HighPass(0, mic1);
    // mic2 = I2sHal_HighPass(1, mic2);
    // mic3 = I2sHal_HighPass(2, mic3);
    // mic4 = I2sHal_HighPass(3, mic4);

    // Then low-pass
    // mic1 = I2sHal_LowPass(0, mic1);
    // mic2 = I2sHal_LowPass(1, mic2);
    // mic3 = I2sHal_LowPass(2, mic3);
    // mic4 = I2sHal_LowPass(3, mic4);

    // Apply 93–238 Hz band-pass per mic
    // int mic1 = I2sHal_BandPass_93_238(0, raw1);
    // int mic2 = I2sHal_BandPass_93_238(1, raw2);
    // int mic3 = I2sHal_BandPass_93_238(2, raw3);
    // int mic4 = I2sHal_BandPass_93_238(3, raw4);

    // if(mic1 < 2500 && mic1 >-2500 ){
    //   mic1 = 0;
    // }
    // if(mic2 < 2500 && mic2 >-2500 ){
    //   mic2 = 0;
    // }
    // if(mic3 < 2500 && mic3 >-2500 ){
    //   mic3 = 0;
    // }
    // if(mic4 < 2500 && mic4 >-2500 ){
    //   mic4 = 0;
    // }

    // acc_sq[0] += (int64_t)mic1 * (int64_t)mic1;
    // acc_sq[1] += (int64_t)mic2 * (int64_t)mic2;
    // acc_sq[2] += (int64_t)mic3 * (int64_t)mic3;
    // acc_sq[3] += (int64_t)mic4 * (int64_t)mic4;
    // max_value = acc_sq[0];
    // closest_mic = 1;
    // for (int m = 1; m < 4; ++m)
    // {
    //   if (acc_sq[m] > max_value)
    //   {
    //     max_value = acc_sq[m];
    //     closest_mic = m+1;
    //   }
    // }
    // Serial.println(closest_mic);
    // Serial.print(mic1); Serial.print(' ');
    // Serial.print(mic2); Serial.print(' ');
    // Serial.print(mic3); Serial.print(' ');
    // Serial.println(mic4);
  }
  
  // 3. Calculate RMS & Find Potential Winner
  float rms[4];
  int frame_winner = -1;
  float max_vol = 0;

  for(int k=0; k<4; k++) {
      rms[k] = sqrt(sum_sq[k] / frames_common);
      if (rms[k] > max_vol) {
          max_vol = rms[k];
          frame_winner = k;
      }
  }

  // 4. Hysteresis Logic (The "Brain")
  if (max_vol > SPEECH_RMS_THRESHOLD) {
      // Logic: Is the frame winner different from our current best?
      if (frame_winner != i2sData.current_best_mic) {
          // Logic: Is it SIGNIFICANTLY louder? (20% louder)
          if (rms[frame_winner] > (rms[i2sData.current_best_mic] * HYSTERESIS_FACTOR)) {
              i2sData.hysteresis_count++;
              // Logic: Has it been louder for long enough?
              if (i2sData.hysteresis_count >= HYSTERESIS_FRAMES) {
                  i2sData.current_best_mic = frame_winner; // SWAP CAMERAS!
                  i2sData.hysteresis_count = 0;
              }
          } else {
              i2sData.hysteresis_count = 0;
          }
      } else {
          i2sData.hysteresis_count = 0;
      }
  }

  // 5. VISUALIZATION
  // Print RMS Lines
  Serial.print("Mic1:"); Serial.print(rms[0]); Serial.print(" ");
  Serial.print("Mic2:"); Serial.print(rms[1]); Serial.print(" ");
  Serial.print("Mic3:"); Serial.print(rms[2]); Serial.print(" ");
  Serial.print("Mic4:"); Serial.print(rms[3]); Serial.print(" ");
  
  // Print "Decision Line" (Steps up/down to show selected mic)
  Serial.print("SELECTED:"); Serial.println((i2sData.current_best_mic + 1));

  // Compute RMS for each mic
  // float rms[4];
  // bool speech_detected[4] = {false, false, false, false};
  
  // for (int m = 0; m < 4; ++m)
  // {
  //   rms[m] = sqrtf((float)acc_sq[m] / (float)frames_common);
    
  //   // Check if RMS exceeds speech threshold
  //   if (rms[m] > SPEECH_RMS_THRESHOLD)
  //   {
  //     speech_detected[m] = true;
  //     i2sData.speech_frame_count[m]++;
  //   }
  //   else
  //   {
  //     rms[m] = 0;
  //     i2sData.speech_frame_count[m] = 0;  // Reset counter
  //    }
  // }

  // Serial.print(rms[0]); Serial.print(' ');
  // Serial.print(rms[1]); Serial.print(' ');
  // Serial.print(rms[2]); Serial.print(' ');
  // Serial.println(rms[3]);

  // // Find microphone with highest RMS that exceeds threshold
  // int closest_mic = -1;
  // float max_rms = SPEECH_RMS_THRESHOLD;

  // for (int m = 0; m < 3; ++m)//4
  // {
  //   //if (i2sData.speech_frame_count[m] >= MIN_SPEECH_FRAMES && rms[m] > max_rms)
  //   if (rms[m] > max_rms)
    
  //   {
  //     max_rms = rms[m];
  //     closest_mic = m;
  //     if(closest_mic >= 0 && i2sData.prev_closest_mic != closest_mic){
  //       i2sData.prev_closest_mic = closest_mic;
  //     }
  //   }
  // }

  // // Print results
  // Serial.print("RMS: ");
  // for (int m = 0; m < 4; ++m)
  // {
  //   Serial.print("mic"); Serial.print(m+1); Serial.print("="); 
  //   Serial.print(rms[m], 1);
  //   if (speech_detected[m]) Serial.print("*");
  //   Serial.print(" ");
  // }
  
  // if (closest_mic >= 0)
  // {
  //   Serial.print(" -> Closest: mic"); Serial.print(closest_mic + 1);
  //   // print previous loud mic if known
  //   Serial.print(" (Prev: mic"); Serial.print(i2sData.prev_closest_mic + 1); Serial.print(")");
  //   Serial.println();

  //   // update previous loud mic
  //   i2sData.prev_closest_mic = (int8_t)closest_mic;
  // }
  // else
  // {
  //   Serial.print(" -> No speech detected");
  //   Serial.print(" (Prev: mic"); Serial.print(i2sData.prev_closest_mic + 1); Serial.println(")");
  //   // clear previous only if no sustained speech across all mics
  //   // keep previous value so short dropouts don't erase history
  // }

  return CAPSTONE_SUCCESS;
}

/* Auto-calibrate microphones by recording peak absolute values for duration_ms milliseconds
   Call this while playing a steady reference tone (e.g. 1 kHz) nearby.
   It measures the peak absolute sample seen on each mic and sets calibration_gain so
   all mics are normalized to the largest peak observed.
*/
capstoneErrorCode_t I2sHal_AutoCalibrate(uint32_t duration_ms)
{
  uint32_t start = millis();
  int32_t peak[4] = {0,0,0,0};

  Serial.print("[I2S] Auto-calibrate for "); Serial.print(duration_ms); Serial.println(" ms");

  while ((millis() - start) < duration_ms)
  {
    if (i2s_read((i2s_port_t)I2S_PORT_0, i2sData.buf_port[I2S_PORT_0], sizeof(i2sData.buf_port[I2S_PORT_0]), &i2sData.buf_bytes_read[I2S_PORT_0], portMAX_DELAY) != ESP_OK )
    {
      i2sData.i2s_error |= I2S_PORT0_READ_ERR;
      return CAPSTONE_FAIL;
    }
    if (i2s_read((i2s_port_t)I2S_PORT_1, i2sData.buf_port[I2S_PORT_1], sizeof(i2sData.buf_port[I2S_PORT_1]), &i2sData.buf_bytes_read[I2S_PORT_1], portMAX_DELAY) != ESP_OK )
    {
      i2sData.i2s_error |= I2S_PORT1_READ_ERR;
      return CAPSTONE_FAIL;
    }

    size_t frames_common = ( (i2sData.buf_bytes_read[I2S_PORT_0] < i2sData.buf_bytes_read[I2S_PORT_1]) ? i2sData.buf_bytes_read[I2S_PORT_0] : i2sData.buf_bytes_read[I2S_PORT_1]) / (sizeof(int32_t) * 2);
    if (frames_common == 0) continue;

    for (size_t i = 0; i < frames_common; ++i)
    {
      int32_t r1 = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_0][i * 2 + 0]);
      int32_t r2 = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_0][i * 2 + 1]);
      int32_t r3 = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_1][i * 2 + 0]);
      int32_t r4 = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_1][i * 2 + 1]);

      int32_t a1 = r1 < 0 ? -r1 : r1;
      int32_t a2 = r2 < 0 ? -r2 : r2;
      int32_t a3 = r3 < 0 ? -r3 : r3;
      int32_t a4 = r4 < 0 ? -r4 : r4;

      if (a1 > peak[0]) peak[0] = a1;
      if (a2 > peak[1]) peak[1] = a2;
      if (a3 > peak[2]) peak[2] = a3;
      if (a4 > peak[3]) peak[3] = a4;
    }
  }

  Serial.println("[I2S] Calibration peaks:");
  for (int i = 0; i < 4; ++i)
  {
    Serial.print(" mic"); Serial.print(i+1); Serial.print(": "); Serial.println(peak[i]);
  }

  int32_t max_peak = peak[0];
  for (int i = 1; i < 4; ++i) if (peak[i] > max_peak) max_peak = peak[i];

  if (max_peak == 0)
  {
    Serial.println("[I2S] No signal detected during calibration");
    return CAPSTONE_FAIL;
  }

  for (int i = 0; i < 4; ++i)
  {
    if (peak[i] == 0)
    {
      i2sData.calibration_gain[i] = 1.0f;
      Serial.print("[I2S] mic"); Serial.print(i+1); Serial.println(" peak==0; gain left at 1.0");
    }
    else
    {
      float gain = (float)max_peak / (float)peak[i];
      i2sData.calibration_gain[i] = gain;
      Serial.print("[I2S] mic"); Serial.print(i+1); Serial.print(" gain="); Serial.println(gain, 4);
    }
  }

  Serial.println("[I2S] Auto-calibration complete");
  return CAPSTONE_SUCCESS;
}

void setup_i2s(I2sPort_t port, int sck, int ws, int sd) 
{
  i2sConfig.is_config = true;
  i2sConfig.i2s_error |= I2S_OK;
  
  i2s_config_t cfg = 
  {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, /* capture full 24-bit INMP441 (MSB-aligned in 32-bit); each side; Frame = Right + Left = 64 bits */
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   // interleaved R then L
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = STEREO_FRAMES,//number of frames per DMA buffer
    .use_apll = false
  };

  i2s_pin_config_t pins = 
  {
    .bck_io_num   = sck,
    .ws_io_num    = ws,
    .data_out_num = I2S_PIN_NO_CHANGE,   // RX only
    .data_in_num  = sd
  };

  if(i2s_driver_install((i2s_port_t)port, &cfg, 0, NULL) != ESP_OK)
  {
    i2sConfig.is_config = true;
    i2sConfig.i2s_error |= I2S_DRIVER_INSTALL_ERR;
  }
  if(i2s_set_pin((i2s_port_t)port, &pins) != ESP_OK)
  {
    i2sConfig.is_config = false;
    i2sConfig.i2s_error |= I2S_SET_PINS_ERR;
  }
  if(i2s_start((i2s_port_t)port) != ESP_OK)
  {
    i2sConfig.is_config = false;
    i2sConfig.i2s_error |= I2S_START_ERR;
  }
  i2s_zero_dma_buffer((i2s_port_t)port);
}