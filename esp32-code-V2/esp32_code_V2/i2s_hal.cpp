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
#define BP_FC_LOW   93.0f    // high-pass corner
#define BP_FC_HIGH  238.0f   // low-pass corner

static float bp_hp_a  = 0.0f;        // high-pass (via low-pass subtraction) coefficient
static float bp_lp_a  = 0.0f;        // low-pass coefficient

// State per mic (4 mics)
static float bp_hp_prev_lp[4]  = {0, 0, 0, 0};  // low-pass state used to make the high-pass
static float bp_lp_prev_out[4] = {0, 0, 0, 0};  // low-pass output after high-pass


// Add to your header file (i2s_hal.h)
#define SPEECH_RMS_THRESHOLD 12000.0f  // Adjust based on your calibration
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
static float hpf_prev_in[4]  = {0, 0, 0, 0};
static float hpf_prev_out[4] = {0, 0, 0, 0};

//static float lpf_prev_out[4] = {0, 0, 0, 0};

static inline int32_t I2sHal_BandPass_93_238(int mic_index, int32_t sample24)
{
  float x = (float)sample24;

  // --- 1) High-pass at ~93 Hz using "x - lowpass(x)" idea ---
  // Low-pass the input with cutoff at BP_FC_LOW (93 Hz)
  // This low-pass is used to track the slow/DC component
  float lp_dc = (1.0f - bp_hp_a) * x + bp_hp_a * bp_hp_prev_lp[mic_index];
  bp_hp_prev_lp[mic_index] = lp_dc;

  // High-pass output = input - its low-frequency (DC/slow) component
  float x_hp = x - lp_dc;

  // --- 2) Low-pass at ~228 Hz to cut everything above that ---
  // The state of the high-pass output is then low-pass filtered
  float y = (1.0f - bp_lp_a) * x_hp + bp_lp_a * bp_lp_prev_out[mic_index];
  bp_lp_prev_out[mic_index] = y;

  // Clamp to 24-bit-ish range for safety
  if (y >  8388607.0f)  y =  8388607.0f;
  if (y < -8388608.0f)  y = -8388608.0f;

  return (int32_t)y;
}

// Assume SAMPLE_RATE is defined elsewhere (e.g., #define SAMPLE_RATE 16000)
// This function needs to be added to calculate the coefficients
void I2sHal_CalculateFilterCoefficients(uint32_t sample_rate)
{
    // High-pass (via low-pass subtraction) for the low corner (BP_FC_LOW = 93 Hz)
    // The low-pass for subtraction (lp_dc) needs to pass low frequencies
    // The alpha for the "lp_dc" calculation needs to be for the cutoff 93 Hz
    float tau_low = 1.0f / (2.0f * M_PI * BP_FC_LOW);
    // bp_hp_a is the alpha for the low-pass part (lp_dc)
    bp_hp_a = tau_low / (tau_low + (1.0f / (float)sample_rate)); 
    
    // Low-pass for the high corner (BP_FC_HIGH = 238 Hz)
    float tau_high = 1.0f / (2.0f * M_PI * BP_FC_HIGH);
    // bp_lp_a is the alpha for the final low-pass
    bp_lp_a = tau_high / (tau_high + (1.0f / (float)sample_rate));
    
    // Serial.print("BP_HP_A: "); Serial.println(bp_hp_a, 6);
    // Serial.print("BP_LP_A: "); Serial.println(bp_lp_a, 6);
}

capstoneErrorCode_t I2sHal_Init(void)
{
  setup_i2s(I2S_PORT_0, I2S0_SCK, I2S0_WS, I2S0_SD);
  setup_i2s(I2S_PORT_1, I2S1_SCK, I2S1_WS, I2S1_SD);

  // Calculate filter coefficients based on the sample rate
  I2sHal_CalculateFilterCoefficients(SAMPLE_RATE);

  /* initialize calibration gains to 1.0 (no change) */
  for (int i = 0; i < 4; ++i) i2sData.calibration_gain[i] = 1.0f;
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
  int64_t acc_sq[4] = {0, 0, 0, 0};
  int64_t max_value = 0;
  int closest_mic = 0;

  for(size_t i = 0; i < frames_common; i++)
  {
    int32_t raw1 = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_0][i * 2 + 0]); // Left from Port 0 (24-bit)
    int32_t raw2 = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_0][i * 2 + 1]); // Right from Port 0 (24-bit)
    int32_t raw3 = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_1][i * 2 + 0]); // Left from Port 1 (24-bit)
    int32_t raw4 = I2sHal_Extract24BitSample(i2sData.buf_port[I2S_PORT_1][i * 2 + 1]); // Right from Port 1 (24-bit)

    int32_t mic1 = I2sHal_ApplyCalibration(raw1, i2sData.calibration_gain[0]);
    int32_t mic2 = I2sHal_ApplyCalibration(raw2, i2sData.calibration_gain[1]);
    int32_t mic3 = I2sHal_ApplyCalibration(raw3, i2sData.calibration_gain[2]);
    int32_t mic4 = I2sHal_ApplyCalibration(raw4, i2sData.calibration_gain[3]);

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
    mic1 = I2sHal_BandPass_93_238(0, mic1);
    mic2 = I2sHal_BandPass_93_238(1, mic2);
    mic3 = I2sHal_BandPass_93_238(2, mic3);
    mic4 = I2sHal_BandPass_93_238(3, mic4);

    if(mic1 < 2500 && mic1 >-2500 ){
      mic1 = 0;
    }
    if(mic2 < 2500 && mic2 >-2500 ){
      mic2 = 0;
    }
    if(mic3 < 2500 && mic3 >-2500 ){
      mic3 = 0;
    }
    if(mic4 < 2500 && mic4 >-2500 ){
      mic4 = 0;
    }

    acc_sq[0] = (int64_t)mic1 * (int64_t)mic1;
    acc_sq[1] = (int64_t)mic2 * (int64_t)mic2;
    acc_sq[2] = (int64_t)mic3 * (int64_t)mic3;
    acc_sq[3] = (int64_t)mic4 * (int64_t)mic4;
    max_value = acc_sq[0];
    closest_mic = 0;
    for (int m = 1; m < 4; ++m)
    {
      if (acc_sq[m] > max_value)
      {
        max_value = acc_sq[m];
        closest_mic = m;
      }
    }
    Serial.println(closest_mic);
    // Serial.print(mic1); Serial.print(' ');
    // Serial.print(mic2); Serial.print(' ');
    // Serial.print(mic3); Serial.print(' ');
    // Serial.println(mic4);
  }
  
  // // Compute RMS for each mic
  // float rms[4];
  // bool speech_detected[4] = {false, false, false, false};
  
  // for (int m = 0; m < 4; ++m)
  // {
  //   rms[m] = sqrtf((float)acc_sq[m] / (float)frames_common);
    
  //   // Check if RMS exceeds speech threshold
  //   if (rms[m] > SPEECH_RMS_THRESHOLD)
  //   {
  //     speech_detected[m] = true;
  //     //i2sData.speech_frame_count[m]++;
  //   }
  //   else
  //   {
  //     rms[m] = 0;
  //     //i2sData.speech_frame_count[m] = 0;  // Reset counter
  //    }
  // }

  // Serial.print(rms[0]); Serial.print(' ');
  // Serial.print(rms[1]); Serial.print(' ');
  // Serial.print(rms[2]); Serial.print(' ');
  // Serial.println(rms[3]);

  // // Find microphone with highest RMS that exceeds threshold
  // int closest_mic = -1;
  // float max_rms = SPEECH_RMS_THRESHOLD;
  
  // for (int m = 0; m < 4; ++m)
  // {
  //   if (i2sData.speech_frame_count[m] >= MIN_SPEECH_FRAMES && rms[m] > max_rms)
  //   {
  //     max_rms = rms[m];
  //     closest_mic = m;
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
  //   Serial.print(" -> Closest: mic"); Serial.println(closest_mic + 1);
  // }
  // else
  // {
  //   Serial.println(" -> No speech detected");
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