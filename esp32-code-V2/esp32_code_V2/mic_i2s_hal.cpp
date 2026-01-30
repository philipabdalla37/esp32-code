/************************************************
*@file mic_i2s_hal.c
*@created by Philip Abdalla
*@brief I2S Hardware Abstraction Layer (New I2S STD Mode API)
*/

/**************************INCLUDES*************************************************/
#include <Arduino.h>
#include "mic_i2s_hal.h"
#include "config_hal.h"
#include <driver/i2s_std.h>
#include <driver/gpio.h>

/********************MACRO DEFINITIONS***********************************************/


/*************************Private Structs***********************************************/
typedef struct {
    int32_t raw_buffer[I2S_PORT_TOTAL][STEREO_FRAMES * I2S_PORT_TOTAL]; // [Port][Samples]
    size_t bytes_read[I2S_PORT_TOTAL];
    
    // Logic State
    float calibration_gain[MIC_TOTAL];
    I2sMics_t current_best_mic;
    int hysteresis_count;

    unsigned long timer_start;          // To track the 3000ms
    double energy_accumulator[MIC_TOTAL]; // To keep score during the 3000ms
    
    // The "Final Output" buffer ready for Speech AI (16-bit Mono)
    int16_t processed_output[STEREO_FRAMES]; 

    float prev_in[MIC_TOTAL];
    float prev_out[MIC_TOTAL];
    
    // Driver Handles
    i2s_chan_handle_t rx_handle[I2S_PORT_TOTAL]; 
} I2sData_t;


// typedef struct
// {
//   i2s_chan_handle_t rx_handle[I2S_PORT_TOTAL];  /* Channel handles for I2S0 and I2S1 */
//   int32_t buf_port[I2S_PORT_TOTAL][STEREO_FRAMES * I2S_PORT_TOTAL];
//   size_t buf_bytes_read[I2S_PORT_TOTAL];
//   uint32_t i2s_error;
//   float calibration_gain[4]; /* per-microphone gain factors (mic1..mic4) */
//   uint8_t speech_frame_count[4];  // Track consecutive speech frames per mic
//   int8_t prev_closest_mic;        // Previously-detected loud mic (-1 = none)
//   int current_best_mic; // The Mic ID (0-3) currently selected as the "Winner"
//   int hysteresis_count; // Counter: How many frames has a challenger been louder?
// } I2sData_t;

/*************************Local Functions***********************************************/

/*************************Local Variables***********************************************/
static I2sConfig_t i2sConfig = {0}; // Used only for Init
static I2sData_t i2sData = {0};

// Alpha for 100Hz High Pass at 16kHz: alpha = 1 / (1 + 2*PI*dt*fc)
// dt = 1/16000, fc = 100 -> alpha approx 0.962
const float alpha = 0.962f;

/*************************DSP Functions***********************************************/
// --- HELPER: DC Blocker & Speech Conversion ---
// Converts 32-bit I2S data -> Filtered -> 16-bit Speech Safe Audio
int16_t ProcessSample(int mic_idx, int32_t raw_input) {
    // A. Shift: Convert 24-bit (in 32-bit box) to roughly 16-bit levels
    // Shift 14 gives a Volume Boost (Gain). Standard is 16.
    float input_val = (float)(raw_input >> 14); 

    // B. Apply Calibration
    input_val *= i2sData.calibration_gain[mic_idx];

    // C. High Pass Filter (DC Blocker)
    // Algorithm: Output = alpha * (Previous_Output + Input - Previous_Input)
    float filtered = alpha * (i2sData.prev_out[mic_idx] + input_val - i2sData.prev_in[mic_idx]);
    
    // Update History for next time
    i2sData.prev_in[mic_idx] = input_val;
    i2sData.prev_out[mic_idx] = filtered;

    // D. Safety Clipper (Hard Limit)
    // Prevents "Integer Overflow" crackling sounds
    if (filtered > 32767.0f) filtered = 32767.0f; //2^16
    if (filtered < -32768.0f) filtered = -32768.0f;

    return (int16_t)filtered;
}
/*************************I2S Setup (New API)***********************************************/

/*************************Public I2S Functions***********************************************/

// --- INIT FUNCTION ---
capstoneErrorCode_t I2sHal_Init(void) {
    // initialize logic
    for(int i=0; i<4; i++) i2sData.calibration_gain[i] = 1.0f;
    i2sData.current_best_mic = MIC_1;
    i2sData.timer_start = millis(); // <--- ADD THIS LINE
memset(i2sData.energy_accumulator, 0, sizeof(i2sData.energy_accumulator)); // <--- ADD THIS LINE

    // 1. Define General Config
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_NUM_DMA;            // Number of "buckets"
    chan_cfg.dma_frame_num = STEREO_FRAMES; // Size of bucket (64)

    // 2. Define Audio Standard (Philips 16kHz)
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .dout = I2S_GPIO_UNUSED,
            .invert_flags = {0},
        },
    };

    // 3. Setup Port 0 (Mics 1 & 2)
    std_cfg.gpio_cfg.bclk = I2S0_SCK;
    std_cfg.gpio_cfg.ws   = I2S0_WS;
    std_cfg.gpio_cfg.din  = I2S0_SD;
    
    if (i2s_new_channel(&chan_cfg, NULL, &i2sData.rx_handle[I2S_PORT_0]) != ESP_OK) return CAPSTONE_FAIL;
    if (i2s_channel_init_std_mode(i2sData.rx_handle[I2S_PORT_0], &std_cfg) != ESP_OK) return CAPSTONE_FAIL;
    if (i2s_channel_enable(i2sData.rx_handle[I2S_PORT_0]) != ESP_OK) return CAPSTONE_FAIL;

    // 4. Setup Port 1 (Mics 3 & 4)
    std_cfg.gpio_cfg.bclk = I2S1_SCK;
    std_cfg.gpio_cfg.ws   = I2S1_WS;
    std_cfg.gpio_cfg.din  = I2S1_SD;

    if (i2s_new_channel(&chan_cfg, NULL, &i2sData.rx_handle[I2S_PORT_1]) != ESP_OK) return CAPSTONE_FAIL;
    if (i2s_channel_init_std_mode(i2sData.rx_handle[I2S_PORT_1], &std_cfg) != ESP_OK) return CAPSTONE_FAIL;
    if (i2s_channel_enable(i2sData.rx_handle[I2S_PORT_1]) != ESP_OK) return CAPSTONE_FAIL;

    return CAPSTONE_SUCCESS;
}

capstoneErrorCode_t I2sHal_Run(void) {
    // 1. Read Raw Data (Blocks until 64 frames are ready)
    // We read into our struct's raw_buffer
    if (i2s_channel_read(i2sData.rx_handle[I2S_PORT_0], i2sData.raw_buffer[I2S_PORT_0], sizeof(i2sData.raw_buffer[I2S_PORT_0]), &i2sData.bytes_read[I2S_PORT_0], 1000) != ESP_OK) return CAPSTONE_FAIL;
    if (i2s_channel_read(i2sData.rx_handle[I2S_PORT_1], i2sData.raw_buffer[I2S_PORT_1], sizeof(i2sData.raw_buffer[I2S_PORT_1]), &i2sData.bytes_read[I2S_PORT_1], 1000) != ESP_OK) return CAPSTONE_FAIL;

    // Safety: Ensure we actually got data
    int samples_read = i2sData.bytes_read[0] / 4; 
    int frames = samples_read / 2; // Should be 64
    if (frames == 0) return CAPSTONE_FAIL;

    //double sum_sq[MIC_TOTAL] = {0};

    // 2. Process Every Frame
    for (int i = 0; i < frames; i++) {
        // A. Extract Interleaved Data
        int32_t r1 = i2sData.raw_buffer[I2S_PORT_0][i*2];     // Left (Mic 1)
        int32_t r2 = i2sData.raw_buffer[I2S_PORT_0][i*2 + 1]; // Right (Mic 2)
        int32_t r3 = i2sData.raw_buffer[I2S_PORT_1][i*2];     // Left (Mic 3)
        int32_t r4 = i2sData.raw_buffer[I2S_PORT_1][i*2 + 1]; // Right (Mic 4)

        // B. Filter & Normalize
        int16_t s1 = ProcessSample(MIC_1, r1);
        int16_t s2 = ProcessSample(MIC_2, r2);
        int16_t s3 = ProcessSample(MIC_3, r3);
        int16_t s4 = ProcessSample(MIC_4, r4);

        // C. ACCUMULATE ENERGY (Keeping Score)
        // We add the energy to the running total for this 3-second window
        i2sData.energy_accumulator[MIC_1] += (double)s1 * s1;
        i2sData.energy_accumulator[MIC_2] += (double)s2 * s2;
        i2sData.energy_accumulator[MIC_3] += (double)s3 * s3;
        i2sData.energy_accumulator[MIC_4] += (double)s4 * s4;

        // // C. Calculate Energy (RMS) for Decision Logic
        // sum_sq[MIC_1] += (double)s1 * s1;
        // sum_sq[MIC_2] += (double)s2 * s2;
        // sum_sq[MIC_3] += (double)s3 * s3;
        // sum_sq[MIC_4] += (double)s4 * s4;

        // D. Save "Winner" Audio for Output
        // This acts as a digital switch (multiplexer)
        switch(i2sData.current_best_mic) {
            case MIC_1: i2sData.processed_output[i] = s1; break;
            case MIC_2: i2sData.processed_output[i] = s2; break;
            case MIC_3: i2sData.processed_output[i] = s3; break;
            case MIC_4: i2sData.processed_output[i] = s4; break;
        }
    }

    // // 3. Logic: Who is the loudest? (Sticky/Hysteresis)
    // float rms[MIC_TOTAL];
    // int frame_loudest = MIC_1;
    // float max_vol = 0;

    // for(int k=0; k<MIC_TOTAL; k++) {
    //     rms[k] = sqrt(sum_sq[k] / frames);
    //     if (rms[k] > max_vol) {
    //         max_vol = rms[k];
    //         frame_loudest = k;
    //     }
    // }

    // "Sticky" Switching Logic
    // Only switch if:
    // 1. Volume is above silence threshold (500)
    // 2. Challenger is 20% louder than current champion
    // 3. Challenger wins for 3 consecutive cycles (debounce)
    // if (max_vol > 500.0f) {
    //     if (frame_loudest != i2sData.current_best_mic) {
    //         if (rms[frame_loudest] > (rms[i2sData.current_best_mic] * 1.20f)) {
    //             i2sData.hysteresis_count++;
    //             if (i2sData.hysteresis_count >= 3) { // 3 cycles * 4ms = 12ms reaction
    //                 i2sData.current_best_mic = (I2sMics_t)frame_loudest;
    //                 i2sData.hysteresis_count = 0;
    //             }
    //         } else {
    //             i2sData.hysteresis_count = 0;
    //         }
    //     }
    // }

    // // This helps the AI realize nobody is talking.
    // bool is_silence = (max_vol < 300.0f); // Adjust this threshold based on your room

    // // --- NEW: Switching Logic (Reduced Clicking) ---
    // if (!is_silence) {
    //     if (frame_loudest != i2sData.current_best_mic) {
    //         // Only switch if the new mic is significantly louder (20%)
    //         if (rms[frame_loudest] > (rms[i2sData.current_best_mic] * 1.2f)) {
    //              i2sData.hysteresis_count++;
    //              if (i2sData.hysteresis_count >= 3) {
    //                  i2sData.current_best_mic = (I2sMics_t)frame_loudest;
    //                  i2sData.hysteresis_count = 0;
    //              }
    //         } else {
    //             i2sData.hysteresis_count = 0;
    //         }
    //     }
    // }

    // // --- APPLY GATE TO BUFFER ---
    // // If we determined it's silent, overwrite the output buffer with Zeros.
    // if (is_silence) {
    //     memset(i2sData.processed_output, 0, sizeof(i2sData.processed_output));
    // }

    // 3. THE TIMER LOGIC
    // Only check the winner if 3000ms has passed
    if (millis() - i2sData.timer_start >= 3000) {
        
        double max_energy = 0;
        int new_winner = i2sData.current_best_mic; // Default to staying put

        // Find who had the highest total score over the last 3 seconds
        for(int k=0; k<MIC_TOTAL; k++) {
            if (i2sData.energy_accumulator[k] > max_energy) {
                max_energy = i2sData.energy_accumulator[k];
                new_winner = k;
            }
        }

        // Silence Check (optional): If total energy is super low, don't switch randomly
        // You can remove this 'if' if you want it to always switch to the loudest noise (even floor noise)
        if (max_energy > (500.0 * 500.0 * frames * 10)) { // Rough heuristic for silence
             i2sData.current_best_mic = (I2sMics_t)new_winner;
        }

        // Reset for the next 3000ms round
        memset(i2sData.energy_accumulator, 0, sizeof(i2sData.energy_accumulator));
        i2sData.timer_start = millis();
    }

    return CAPSTONE_SUCCESS;
}

/**
 * @brief  Retrieve the latest 16-bit audio buffer (64 samples).
 * Call this immediately after I2sHal_Run() returns SUCCESS.
 * @return Pointer to the internal processed buffer.
 */
int16_t* I2sHal_GetBuffer(void) {
    return i2sData.processed_output;
}

/**
 * @brief  Returns the ID of the microphone currently receiving the loudest audio.
 * @return 0 = Mic 1, 1 = Mic 2, 2 = Mic 3, 3 = Mic 4
 */
I2sMics_t I2sHal_GetClosestMic(void) {
    return i2sData.current_best_mic;
}
