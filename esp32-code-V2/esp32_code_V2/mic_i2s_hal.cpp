/************************************************
*@file mic_i2s_hal.c
*@created by Philip Abdalla
*@brief I2S Hardware Abstarction Layer
*/
/**************************INCLUDES*************************************************/
#include <Arduino.h>
#include "mic_i2s_hal.h"
#include "config_hal.h"
#include <driver/i2s_std.h>
#include <driver/gpio.h>

/*************************Private Structs***********************************************/
typedef struct {
    int32_t raw_buffer[I2S_PORT_TOTAL][STEREO_FRAMES * I2S_PORT_TOTAL];
    size_t bytes_read[I2S_PORT_TOTAL];
    
    float calibration_gain[MIC_TOTAL];
    I2sMics_t current_best_mic;
    
    unsigned long timer_start;
    unsigned long current_interval; // Variable: changes between 200ms and 3000ms
    double energy_accumulator[MIC_TOTAL]; 
    int samples_accumulated;        // To calculate average volume (RMS)
    
    int16_t processed_output[STEREO_FRAMES]; 

    float prev_in[MIC_TOTAL];
    float prev_out[MIC_TOTAL];
    
    i2s_chan_handle_t rx_handle[I2S_PORT_TOTAL]; 
} I2sData_t;

/*************************Local Variables***********************************************/
static I2sData_t i2sData = {0};
const float alpha = 0.962f;

// --- TUNING KNOB ---
// If the RMS volume is below this, we are in "Fast Scan" mode.
// If above this, we lock in for 3 seconds.
#define SILENCE_THRESHOLD 300.0f 

/*************************DSP Functions***********************************************/
int16_t ProcessSample(int mic_idx, int32_t raw_input) {
    float input_val = (float)(raw_input >> 14); 
    input_val *= i2sData.calibration_gain[mic_idx];
    float filtered = alpha * (i2sData.prev_out[mic_idx] + input_val - i2sData.prev_in[mic_idx]);
    i2sData.prev_in[mic_idx] = input_val;
    i2sData.prev_out[mic_idx] = filtered;
    if (filtered > 32767.0f) filtered = 32767.0f;
    if (filtered < -32768.0f) filtered = -32768.0f;
    return (int16_t)filtered;
}

/*************************Public I2S Functions***********************************************/

capstoneErrorCode_t I2sHal_Init(void) {
    for(int i=0; i<4; i++) i2sData.calibration_gain[i] = 1.0f;
    i2sData.current_best_mic = MIC_1;
    
    // Initialize Logic
    i2sData.timer_start = millis();
    i2sData.current_interval = 200; // Start in FAST mode (listening for start of speech)
    i2sData.samples_accumulated = 0;
    memset(i2sData.energy_accumulator, 0, sizeof(i2sData.energy_accumulator));

    // 1. Define General Config
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_NUM_DMA;
    chan_cfg.dma_frame_num = STEREO_FRAMES;

    // 2. Define Audio Standard (Philips 16kHz)
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = { .mclk = I2S_GPIO_UNUSED, .dout = I2S_GPIO_UNUSED, .invert_flags = {0} },
    };

    // 3. Setup Port 0
    std_cfg.gpio_cfg.bclk = I2S0_SCK;
    std_cfg.gpio_cfg.ws   = I2S0_WS;
    std_cfg.gpio_cfg.din  = I2S0_SD;
    if (i2s_new_channel(&chan_cfg, NULL, &i2sData.rx_handle[I2S_PORT_0]) != ESP_OK) return CAPSTONE_FAIL;
    if (i2s_channel_init_std_mode(i2sData.rx_handle[I2S_PORT_0], &std_cfg) != ESP_OK) return CAPSTONE_FAIL;
    if (i2s_channel_enable(i2sData.rx_handle[I2S_PORT_0]) != ESP_OK) return CAPSTONE_FAIL;

    // 4. Setup Port 1
    std_cfg.gpio_cfg.bclk = I2S1_SCK;
    std_cfg.gpio_cfg.ws   = I2S1_WS;
    std_cfg.gpio_cfg.din  = I2S1_SD;
    if (i2s_new_channel(&chan_cfg, NULL, &i2sData.rx_handle[I2S_PORT_1]) != ESP_OK) return CAPSTONE_FAIL;
    if (i2s_channel_init_std_mode(i2sData.rx_handle[I2S_PORT_1], &std_cfg) != ESP_OK) return CAPSTONE_FAIL;
    if (i2s_channel_enable(i2sData.rx_handle[I2S_PORT_1]) != ESP_OK) return CAPSTONE_FAIL;

    return CAPSTONE_SUCCESS;
}

capstoneErrorCode_t I2sHal_Run(void) {
    // 1. Read Data
    if (i2s_channel_read(i2sData.rx_handle[I2S_PORT_0], i2sData.raw_buffer[I2S_PORT_0], sizeof(i2sData.raw_buffer[I2S_PORT_0]), &i2sData.bytes_read[I2S_PORT_0], 1000) != ESP_OK) return CAPSTONE_FAIL;
    if (i2s_channel_read(i2sData.rx_handle[I2S_PORT_1], i2sData.raw_buffer[I2S_PORT_1], sizeof(i2sData.raw_buffer[I2S_PORT_1]), &i2sData.bytes_read[I2S_PORT_1], 1000) != ESP_OK) return CAPSTONE_FAIL;

    int samples_read = i2sData.bytes_read[0] / 4; 
    int frames = samples_read / 2; 
    if (frames == 0) return CAPSTONE_FAIL;

    // 2. Process Audio
    for (int i = 0; i < frames; i++) {
        int32_t r1 = i2sData.raw_buffer[I2S_PORT_0][i*2];   
        int32_t r2 = i2sData.raw_buffer[I2S_PORT_0][i*2 + 1]; 
        int32_t r3 = i2sData.raw_buffer[I2S_PORT_1][i*2];    
        int32_t r4 = i2sData.raw_buffer[I2S_PORT_1][i*2 + 1]; 

        int16_t s1 = ProcessSample(MIC_1, r1);
        int16_t s2 = ProcessSample(MIC_2, r2);
        int16_t s3 = ProcessSample(MIC_3, r3);
        int16_t s4 = ProcessSample(MIC_4, r4);

        // Accumulate Energy (Votes)
        i2sData.energy_accumulator[MIC_1] += (double)s1 * s1;
        i2sData.energy_accumulator[MIC_2] += (double)s2 * s2;
        i2sData.energy_accumulator[MIC_3] += (double)s3 * s3;
        i2sData.energy_accumulator[MIC_4] += (double)s4 * s4;
        
        // Output Mux
        switch(i2sData.current_best_mic) {
            case MIC_1: i2sData.processed_output[i] = s1; break;
            case MIC_2: i2sData.processed_output[i] = s2; break;
            case MIC_3: i2sData.processed_output[i] = s3; break;
            case MIC_4: i2sData.processed_output[i] = s4; break;
        }
    }
    i2sData.samples_accumulated += frames;

    // 3. SMART TIMER LOGIC
    // We check against 'current_interval' which changes dynamically
    if (millis() - i2sData.timer_start >= i2sData.current_interval) {
        
        double max_energy = 0;
        int new_winner = i2sData.current_best_mic;

        // A. Find the winner
        for(int k=0; k<MIC_TOTAL; k++) {
            if (i2sData.energy_accumulator[k] > max_energy) {
                max_energy = i2sData.energy_accumulator[k];
                new_winner = k;
            }
        }

        // B. Calculate RMS Volume (Average Loudness)
        // We need to know if this is silence or speech
        double mean_square = max_energy / i2sData.samples_accumulated;
        float rms = sqrt(mean_square);

        // C. Decide the Interval for NEXT time
        if (rms > SILENCE_THRESHOLD) {
            // SPEECH DETECTED! 
            // Lock onto this winner for 3 seconds to ensure stability.
            i2sData.current_best_mic = (I2sMics_t)new_winner;
            i2sData.current_interval = 3000; 
        } else {
            // SILENCE DETECTED.
            // Stay on the winner, but check again very soon (200ms).
            // This allows us to react instantly when you finally start talking.
            i2sData.current_best_mic = (I2sMics_t)new_winner;
            i2sData.current_interval = 200; 
        }

        // D. Reset for next round
        memset(i2sData.energy_accumulator, 0, sizeof(i2sData.energy_accumulator));
        i2sData.samples_accumulated = 0;
        i2sData.timer_start = millis();
    }

    return CAPSTONE_SUCCESS;
}

int16_t* I2sHal_GetBuffer(void) {
    return i2sData.processed_output;
}

I2sMics_t I2sHal_GetClosestMic(void) {
    return i2sData.current_best_mic;
}