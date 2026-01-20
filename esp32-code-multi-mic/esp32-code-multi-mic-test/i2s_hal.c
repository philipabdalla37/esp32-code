/************************************************
*@file i2s_hal.c
*@created by Philip Abdalla
*@brief I2S Hardware Abstarction Layer
*/

/**************************INCLUDES*************************************************/
#include "i2s_hal.h"

/********************MACRO DEFINITIONS***********************************************/

/*************************Enumerations***********************************************/


/*************************Private Structs***********************************************/
typedef struct
{
  int32_t buf_port[I2S_PORT_TOTAL][STEREO_FRAMES * 2];
} I2sData_t;


/*************************Local Functions***********************************************/
void setup_i2s(I2sPort_t port, int sck, int ws, int sd);

/*************************Local Variables***********************************************/
I2sConfig_t i2sConfig = {0};
I2sData_t i2sData = {0};

capstoneErrorCode_t I2sHal_Init(void){
  setup_i2s(I2S_Port_0, I2S0_SCK, I2S0_WS, I2S0_SD);
  setup_i2s(I2S_Port_1, I2S1_SCK, I2S1_WS, I2S1_SD);
  i2sConfig.is_config = true;
  i2sConfig.i2s_error = I2S_OK;
}

// capstoneErrorCode_t I2sHal_Read(void);

void setup_i2s(I2sPort_t port, int sck, int ws, int sd) {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, /* the DAC module will only take the 8bits from MSB */
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   // interleaved R then L
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = STEREO_FRAMES,
    .use_apll = false
  };

  i2s_pin_config_t pins = {
    .bck_io_num   = sck,
    .ws_io_num    = ws,
    .data_out_num = I2S_PIN_NO_CHANGE,   // RX only
    .data_in_num  = sd
  };

  i2s_driver_install((i2s_port_t)port, &cfg, 0, NULL);
  i2s_set_pin((i2s_port_t)port, &pins);
  i2s_start((i2s_port_t)port);
}