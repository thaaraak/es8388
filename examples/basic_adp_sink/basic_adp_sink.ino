#include "BluetoothA2DPSink.h"

BluetoothA2DPSink a2dp_sink;

#include "BluetoothA2DPSink.h"
#include "es8388.h"

int bits_per_sample = 32;

void setup() {

    // Input/Output Modes
    es_dac_output_t output = (es_dac_output_t) ( DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2 );
    es_adc_input_t input = ADC_INPUT_LINPUT2_RINPUT2;
    //  es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;

    TwoWire wire(0);
    wire.setPins( 33, 32 );
  
    es8388 codec;
    codec.begin( &wire );
    codec.config( bits_per_sample, output, input, 70 );
    i2s_pin_config_t my_pin_config = {
        .bck_io_num = 27,
        .ws_io_num = 25,
        .data_out_num = 26,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_config_t i2s_config = {
      .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 44100, // updated automatically by A2DP
      .bits_per_sample = (i2s_bits_per_sample_t)bits_per_sample,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = true,
      .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
  };
    a2dp_sink.set_pin_config(my_pin_config);
    a2dp_sink.set_i2s_config(i2s_config);
    a2dp_sink.start("AG5LE");
}


void loop() {
}
