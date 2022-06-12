/**
 * @file streams-i2s-a2dp.ino
 * @author Phil Schatzmann
 * @brief see https://github.com/pschatzmann/arduino-audio-tools/blob/main/examples/examples-stream/streams-i2s-a2dp/README.md
 * 
 * @author Phil Schatzmann
 * @copyright GPLv3
 */

#include "AudioTools.h"
#include "AudioLibs/AudioA2DP.h"
#include "es8388.h"


I2SStream i2sStream;                            // Access I2S as stream
A2DPStream a2dpStream = A2DPStream::instance(); // access A2DP as stream
//VolumeStream volume(i2sStream);
StreamCopy copier(i2sStream, a2dpStream); // copy i2sStream to a2dpStream
ConverterFillLeftAndRight<int16_t> filler(LeftIsEmpty); // fill both channels

uint16_t sample_rate=44100;
uint8_t channels = 2;   
uint16_t bits_per_sample = 16;

// Arduino Setup
void setup(void) {
    Serial.begin(115200);
    AudioLogger::instance().begin(Serial, AudioLogger::Error);

    // set intial volume
    //volume.setVolume(0.3);
    
    // start bluetooth
    Serial.println("starting A2DP...");
    auto cfgA2DP = a2dpStream.defaultConfig(RX_MODE);
    cfgA2DP.name = "NA5Y";
    a2dpStream.begin(cfgA2DP);

    // start i2s input with default configuration
    Serial.println("starting I2S...");

    // Input/Output Modes
    es_dac_output_t output = (es_dac_output_t) ( DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2 );
    es_adc_input_t input = ADC_INPUT_LINPUT2_RINPUT2;
    //  es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;

    TwoWire wire(0);
    wire.setPins( 33, 32 );
  
    es8388 codec;
    codec.begin( &wire );
    codec.config( bits_per_sample, output, input, 90 );
  
    // start I2S
    Serial.println("starting I2S...");
    auto config = i2sStream.defaultConfig(RXTX_MODE);
    config.sample_rate = sample_rate; 
    config.bits_per_sample = bits_per_sample; 
    config.channels = 2;
    config.i2s_format = I2S_STD_FORMAT;
    config.pin_ws = 25;
    config.pin_bck = 27;
    config.pin_data = 26;
    config.pin_data_rx = 35;
    //config.fixed_mclk = 0;
    config.pin_mck = 0;
    config.buffer_count = 8,
    config.buffer_size = 64,

/*
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = true,
      .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
*/
    
    a2dpStream.setNotifyAudioChange(i2sStream); // i2s is using the info from a2dp
    i2sStream.begin(config);

}

// Arduino loop - copy data
void loop() {
   // copier.copy(filler);
    copier.copy();
}
