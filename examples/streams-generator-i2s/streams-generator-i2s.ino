/**
 * @file streams-generator-i2s.ino
 * @author Phil Schatzmann
 * @brief see https://github.com/pschatzmann/arduino-audio-tools/blob/main/examples/examples-stream/streams-generator-i2s/README.md 
 * @copyright GPLv3
 */
 
#include "AudioTools.h"
#include "es8388.h"

uint16_t sample_rate=22000;
uint8_t channels = 2;                                      // The stream will have 2 channels 
SineWaveGenerator<int16_t> sineWave(20000);                // subclass of SoundGenerator with max amplitude of 32000
GeneratedSoundStream<int16_t> sound(sineWave);             // Stream generated from sine wave
I2SStream out; 
StreamCopy copier(out, sound);                             // copies sound into i2s

uint16_t bits_per_sample = 16; // or try with 24 or 32

// Arduino Setup
void setup(void) {  
  // Open Serial 
  Serial.begin(115200);
  while(!Serial);
  AudioLogger::instance().begin(Serial, AudioLogger::Error);

  // Input/Output Modes
  es_dac_output_t output = (es_dac_output_t) ( DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2 );
  es_adc_input_t input = ADC_INPUT_LINPUT2_RINPUT2;
  //  es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;

  TwoWire wire(0);
  wire.setPins( 4, 15 );
  
  es8388 codec;
  codec.begin( &wire );
  codec.config( bits_per_sample, output, input, 90 );
  
  // start I2S
  Serial.println("starting I2S...");
  auto config = out.defaultConfig(TX_MODE);
  config.sample_rate = sample_rate; 
  config.bits_per_sample = bits_per_sample; 
  config.channels = 2;
  config.i2s_format = I2S_STD_FORMAT;
  config.pin_ws = 18;
  config.pin_bck = 5;
  config.pin_data = 17;
  config.pin_data_rx = 16;
  //config.fixed_mclk = 0;
  config.pin_mck = 3;
  out.begin(config);

  // Setup sine wave
  sineWave.begin(channels, sample_rate, 500);
  Serial.println("started...");
}

// Arduino loop - copy sound to out 
void loop() {
  copier.copy();
}
