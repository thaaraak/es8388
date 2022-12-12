/**
 * @file streams-i2s-i2s.ino
 * @brief Copy audio from I2S to I2S  - I2S uses 1 i2s port
 * @author Phil Schatzmann
 * @copyright GPLv3
 */

#include "AudioTools.h"
#include "es8388.h"
#include "Wire.h"

uint16_t sample_rate=40000;
uint16_t channels = 2;
uint16_t bits_per_sample = 16; // or try with 24 or 32
I2SStream i2s;
StreamCopy copier(i2s, i2s); // copies sound into i2s


// Arduino Setup
void setup(void) {  
  // Open Serial 
  Serial.begin(115200);
  // change to Warning to improve the quality
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

  // start I2S in
  Serial.println("starting I2S...");
  auto config = i2s.defaultConfig(RXTX_MODE);
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

  i2s.begin(config);

  Serial.println("I2S started...");
}

// Arduino loop - copy sound to out 
void loop() {
  copier.copy();
}
