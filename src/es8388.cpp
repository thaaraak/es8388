#include "es8388.h"

#define ES8388_ADDR 0b0010000

static const char *ES_TAG = "ES8388_DRIVER";


es8388::es8388()
{
}

bool es8388::begin( TwoWire *theWire )
{
	  if (i2c_dev)
	    delete i2c_dev;
	  i2c_dev = new Adafruit_I2CDevice(ES8388_ADDR, theWire);
	  if (!i2c_dev->begin())
	    return false;

	  return true;

}

bool es8388::i2c_write( uint8_t reg, uint8_t value)
{
	uint8_t buffer[2] = {reg, value};
	if (i2c_dev->write(buffer, 2)) {
	    return true;
	} else {
	    return false;
	}
}


bool es8388::i2c_read( uint8_t reg, uint8_t* value )
{
	if (i2c_dev->write_then_read(&reg, 1, value, 1)) {
	    return true;
	} else {
	    return false;
	}
}

bool es8388::es_write_reg(uint8_t reg_add, uint8_t data)
{
    return i2c_write( reg_add, data );
}

bool es8388::es_read_reg(uint8_t reg_add, uint8_t *p_data)
{
    return i2c_read( reg_add, p_data );
}

void es8388::read_all()
{
    for (int i = 0; i < 50; i++) {
        uint8_t reg = 0;
        es_read_reg(i, &reg);
    }
}

int es8388::set_adc_dac_volume(int mode, int volume, int dot)
{
    int res = 0;
    if ( volume < -96 || volume > 0 ) {
        if (volume < -96)
            volume = -96;
        else
            volume = 0;
    }
    dot = (dot >= 5 ? 1 : 0);
    volume = (-volume << 1) + dot;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADCCONTROL8, volume);
        res |= es_write_reg(ES8388_ADCCONTROL9, volume);  //ADC Right Volume=0db
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_DACCONTROL5, volume);
        res |= es_write_reg(ES8388_DACCONTROL4, volume);
    }
    return res;
}

bool es8388::init( es_dac_output_t output, es_adc_input_t input )
{
    int res = 0;

    res |= es_write_reg(ES8388_DACCONTROL3, 0x04);  // 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp

    res |= es_write_reg(ES8388_CONTROL2, 0x50);
    res |= es_write_reg(ES8388_CHIPPOWER, 0x00); //normal all and power up all
    res |= es_write_reg(ES8388_MASTERMODE, ES_MODE_SLAVE ); //CODEC IN I2S SLAVE MODE

    res |= es_write_reg(ES8388_DACPOWER, 0xC0);  //disable DAC and disable Lout/Rout/1/2
    res |= es_write_reg(ES8388_CONTROL1, 0x12);  //Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
    res |= es_write_reg(ES8388_DACCONTROL1, 0x18);//1a 0x18:16bit iis , 0x00:24
    res |= es_write_reg(ES8388_DACCONTROL2, 0x02);  //DACFsMode,SINGLE SPEED; DACFsRatio,256
    res |= es_write_reg(ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
    res |= es_write_reg(ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
    res |= es_write_reg(ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
    res |= es_write_reg(ES8388_DACCONTROL21, 0x80); //set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
    res |= es_write_reg(ES8388_DACCONTROL23, 0x00);   //vroi=0
    res |= set_adc_dac_volume(ES_MODULE_DAC, 0, 0);          // 0db

    res |= es_write_reg(ES8388_DACPOWER, output );
    res |= es_write_reg(ES8388_ADCPOWER, 0xFF);
    res |= es_write_reg(ES8388_ADCCONTROL1, 0x11); // MIC Left and Right channel PGA gain


    res |= es_write_reg(ES8388_ADCCONTROL2, input);

    res |= es_write_reg(ES8388_ADCCONTROL3, 0x02);
    res |= es_write_reg(ES8388_ADCCONTROL4, 0x0d); // Left/Right data, Left/Right justified mode, Bits length, I2S format
    res |= es_write_reg(ES8388_ADCCONTROL5, 0x02);  //ADCFsMode,singel SPEED,RATIO=256
    //ALC for Microphone
    res |= set_adc_dac_volume(ES_MODULE_ADC, 0, 0);      // 0db
    res |= es_write_reg(ES8388_ADCPOWER, 0x09); //Power on ADC, Enable LIN&RIN, Power off MICBIAS, set int1lp to low power mode

    return res;
}

// This function sets the I2S format which can be one of
//		I2S_NORMAL
//		I2S_LEFT		Left Justified
//		I2S_RIGHT,      Right Justified
//		I2S_DSP,        dsp/pcm format
//
// and the bits per sample which must be one of
//		BIT_LENGTH_16BITS
//		BIT_LENGTH_18BITS
//		BIT_LENGTH_20BITS
//		BIT_LENGTH_24BITS
//		BIT_LENGTH_32BITS
//
// Note the above must match the ESP-IDF I2S configuration which is set separately

bool es8388::config_i2s( es_bits_length_t bits_length, es_module_t mode, es_format_t fmt )
{
    bool res = ESP_OK;
    uint8_t reg = 0;

    // Set the Format
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S ADC Format\n");
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xfc;
        res |= es_write_reg(ES8388_ADCCONTROL4, reg | fmt);
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S DAC Format\n");
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xf9;
        res |= es_write_reg(ES8388_DACCONTROL1, reg | (fmt << 1));
    }


    // Set the Sample bits length
    int bits = (int)bits_length;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S ADC Bits: %d\n", bits);
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xe3;
        res |=  es_write_reg(ES8388_ADCCONTROL4, reg | (bits << 2));
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        ESP_LOGE(ES_TAG, "Setting I2S DAC Bits: %d\n", bits);
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xc7;
        res |= es_write_reg(ES8388_DACCONTROL1, reg | (bits << 3));
    }
    return res;
}


bool es8388::set_voice_mute(bool enable)
{
    bool res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    reg = reg & 0xFB;
    res |= es_write_reg(ES8388_DACCONTROL3, reg | (((int)enable) << 2));
    return res;
}

bool es8388::start(es_module_t mode)
{
    bool res = ESP_OK;
    uint8_t prev_data = 0, data = 0;
    es_read_reg(ES8388_DACCONTROL21, &prev_data);
    if (mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_DACCONTROL16, 0x09); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2 by pass enable
        res |= es_write_reg(ES8388_DACCONTROL17, 0x50); // left DAC to left mixer enable  and  LIN signal to left mixer enable 0db  : bupass enable
        res |= es_write_reg(ES8388_DACCONTROL20, 0x50); // right DAC to right mixer enable  and  LIN signal to right mixer enable 0db : bupass enable
        res |= es_write_reg(ES8388_DACCONTROL21, 0xC0); //enable adc
    } else {
        res |= es_write_reg(ES8388_DACCONTROL21, 0x80);   //enable dac
    }
    es_read_reg(ES8388_DACCONTROL21, &data);
    if (prev_data != data) {
    	printf( "Resetting State Machine\n");

        res |= es_write_reg(ES8388_CHIPPOWER, 0xF0);   //start state machine
        // res |= es_write_reg(ES8388_CONTROL1, 0x16);
        // res |= es_write_reg(ES8388_CONTROL2, 0x50);
        res |= es_write_reg(ES8388_CHIPPOWER, 0x00);   //start state machine
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	printf( "Powering up ADC\n");
        res |= es_write_reg(ES8388_ADCPOWER, 0x00);   //power up adc and line in
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	printf( "Powering up DAC\n");
        res |= es_write_reg(ES8388_DACPOWER, 0x3c);   //power up dac and line out
        res |= set_voice_mute(false);
    }

    return res;
}


bool es8388::set_voice_volume(int volume)
{
    bool res = ESP_OK;
    if (volume < 0)
        volume = 0;
    else if (volume > 100)
        volume = 100;
    volume /= 3;
    res = es_write_reg(ES8388_DACCONTROL24, volume);
    res |= es_write_reg(ES8388_DACCONTROL25, volume);
    res |= es_write_reg(ES8388_DACCONTROL26, volume);
    res |= es_write_reg(ES8388_DACCONTROL27, volume);
    return res;
}


void es8388::config( int bits, es_dac_output_t output, es_adc_input_t input, int volume )
{

    init( output, input );

    es_bits_length_t bits_length;

    if ( bits == 16 )
    	bits_length = BIT_LENGTH_16BITS;
    else if ( bits == 24 )
    	bits_length = BIT_LENGTH_24BITS;
    else if ( bits == 32 )
    	bits_length = BIT_LENGTH_32BITS;


    es_module_t module = ES_MODULE_ADC_DAC;
    es_format_t fmt = I2S_NORMAL;

    config_i2s( bits_length, ES_MODULE_ADC_DAC, fmt );
    set_voice_volume( volume );
    start( module );

}
