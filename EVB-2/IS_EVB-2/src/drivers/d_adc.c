#include "asf.h"
#include "d_adc.h"


int afec0_init(void)
{
	static bool initialized = false;
	if (initialized)
		return 0;
	//Configure AFEC unit
	afec_enable(AFEC0);
	struct afec_config afec_cfg;
	afec_get_config_defaults(&afec_cfg);
	afec_init(AFEC0, &afec_cfg);
	afec_set_trigger(AFEC0, AFEC_TRIG_FREERUN);
	initialized = true;

	return 0;
}

int adc4_init(void)
{
	static bool initialized = false;
	if (initialized)
		return 0;
	//Configure ADC channel
	struct afec_ch_config afec_ch_cfg;
	afec_ch_cfg.diff = false;
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_4, &afec_ch_cfg);
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_4, 0x200);
	afec_channel_enable(AFEC0, AFEC_CHANNEL_4);

	
	//ioport_set_pin_input_mode(PIO_PE4_IDX, 0, 0);	
	//ioport_set_pin_input_mode(PIO_PD27_IDX, 0, 0);
	ioport_set_pin_dir(PIO_PE4_IDX, IOPORT_DIR_INPUT);
	
	initialized = true;

	return 0;
}

int adc1_init(void)
{
	static bool initialized = false;
	if (initialized)
		return 0;

	//Configure ADC channel
	struct afec_ch_config afec_ch_cfg;
	afec_ch_cfg.diff = false;
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_1, &afec_ch_cfg);
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_1, 0x200);
	afec_channel_enable(AFEC0, AFEC_CHANNEL_1);
//
	//ioport_set_pin_input_mode(PIO_PA16_IDX, 0, 0);	
	//ioport_set_pin_input_mode(PIO_PA21_IDX, 0, 0);
	ioport_set_pin_dir(PIO_PA21_IDX, IOPORT_DIR_INPUT);
	
	initialized = true;

	return 0;
}

float adc_voltage(Afec *const afec, enum afec_channel_num afec_ch)
{
	volatile uint32_t data = afec_channel_get_value(afec, afec_ch);
	return (float)data * ADC_VOLT_REF / ADC_MAX_DIGITAL / 1000.0f;
}
