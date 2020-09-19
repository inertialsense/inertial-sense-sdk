#ifndef D_ADC_H_
#define D_ADC_H_
#ifdef __cplusplus
extern "C" {
#endif

// defines
#define ADC_VOLT_REF     3300    // reference voltage for AFEC in mV
#define ADC_MAX_DIGITAL  4096UL  // the maximal digital value

// prototypes
int adc_init(void);
float adc_voltage(void);

#ifdef __cplusplus
}
#endif
#endif // D_ADC_H_
