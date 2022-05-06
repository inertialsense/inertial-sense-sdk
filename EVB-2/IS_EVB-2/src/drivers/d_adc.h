#ifndef D_ADC_H_
#define D_ADC_H_
#ifdef __cplusplus
extern "C" {
#endif

// defines
#define ADC_VOLT_REF     3300    // reference voltage for AFEC in mV
#define ADC_MAX_DIGITAL  4096UL  // the maximal digital value

// prototypes
int afec0_init(void);
int adc4_init(void);
float adc4_voltage(void);
int adc1_init(void);
float adc1_voltage(void);

#ifdef __cplusplus
}
#endif
#endif // D_ADC_H_
