#ifndef ADC_H
#define ADC_H

void adc_init(void);
void adc_start(void);
char adc_running(void);
uint8_t adc_value(void);
void adc_select(uint8_t pin);
uint8_t adc_read(uint8_t pin);
#endif
