#include "adc.h"
#include <avr/io.h>
#include "misc.h"

// initialize the adc
void adc_init(void)
{
  // left adjust data in register
  SETBIT1(ADMUX, ADLAR);
  // AVcc as reference
  SETBIT0(ADMUX, REFS1);
  SETBIT1(ADMUX, REFS0);
  // set adc prescaler to 64
  SETBIT1(ADCSRA, ADPS2);
  SETBIT1(ADCSRA, ADPS1);
  SETBIT0(ADCSRA, ADPS0);
  // enable adc
  SETBIT1(ADCSRA, ADEN);
}

// set adc pin
void adc_select(uint8_t pin) { ADMUX = (ADMUX & 0xF0) | (pin & 0x0F); }
// start adc conversion
void adc_start(void) { SETBIT1(ADCSRA, ADSC); }
// is conversion running
char adc_running(void) { return GETBIT(ADCSRA, ADSC); }
// read adc value
uint8_t adc_value(void) { return ADCH; }
uint8_t adc_read(uint8_t pin)
{
  adc_select(pin);
  adc_start();
  while (adc_running()) {
  }
  return adc_value();
}
