#include "misc.h"
#include "adc.h"
#include <avr/io.h>

void adc_init(void) {
    SETBIT0(DDRC, 0);
    SETBIT1(ADMUX, ADLAR);
    SETBIT0(ADMUX, REFS1);
    SETBIT1(ADMUX, REFS0);
    SETBIT1(ADCSRA, ADPS2);
    SETBIT1(ADCSRA, ADPS1);
    SETBIT0(ADCSRA, ADPS0);
    SETBIT1(ADCSRA, ADEN);
}

void adc_select_0(void) {
    SETBIT0(ADMUX, MUX3);
    SETBIT0(ADMUX, MUX2);
    SETBIT1(ADMUX, MUX1);
    SETBIT1(ADMUX, MUX0);
}

void adc_select_1(void) {
    SETBIT0(ADMUX, MUX3);
    SETBIT1(ADMUX, MUX2);
    SETBIT0(ADMUX, MUX1);
    SETBIT0(ADMUX, MUX0);
}

void adc_start(void) {
    SETBIT1(ADCSRA, ADSC);
}

char adc_running(void) {
    return GETBIT(ADCSRA, ADSC);
}

uint8_t adc_value(void) {
    return ADCH;
}

