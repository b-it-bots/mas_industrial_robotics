#ifndef F_CPU
#define F_CPU 16000000
#endif

#define BAUD 9600

#include "misc.h"
#include "uart.c"
#include "timer.c"
#include "adc.c"
#include <avr/io.h>
#include <avr/interrupt.h>

//#define UART_DEBUG
#define IR_LIMIT 0x7F

int main (void) {
    // Onboard LED Pin as output
    SETBIT1(DDRB, DDB5);

    uart_init();
    timer_init();
    adc_init();

    // Interrupt enable
    sei();

    while(1) {
    }
}

uint8_t read_lowest_value(void) {
    adc_select_0();
    adc_start();
    while(adc_running()) {}
    uint8_t adc0 = adc_value();
    adc_select_1();
    adc_start();
    while(adc_running()) {}
    uint8_t adc1 = adc_value();
    if(adc0 < adc1)
        return adc0;
    else
        return adc1;
    // return (adc0 < adc1) ? adc0 : adc1;
}

char timer_counter = 0;
// 16MHz / 1024 / 256 / 6 ca 10 Hz
ISR (TIMER0_OVF_vect) {
    if(timer_counter++ >= 6) {
        timer_counter = 0;
        uint8_t value = read_lowest_value();
        SETBIT(PORTB, PORTB5, (0xFF & value) < IR_LIMIT);
        #ifdef UART_DEBUG
        uart_write_hex(value);
        uart_write(' ');
        if((0xFF & value) > 0x7F) {
            // free
            uart_write('F');
        } else {
            // blocked
            uart_write('B');
        }
        uart_write('\r');
        uart_write('\n');
        #else
        uart_write(value);
        #endif
    }
}

