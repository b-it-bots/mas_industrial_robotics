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

// #define UART_DEBUG
#define IR_LIMIT 0x7F

int main (void) {
    // Onboard LED Pin as output
    SETBIT1(DDRB, DDB5);
    // IR LED Pin as output (D12)
    SETBIT1(DDRB, DDB4);
    // Enable IR LED
    SETBIT1(PORTB, PORTB4);

    uart_init();
    timer_init();
    adc_init();

    // Interrupt enable
    sei();

    while(1) {
    }
}

char timer_counter = 0;
// 16MHz / 1024 / 256 / 6 ca 10 Hz
ISR (TIMER0_OVF_vect) {
    if(timer_counter++ >= 6) {
        timer_counter = 0;
        adc_start();
        while(adc_running()) {}
        char value = adc_value();
        SETBIT(PORTB, PORTB5, (0xFF & value) < IR_LIMIT);
        #if UART_DEBUG
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

