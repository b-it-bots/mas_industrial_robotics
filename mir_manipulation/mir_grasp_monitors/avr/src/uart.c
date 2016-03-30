#include "uart.h"
#include "misc.h"
#include <avr/io.h>

void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    #if USE_2X
    UCSR0A |= _BV(U2X0);
    #else
    UCSR0A &= ~(_BV(U2X0));
    #endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);
}

void uart_write(char c) {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
}

static char toHex(char b) {
    b = 0x0F & b;
    if(b > 9) {
        return b + 0x37;
    } else {
        return b + 0x30;
    }
}

void uart_write_hex(char b) {
    uart_write('0');
    uart_write('x');
    uart_write(toHex(b >> 4));
    uart_write(toHex(0x0F & b));
}

