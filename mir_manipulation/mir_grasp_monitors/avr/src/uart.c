#include "uart.h"
#include <avr/io.h>
#include "misc.h"

void uart_init(void) {
  // set baud rate
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

  // enable transmitter
  SETBIT1(UCSR0B, TXEN0);

  // 8 bit mode
  SETBIT0(UCSR0B, UCSZ02);
  SETBIT1(UCSR0C, UCSZ01);
  SETBIT1(UCSR0C, UCSZ00);
}

void uart_write(char c) {
  // wait for previous transmission to be finished
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
}

static char to_hex(char b) {
  b = 0x0F & b;
  if (b > 9) {
    return b + 0x37;
  } else {
    return b + 0x30;
  }
}

void uart_write_hex(char b) {
  uart_write('0');
  uart_write('x');
  uart_write(to_hex(b >> 4));
  uart_write(to_hex(0x0F & b));
}
