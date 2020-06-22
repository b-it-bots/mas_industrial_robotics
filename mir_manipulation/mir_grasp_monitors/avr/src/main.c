#ifndef F_CPU
#define F_CPU 16000000
#endif

#define BAUD 9600

#include <avr/interrupt.h>
#include <avr/io.h>
#include "adc.c"
#include "misc.h"
#include "timer.c"
#include "uart.c"

// #define UART_DEBUG

int main(void) {
  // Onboard LED Pin as output
  SETBIT1(DDRB, DDB5);

  uart_init();
  timer_init();
  adc_init();

  // Interrupt enable
  sei();

  while (1) {
  }
}

// 16MHz / 1024 / 256  ca 60 Hz
ISR(TIMER0_OVF_vect) {
  // read both adc values and send them via uart
  uint8_t adc0 = adc_read(3);
  uint8_t adc1 = adc_read(4);
#ifdef UART_DEBUG
  uart_write_hex(adc0);
  uart_write(' ');
  uart_write_hex(adc1);
  uart_write('\r');
  uart_write('\n');
#else
  uart_write(adc0);
  uart_write(adc1);
#endif
}
