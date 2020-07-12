#ifndef UART_H
#define UART_H

#include <util/setbaud.h>

void uart_init(void);
void uart_write(char c);
void uart_write_hex(char b);

#endif
