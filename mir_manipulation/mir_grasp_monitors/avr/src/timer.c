#include "misc.h"

void timer_init(void) {
    // set timer prescaler to 256
    SETBIT1(TCCR0B, CS02);
    SETBIT0(TCCR0B, CS01);
    SETBIT1(TCCR0B, CS00);
    // enable overflow interrupt
    SETBIT1(TIMSK0, TOIE0);
}

