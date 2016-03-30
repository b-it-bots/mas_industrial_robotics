#include "misc.h"

void timer_init(void) {
    SETBIT1(TCCR0B, CS02);
    SETBIT0(TCCR0B, CS01);
    SETBIT1(TCCR0B, CS00);

    SETBIT1(TIMSK0, TOIE0);
}

