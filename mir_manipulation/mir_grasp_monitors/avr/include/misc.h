#ifndef MISC_H
#define MISC_H

#define TOGBIT(var, num) ((var) ^= (_BV(num)))
#define GETBIT(var, num) (((var) & (_BV(num))) >> (num))
#define SETBIT0(var, num) ((var) &= ~(_BV(num)))
#define SETBIT1(var, num) ((var) |= (_BV(num)))

#define SETBIT(var, num, val) \
  {                           \
    if (val) {                \
      SETBIT1(var, num);      \
    } else {                  \
      SETBIT0(var, num);      \
    }                         \
  }

#endif
