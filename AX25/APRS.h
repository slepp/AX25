#ifndef APRS_H
#define APRS_H

#include <avr/pgmspace.h>

class Base91 {
  public:
    static const char *encode(unsigned long);
};

extern const unsigned short speedTable[85] PROGMEM;

#endif
