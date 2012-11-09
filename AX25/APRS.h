#ifndef APRS_H
#define APRS_h

class Base91 {
  public:
    static const char *encode(unsigned long);
    static const char *encode2(unsigned long);
};

extern const unsigned short speedTable[85];

#endif
