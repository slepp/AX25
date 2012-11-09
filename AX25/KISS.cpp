#include "KISS.h"

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Packet.h"

size_t KISS::write(uint8_t c) {
  if(c == KISS_FEND || c == KISS_FESC) {
    SoftwareSerial::write((uint8_t)KISS_FESC);
    if(c == KISS_FEND)
      SoftwareSerial::write((uint8_t)KISS_TFEND);
      else
      SoftwareSerial::write((uint8_t)KISS_TFESC);
    return 1;
  }
  return SoftwareSerial::write(c);
}

size_t KISS::writePacket(Packet *p) {
  fend();
  unsigned int i;
  write((uint8_t)0x0);
  for(i = 0; i < p->len; ++i) {
	unsigned char c = p->getByte();
    if(c == HDLC_ESCAPE)
      write((uint8_t)p->getByte());
    else
      write((uint8_t)c);
  }
  fend();
  write('\n');
  return i;
}
