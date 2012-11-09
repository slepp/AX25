#include "KISS.h"

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Packet.h"

// Write a byte out to the KISS interface properly encoded
size_t KISS::write(uint8_t c) {
	switch(c) {
		case KISS_FEND:
			fesc(); return SoftwareSerial::write((uint8_t)KISS_TFEND);
		case KISS_FESC:
			fesc(); return SoftwareSerial::write((uint8_t)KISS_TFESC);
		default:
			return SoftwareSerial::write(c);
	}
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
  return i;
}
