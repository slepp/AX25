#ifndef KISS_H
#define KISS_H

#define KISS_FEND 0xC0
#define KISS_FESC 0xDB
#define KISS_TFEND 0xDC
#define KISS_TFESC 0xDD

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Packet.h"

class KISS : public SoftwareSerial {
	public:
	KISS(const unsigned char rx, const unsigned char tx):SoftwareSerial(rx,tx) {}
	virtual size_t write(uint8_t);
	inline size_t fend() {
		return SoftwareSerial::write((uint8_t)KISS_FEND);
	}
	inline size_t fesc() {
		return SoftwareSerial::write((uint8_t)KISS_FESC);
	}
	size_t writePacket(Packet *);
};

#endif

