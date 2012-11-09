/*
writestream: Setup the SRAM in sequential write mode starting from the passed address.
Bytes can then be written one byte at a time using RWdata(byte data).
Each byte is stored in the next location and it wraps around 32767.

readstream:  Setup the SRAM in sequential read mode starting from the passed address.
Bytes can then be read one byte at a time using  byte RWdata(0).The passed data is irrelavent.
Each byte is read from the next location and it wraps around 32767.

RWdata:      Write or read a byte at any time from the SRAM.
If the writesteam is open the passed byte will be written to the current address.
If the readstream is open the byte from the current address will be returned.

closeRWstream: Use to close the  open read or write stream.
Dont need when changing between read/write.
Close before using SPI somewhere else.

digital pin 13    SCK
digital pin 12    MISO
digital pin 11    MOSI
digital pin 10    SS
*/
#ifndef SRAM_h
#define SRAM_h

#define setupSPI SPCR = 0x50 | _BV(CPHA) /*| _BV(SPIE)*/; SPSR &= ~_BV(SPI2X) //Master mode, MSB first, SCK phase low, SCK idle low, clock/4
#define setupDDRB DDRB |= 0x2c  //set  SCK(13) MOSI(11) and SS as output
#define selectSS PORTB &= ~0x04  //set the SS to 0 to select
#define deselectSS PORTB |= 0x04  //set the SS to 1 to deselect

#include "Arduino.h"

typedef unsigned short sramPtr_t;

class SRAMByte;

extern volatile bool SRAMdone;

class SRAMclass
{
	public:
	SRAMclass() {};  //the constructor
	void begin();
	void startWriteStream(const sramPtr_t);
	void writeStream(const unsigned char);
	void writeByte(const sramPtr_t, const unsigned char);
	void writeBuffer(const sramPtr_t, const unsigned char *, const unsigned char);
	void writePage(const sramPtr_t, const unsigned char *);
	private:
	void endStream();
	public:
	void startReadStream(const sramPtr_t);
	unsigned char readStream();
	unsigned char readByte(const sramPtr_t);
	void readBuffer(const sramPtr_t, unsigned char *, const unsigned char);
	void readPage(const sramPtr_t, unsigned char *);
	SRAMByte operator[](const unsigned short);
	private:
	inline byte Rdata();
	inline void Wdata(const byte);
	inline void writeStatus(const unsigned char);
	inline unsigned char readStatus();
	volatile unsigned char lastMode;
	volatile unsigned char lastState;
	volatile sramPtr_t lastPos;
};//end of class SRAMclass

extern SRAMclass SRAM;

class SRAMByte {
	public:
	SRAMByte(const sramPtr_t p, const unsigned char c):position(p),changed(0),value(c) {};
	~SRAMByte() { if(changed) { SRAM.writeByte(position, value); } };
	inline operator unsigned char() const { return value; };
	inline operator char() const { return value; };
	inline operator int() const { return value; };
	SRAMByte(const SRAMByte &sb) {
		if(position != sb.position) {
			SRAM.writeByte(position, value);
		}
		position = sb.position;
		value = sb.value;
		changed = sb.changed;
	}
	inline SRAMByte &operator=(const SRAMByte &sb) {
		value = sb.value;
		SRAM.writeByte(position, value);
		changed = 1;
		return *this;
	}
	inline SRAMByte &operator=(const unsigned char c) {
		value = c;
		SRAM.writeByte(position, value);
		changed = 1;
		return *this;
	}
	private:
	sramPtr_t position;
	bool changed;
	unsigned char value;
};

#define heapStart 64
#define heapEnd 32768

class SRAMMemory {
	public:
	SRAMMemory() {
		freeListHead = 0;
		lastHeap = 0;
	};
	sramPtr_t allocate(sramPtr_t size);
	void free(sramPtr_t idx);
	private:
	sramPtr_t freeListHead;
	sramPtr_t lastHeap;
};

extern SRAMMemory smm;

#endif
