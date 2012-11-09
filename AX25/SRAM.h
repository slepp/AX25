/*
digital pin 13    SCK
digital pin 12    MISO
digital pin 11    MOSI
digital pin 10    SS
*/
#ifndef SRAM_h
#define SRAM_h

// The size of the chip's SRAM
// 65536 for 23lc512, 32768 for 23k256, etc
#define SRAM_SIZE			65536
// Maximum addressable segment of SRAM
#define SRAM_MAX_ADDRESS	(SRAM_SIZE - 1)
// Where to start our dynamic allocation, allowing for some statics
// This should never be 0, as we use that for NULL
#define SRAM_HEAP_START		2048
// Where to stop allocating dynamic memory
#define SRAM_HEAP_END		SRAM_MAX_ADDRESS

// Allow asynchronous calls to writing SRAM?
//#define SRAM_ASYNC

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
	void startReadStream(const sramPtr_t);
	unsigned char readStream();
	unsigned char readByte(const sramPtr_t);
	void readBuffer(const sramPtr_t, unsigned char *, const unsigned char);
	void readPage(const sramPtr_t, unsigned char *);
	SRAMByte operator[](const unsigned short);
	private:
	void endStream(); // Private so we don't interfere
	inline byte Rdata();
	inline void Wdata(const byte);
	inline void writeStatus(const unsigned char);
	inline unsigned char readStatus();
	volatile unsigned char lastMode;
	volatile unsigned char lastState;
	volatile sramPtr_t lastPos;
};//end of class SRAMclass

extern SRAMclass SRAM;

// SRAM Bytes are really here just for convenience of dynamically allocated chunks of RAM
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
