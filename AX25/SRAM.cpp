/*
writestream: Setup the SRAM in sequential write mode starting from the passed address.
Data can then be written one byte at a time using RWdata(byte data).
Each byte is stored in the next location and it wraps around 32767.

readstream:  Setup the SRAM in sequential read mode starting from the passed address.
Data can then be read one byte at a time using  byte RWdata(0).The passed data is irrelavent.
Each byte is read from the next location and it wraps around 32767.

RWdata:      Write or read the data from the SRAM.
If the writesteam is open the passed data will be written to the current address.
If the readstream is open the data from the current address will be returned.

closeRWstream: Use to close the  open read or write stream.
Dont need when changing between read/write.
Close before using SPI somewhere else.

digital pin 13    SCK
digital pin 12    MISO
digital pin 11    MOSI
digital pin 10    SS
*/

#include "Arduino.h"
#include "SRAM.h"

#include <util/atomic.h>

// 23K256 Commands
#define CMD_READ  0x03
#define CMD_WRITE 0x02
#define CMD_RDSR  0x05
#define CMD_WRSR  0x01

// 23K256 Modes
#define MODE_BYTE 0b00000001
#define MODE_PAGE 0b10000001
#define MODE_SEQN 0b01000001

#ifdef SRAM_ASYNC
volatile bool SRAMdone = true;
#endif

// Wait for a result when reading
inline byte SRAMclass::Rdata()
{
	unsigned char rVal;
#ifndef SRAM_ASYNC
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		++lastPos;
		SPDR = 0xFF;
		while (!(SPSR & _BV(SPIF)));
		rVal = SPDR;
	}
#else // We will use ASYNC methods
	++lastPos;
	while(!SRAMdone);
	SPDR = 0xFF;
	while(!SRAMdone);
	rVal = SPDR;
	SRAMdone = false;
#endif
	return rVal;
}

// Let writes happen while we continue
inline void SRAMclass::Wdata(const byte data)
{
#ifndef SRAM_ASYNC
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		++lastPos;
		SPDR = data;
		while (!(SPSR & _BV(SPIF)));
	}
#else
	++lastPos;
	while(!SRAMdone);
	SRAMdone = false;
	SPDR = data;
#endif
}

inline void SRAMclass::writeStatus(const unsigned char b) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		if(lastMode != b) {
			deselectSS;
			selectSS;
			Wdata(CMD_WRSR);
			Wdata(b);
			deselectSS;
			lastMode = b;
			lastState = CMD_WRSR;
			lastPos = 0;
		}
	}
}

inline unsigned char SRAMclass::readStatus() {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		deselectSS;
		selectSS;
		Wdata(CMD_RDSR);
		lastState = CMD_RDSR;
		unsigned char res = Rdata();
		deselectSS;
		lastPos = 0;
		return res;
	}
}

void SRAMclass::begin()  //constructor
{
	setupDDRB;
	setupSPI;
	digitalWrite(12, HIGH); // Set MISO pull up high
#ifdef SRAM_ASYNC
	SRAMdone = true;
#endif
	writeStatus(MODE_SEQN);
	startWriteStream(0x0);
	for(unsigned long i = 0; i < SRAM_MAX_ADDRESS; ++i)
	Wdata(0x0);
	deselectSS;
	lastMode = MODE_SEQN;
	lastState = CMD_WRITE;
	lastPos = 0;
}

void SRAMclass::startWriteStream(const sramPtr_t addr) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if(lastMode == MODE_SEQN && lastState == CMD_WRITE && addr == lastPos)
		return;

		writeStatus(MODE_SEQN);
		deselectSS;

		selectSS;
		Wdata(CMD_WRITE);
		lastState = CMD_WRITE;

		Wdata(addr>>8);
		Wdata(addr&0xff);
		lastPos = addr;
	}
}

void SRAMclass::writeStream(const unsigned char b) {
	Wdata(b);
}

void SRAMclass::endStream() {
	lastPos = 0;
	lastMode = MODE_PAGE; // Dummy?
	lastState = CMD_READ; // Another dummy?
	deselectSS;
}

void SRAMclass::startReadStream(const sramPtr_t addr) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		if(lastMode == MODE_SEQN && lastState == CMD_READ && addr == lastPos)
		return;

		writeStatus(MODE_SEQN);
		deselectSS;

		selectSS;
		Wdata(CMD_READ);
		lastState = CMD_READ;

		Wdata(addr>>8);
		Wdata(addr&0xff);
		lastPos = addr;
	}
}

unsigned char SRAMclass::readStream() {
	return Rdata();
}

void SRAMclass::writeByte(const sramPtr_t addr, const unsigned char b) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		startWriteStream(addr);
		Wdata(b); // Send clock pulses
	}
}

void SRAMclass::writeBuffer(const sramPtr_t addr, const unsigned char *b, const unsigned char len) {
	startWriteStream(addr);
	for(unsigned char p = 0; p < len; ++p) {
		Wdata(b[p]);
	}
}

void SRAMclass::writePage(const sramPtr_t addr, const unsigned char *b) {
	writeBuffer(addr, b, 32);
}

unsigned char SRAMclass::readByte(const sramPtr_t addr) {
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		startReadStream(addr);
		return Rdata();
	}
	return 0; // Make the compiler happy.
}

void SRAMclass::readBuffer(const sramPtr_t addr, unsigned char *b, const unsigned char len) {
	startReadStream(addr);
	for(unsigned char p = 0; p < len; ++p) {
		b[p] = Rdata();
	}
}

void SRAMclass::readPage(const sramPtr_t addr, unsigned char *b) {
	readBuffer(addr, b, 32);
}

SRAMclass SRAM;

// Read the byte at the index
SRAMByte SRAMclass::operator[](sramPtr_t idx) {
	// If we want an index that isn't next in sequence, reset the memory pointer
	return SRAMByte(idx, readByte(idx));
}

SRAMMemory smm;

#define GET_INT(addr) (unsigned short)( (((unsigned char)SRAM[(addr)]) << 8) + ((unsigned char)SRAM[(addr)+1]) )
#define PUT_INT(addr, value) SRAM[(addr)] = (unsigned char)((value)>>8); SRAM[(addr)+1] = (unsigned char)((value)&0xFF);

#define GET_NX(addr) GET_INT(addr+sizeof(sramPtr_t))
#define PUT_NX(addr,nx) PUT_INT(addr+sizeof(sramPtr_t),(nx))

sramPtr_t SRAMMemory::allocate(sramPtr_t size) {
	sramPtr_t s, fp1, fp2;

	//printf("Allocate of %d\n", size);

	if(size == 0)
	return 0;

	if(size < sizeof(sramPtr_t))
	size = sizeof(sramPtr_t);

	sramPtr_t sz;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		// Check the free list first for a perfect match
		for(s = 0, fp1 = freeListHead, fp2 = 0; fp1; fp2 = fp1, fp1 = GET_NX(fp1)) {
			sz = GET_INT(fp1);
			//printf("Block of %d at %d\n", sz, fp1);
			if(sz == size) {
				if(fp2) {
					sramPtr_t nx = GET_NX(fp1);
					PUT_NX(fp2, nx);
				} else
				freeListHead = GET_NX(fp1);
				return fp1+sizeof(sramPtr_t);
			}
			if(sz > size) {
				if(s == 0 || sz < s)
				s = sz;
			}
		}

		//printf("No perfect match, closest was %d\n", s);
		if(s) { // Found something useful
			if(s - size < sizeof(sramPtr_t)+sizeof(sramPtr_t))
			size = s;
			for(fp1 = freeListHead, fp2 = 0; fp1; fp2 = fp1, fp1 = GET_NX(fp1)) {
				sz = GET_INT(fp1);
				if(sz == s) {
					if(size == s) {
						sramPtr_t nx = GET_NX(fp1);
						if(fp2) {
							PUT_NX(fp2, nx);
						} else
							freeListHead = nx;
						return fp1+sizeof(sramPtr_t);
					}
					// Split them
					fp2 = fp1;
					s -= size;
					fp2 += s;
					PUT_INT(fp2, size);
					s -= sizeof(sramPtr_t);
					PUT_INT(fp1, s);
					return fp2+sizeof(sramPtr_t);
				}
			}
		}

		//printf("No available slots\n");
		if(lastHeap == 0) {
			lastHeap = SRAM_HEAP_START;
		}
		sramPtr_t avail = SRAM_HEAP_END - lastHeap;
		if(avail >= size && avail >= size + sizeof(sramPtr_t)) {
			fp1 = lastHeap;
			lastHeap += size + sizeof(sramPtr_t);
			PUT_INT(fp1, size);
			return (fp1+sizeof(sramPtr_t)); // Return pointer to the data segment
		}
	} // End of atomic block
	return 0;
}

void SRAMMemory::free(sramPtr_t idx) {
	sramPtr_t fp1, fp2, fpnew;

	if(idx == 0)
		return;

	// To our real position
	fpnew = idx-sizeof(sramPtr_t);

	// Update next pointer for this block
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		PUT_NX(fpnew, 0x0);

		if(freeListHead == 0) { // Only item available
			freeListHead = fpnew;
			return;
		}

		for(fp1 = freeListHead, fp2 = 0; fp1; fp2 = fp1, fp1 = GET_NX(fp1)) {
			if(fp1 < fpnew)
				continue;
			PUT_NX(fpnew, fp1);
			//printf("fpnew %d + sizeof(sramPtr_t) %d + GET_INT(fpnew) %d == %d\n", fpnew, sizeof(sramPtr_t), GET_INT(fpnew), fp1);
			if(fpnew + sizeof(sramPtr_t) + GET_INT(fpnew) == fp1) {
				// Adjacent blocks
				//printf("Block merger at fpnew %d and fp1 %d.\n", fpnew, fp1);
				sramPtr_t sz = GET_INT(fpnew);
				sz += GET_INT(fp1);
				sz += sizeof(sramPtr_t);
				PUT_INT(fpnew, sz);
				PUT_NX(fpnew, GET_NX(fp1));
			}
			if(fp2 == 0) {
				freeListHead = fpnew;
				return;
			}
			break;
		}

		PUT_NX(fp2, fpnew);
		//printf("fpnew %d + sizeof(sramPtr_t) %d + GET_INT(fpnew) %d == %d\n", fpnew, sizeof(sramPtr_t), GET_INT(fp2), fpnew);
		if(fp2 + sizeof(sramPtr_t) + GET_INT(fp2) == fpnew) {
			/* Adjacent, so merge */
			//printf("Parent adjacency merge for %d and %d\n", fp2, fpnew);
			PUT_INT(fp2, GET_INT(fp2) + GET_INT(fpnew) + sizeof(sramPtr_t));
			PUT_NX(fp2, GET_NX(fpnew));
		}
	} // End of atomic block		
}

#ifdef SRAM_ASYNC
ISR(SPI_STC_vect, ISR_NAKED) {
	// Set SRAMdone to 1, quickly
	__asm__ __volatile__ (
		"push r16"			"\n\t"
		"ldi r16, 0x01"		"\n\t"
		"sts SRAMdone, r16"	"\n\t"
		"pop r16"			"\n\t"
		"reti"				"\n\t"
	);
}
#endif