#ifndef PACKET_BUFFER_H
#define PACKET_BUFFER_H

#define PACKET_STATIC 0

#include <Arduino.h>
#include "SRAM.h"

// This is with all the digis, two addresses, framing and full payload
// Two more bytes are added for HDLC_ESCAPEs
#define PACKET_MAX_LEN 512

// HDLC framing bits
#define HDLC_FRAME    0x7E
#define HDLC_RESET    0x7F
#define HDLC_PREAMBLE 0x00
#define HDLC_ESCAPE   0x1B
#define HDLC_TAIL     0x1C

class Packet:public Print {
  public:
    Packet():Print() {};
    virtual size_t write(uint8_t);
	// Stock virtual method does what we want here.
	//virtual size_t write(const char *);
	virtual size_t write(const uint8_t *, size_t);
    using Print::write;
    unsigned char ready : 1;
    unsigned char type : 2;
    unsigned char freeData : 1;
    unsigned short len;
    unsigned short maxLen;
  //void init(uint8_t *buf, unsigned int dlen, bool freeData); 
  void init(unsigned short dlen);
  inline void free() {
	  if(freeData)
		smm.free(dataPtr);
  }
  inline const unsigned char getByte(void) {
	  return SRAM[readPos++];
  }
  inline const unsigned char getByte(uint16_t p) {
	  return SRAM[dataPtr+p];
  }
  inline void start() {
    fcs = 0xffff;
    SRAM[dataPos++] = HDLC_ESCAPE;
    SRAM[dataPos++] = HDLC_FRAME;
	len = 2;
  }

  inline bool append(char c) {
    if(len < maxLen) {
	  ++len;
      SRAM[dataPos++] = c;
	  return true;
	}
	return false;	  
  }

  #define UPDATE_FCS(d) e=fcs^(d); f=e^(e<<4); fcs=(fcs>>8)^(f<<8)^(f<<3)^(f>>4)
  //#define UPDATE_FCS(d) s=(d)^(fcs>>8); t=s^(s>>4); fcs=(fcs<<8)^t^(t<<5)^(t<<12)
  inline bool appendFCS(unsigned char c) {
    register unsigned char e, f;
    if(len < maxLen - 4) { // Leave room for FCS/HDLC
      append(c);
      UPDATE_FCS(c);
	  return true;
    }
	return false;
  }

  inline void finish() {
    append(~(fcs & 0xff));
    append(~((fcs>>8) & 0xff));
    append(HDLC_ESCAPE);
    append(HDLC_FRAME);
    ready = 1;
  }
  
  inline void clear() {
    fcs = 0xffff;
    len = 0;
	readPos = dataPtr;
	dataPos = dataPtr;
  }
  
  inline bool crcOK() {
    return (fcs == 0xF0B8);
  }
  private:
    sramPtr_t dataPtr, dataPos, readPos;
    unsigned short fcs;
};

#define PACKET_BUFFER_SIZE 8

class PacketBuffer {
  public:
    // Initialize the buffers
    PacketBuffer();
    // How many packets are in the buffer?
    unsigned char count() volatile { return inBuffer; };
    // And how many of those are ready?
    unsigned char readyCount() volatile;
    // Retrieve the next packet
    Packet *getPacket() volatile;
    // Create a packet structure as needed
    // This does not place it in the queue
    static Packet *makePacket(unsigned short);
    // Conveniently free packet memory
    static void freePacket(Packet *);
    // Place a packet into the buffer
    bool putPacket(Packet *) volatile;
  private:
    volatile unsigned char inBuffer;
    Packet * volatile packets[PACKET_BUFFER_SIZE];
    volatile unsigned char nextPacketIn;
    volatile unsigned char nextPacketOut;
};

#endif

