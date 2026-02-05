#ifndef PACKET_BUFFER_H
#define PACKET_BUFFER_H

#include <Arduino.h>
#include "SRAM.h"

#define PACKET_STATIC 0

// This is with all the digis, two addresses, framing and full payload
// Two more bytes are added for HDLC_ESCAPEs
static constexpr uint16_t PACKET_MAX_LEN = 512;

// HDLC framing bytes
static constexpr uint8_t HDLC_FRAME    = 0x7E;
static constexpr uint8_t HDLC_RESET    = 0x7F;
static constexpr uint8_t HDLC_PREAMBLE = 0x00;
static constexpr uint8_t HDLC_ESCAPE   = 0x1B;
static constexpr uint8_t HDLC_TAIL     = 0x1C;

class Packet : public Print {
  public:
    Packet() : Print() {};
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    using Print::write;
    unsigned char ready : 1;
    unsigned char type : 2;
    unsigned char freeData : 1;
    unsigned short len;
    unsigned short maxLen;

  void init(unsigned short dlen);

  inline void free() {
    if(freeData)
      smm.free(dataPtr);
  }

  inline const unsigned char getByte(void) {
    return SRAM[readPos++];
  }

  inline const unsigned char getByte(uint16_t p) {
    return SRAM[dataPtr + p];
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

  inline bool appendFCS(unsigned char c) {
    if(len < maxLen - 4) { // Leave room for FCS + HDLC tail
      append(c);
      // CRC-CCITT update
      unsigned char e = fcs ^ c;
      unsigned char f = e ^ (e << 4);
      fcs = (fcs >> 8) ^ ((uint16_t)f << 8) ^ ((uint16_t)f << 3) ^ (f >> 4);
      return true;
    }
    return false;
  }

  inline void finish() {
    append(~(fcs & 0xff));
    append(~((fcs >> 8) & 0xff));
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

static constexpr uint8_t PACKET_BUFFER_SIZE = 8;

class PacketBuffer {
  public:
    PacketBuffer();
    unsigned char count() volatile { return inBuffer; };
    unsigned char readyCount() volatile;
    Packet *getPacket() volatile;
    static Packet *makePacket(unsigned short);
    static void freePacket(Packet *);
    bool putPacket(Packet *) volatile;
  private:
    volatile unsigned char inBuffer;
    Packet * volatile packets[PACKET_BUFFER_SIZE];
    volatile unsigned char nextPacketIn;
    volatile unsigned char nextPacketOut;
};

#endif
