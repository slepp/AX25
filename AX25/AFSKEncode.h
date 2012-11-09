#ifndef AFSKENCODE_H
#define AFSKENCODE_H

#include <Arduino.h>
#include "Packet.h"

class AFSKEncode {
public:
  AFSKEncode() {
    randomWait = 1000; // At the very begin, wait at least one second
    sending = false;
    done = true;
    packet = 0x0;
    currentBytePos = 0;
  }
  void setFreq(unsigned long, byte);
  volatile inline bool isSending() volatile { 
    return sending; 
  }
  volatile inline bool isDone() volatile { 
    return done; 
  }
  volatile inline bool hasPackets() volatile { 
    return (pBuf.count() > 0); 
  }
  inline bool putPacket(Packet *packet) {
    return pBuf.putPacket(packet);
  }
  inline void setRandomWait() {
    randomWait = 250 + (rand() % 1000) + millis();
  }
  bool start();
  void stop();
  void process();
private:
  volatile bool sending;
  byte currentByte;
  byte currentBit : 1;
  byte currentTone : 1;
  byte lastZero : 3;
  byte bitPosition : 3;
  byte preamble : 6;
  byte bitClock;
  bool hdlc;
  byte maxTx;
  Packet *packet;
  PacketBuffer pBuf;
  unsigned char currentBytePos;
  volatile unsigned long randomWait;
  volatile bool done;
  // Phase accumulator, 32 bits, we'll use ACCUMULATOR_BITS of it
  unsigned long accumulator;
  // Current radian step for the accumulator
  unsigned long rStep;
};

#endif

