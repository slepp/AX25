#ifndef MODEM_H
#define MODEM_H

#include "AFSKEncode.h"
#include "AFSKDecode.h"
#include "Packet.h"

// This is the main "modem" and handles
// all the basic routines needed for IO

extern volatile unsigned long lastTx;
extern volatile unsigned long lastTxEnd;
extern volatile unsigned long lastRx;

class Modem {
public:
  void start(); // Start the timers
  void timer(); // Process a timer event
  inline bool read() {
    return decoder.read();
  }
  inline bool txReady() volatile {
    if(encoder.isDone() && encoder.hasPackets())
      return true;
    return false;
  }
  inline bool isDone() volatile { return encoder.isDone(); }
  inline bool txStart() {
    if(decoder.isReceiving()) {
      encoder.setRandomWait();
      return false;
    } 
    return encoder.start();
  }
  inline bool putTXPacket(Packet *packet) {
    bool ret = encoder.putPacket(packet);
    if(!ret) // No room?
      PacketBuffer::freePacket(packet);
    return ret;
  }
  inline Packet *getRXPacket() {
    return decoder.getPacket();
  }
  inline uint8_t rxPacketCount() volatile {
    return decoder.packetCount();
  }
private:
  unsigned long lastTx;
  unsigned long lastRx;
  AFSKEncode encoder;
  AFSKDecode decoder;
};

#endif
