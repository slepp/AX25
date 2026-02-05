#ifndef MODEM_H
#define MODEM_H

#include "AFSKEncode.h"
#include "AFSKDecode.h"
#include "Packet.h"

extern volatile unsigned long lastTx;
extern volatile unsigned long lastTxEnd;
extern volatile unsigned long lastRx;

class Modem {
public:
  void start();
  void timer();
  inline bool read() {
    return decoder.read();
  }
  inline bool txReady() volatile {
    return encoder.isDone() && encoder.hasPackets();
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
    if(!ret)
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
  // Note: lastTx/lastRx are extern globals, not members - they are
  // shared between the ISR (encoder/decoder) and the main loop.
  AFSKEncode encoder;
  AFSKDecode decoder;
};

#endif
