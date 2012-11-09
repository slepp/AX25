#ifndef AFSKDECODE_H
#define AFSKDECODE_H

#include <Arduino.h>
#include "Packet.h"
#include "SimpleFIFO.h"

#define SAMPLERATE 9600
#define BITRATE    1200

#define SAMPLEPERBIT (SAMPLERATE / BITRATE)

#define RX_FIFO_LEN 16

class HDLCDecode {
public:
  bool hdlcParse(bool, SimpleFIFO<uint8_t,RX_FIFO_LEN> *fifo);
  volatile bool rxstart;
private:
  uint8_t demod_bits;
  uint8_t bit_idx;
  uint8_t currchar;
};

class AFSKDecode {
public:
  AFSKDecode();
  void start();
  bool read();
  void process(int8_t);
  inline bool dataAvailable() {
    return (rx_fifo.count() > 0);
  }
  inline uint8_t getByte() {
    return rx_fifo.dequeue();
  }
  inline uint8_t packetCount() volatile {
    return pBuf.count();
  }
  inline Packet *getPacket() {
    return pBuf.getPacket();
  }
  inline bool isReceiving() volatile {
    return hdlc.rxstart;
  }
private:
  Packet *currentPacket;
  SimpleFIFO<int8_t,SAMPLEPERBIT/2+1> delay_fifo;
  SimpleFIFO<uint8_t,RX_FIFO_LEN> rx_fifo; // This should be drained fairly often
  int16_t iir_x[2];
  int16_t iir_y[2];
  uint8_t sampled_bits;
  int8_t curr_phase;
  uint8_t found_bits;
  PacketBuffer pBuf;
  HDLCDecode hdlc;
};
#endif
