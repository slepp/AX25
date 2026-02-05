#include "Packet.h"
#include "SRAM.h"
#include <Arduino.h>
#include <util/atomic.h>

PacketBuffer::PacketBuffer() {
  nextPacketIn = 0;
  nextPacketOut = 0;
  inBuffer = 0;
  for(unsigned char i = 0; i < PACKET_BUFFER_SIZE; ++i) {
    packets[i] = nullptr;
  }
}

unsigned char PacketBuffer::readyCount() volatile {
  unsigned char cnt = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for(unsigned char i = 0; i < PACKET_BUFFER_SIZE; ++i) {
      if(packets[i] && packets[i]->ready)
        ++cnt;
    }
  }
  return cnt;
}

// Return nullptr on empty packet buffers
Packet *PacketBuffer::getPacket() volatile {
  unsigned char i = 0;
  Packet *p = nullptr;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if(inBuffer == 0) {
      return nullptr;
    }

    do {
      p = packets[nextPacketOut];
      if(p) {
        packets[nextPacketOut] = nullptr;
        --inBuffer;
      }
      // Fix: avoid undefined behavior from modifying and reading
      // nextPacketOut in the same expression (was ++nextPacketOut % ...)
      nextPacketOut = (nextPacketOut + 1) % PACKET_BUFFER_SIZE;
      ++i;
    } while(!p && i < PACKET_BUFFER_SIZE);
  }
  return p;
}

void Packet::init(unsigned short dlen) {
  ready = 0;
  freeData = 1;
  type = PACKET_STATIC;
  len = 0;
  maxLen = dlen;
  dataPtr = smm.allocate(dlen + 16);
  dataPos = dataPtr;
  readPos = dataPtr;
  fcs = 0xffff;
}

// Allocate a new packet with a data buffer as set
Packet *PacketBuffer::makePacket(unsigned short dlen) {
  Packet *p;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    p = new Packet();
    if(p)
      p->init(dlen);
  }
  return p;
}

// Free a packet struct
void PacketBuffer::freePacket(Packet *p) {
  if(!p)
    return;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    p->free();
    delete p;
  }
}

// Put a packet onto the buffer array
bool PacketBuffer::putPacket(Packet *p) volatile {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if(inBuffer >= PACKET_BUFFER_SIZE) {
      return false;
    }
    packets[nextPacketIn] = p;
    // Fix: avoid undefined behavior (was ++nextPacketIn % ...)
    nextPacketIn = (nextPacketIn + 1) % PACKET_BUFFER_SIZE;
    ++inBuffer;
  }
  return true;
}

// Print a single byte to the data array
size_t Packet::write(uint8_t c) {
  return appendFCS(c) ? 1 : 0;
}

size_t Packet::write(const uint8_t *ptr, size_t len) {
  size_t i;
  for(i = 0; i < len; ++i)
    if(!appendFCS(ptr[i]))
      break;
  return i;
}
