#include "Packet.h"
#include "SRAM.h"
#include <Arduino.h>
#include <util/atomic.h>

#define PPOOL_SIZE 6
/*static Packet pPool[PPOOL_SIZE];
static byte pStatus = 0x0;

Packet *findPooledPacket() {
  unsigned char i;
  for(i = 0; i < PPOOL_SIZE; ++i) {
    if((pStatus&(1<<i)) == 0x0)
      break;
  }
  if(i == PPOOL_SIZE)
    return NULL;
  pStatus |= 1<<i;
  return &(pPool[i]);
}
*/

PacketBuffer::PacketBuffer() {
  nextPacketIn = 0;
  nextPacketOut = 0;
  inBuffer = 0;
  for(unsigned char i = 0; i < PACKET_BUFFER_SIZE; ++i) {
    packets[i] = 0x0;
  }
}

unsigned char PacketBuffer::readyCount() volatile {
  unsigned char i;
  unsigned int cnt = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	  for(i = 0; i < PACKET_BUFFER_SIZE; ++i) {
		if(packets[i] && packets[i]->ready)
		  ++cnt;
	  }
  }  
  return cnt;
}

// Return NULL on empty packet buffers
Packet *PacketBuffer::getPacket() volatile {
  unsigned char i = 0;
  Packet *p = NULL;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	  if(inBuffer == 0) {
		return 0x0;
	  }
  
	  do {
		p = packets[nextPacketOut];
		if(p) {
		  packets[nextPacketOut] = 0x0;
		  --inBuffer;
		}
		nextPacketOut = ++nextPacketOut % PACKET_BUFFER_SIZE;
		++i;
	  } while(!p && i<PACKET_BUFFER_SIZE);
  
	  // Return whatever we found, if anything
  }  
  return p;
}

//void Packet::init(uint8_t *buf, unsigned int dlen, bool freeData) {
void Packet::init(unsigned short dlen) {
  //data = (unsigned char *)buf;
  ready = 0;
  freeData = 1; //freeData;
  type = PACKET_STATIC;
  len = 0; // We had a length, but don't put it here.
  maxLen = dlen; // Put it here instead
  dataPtr = smm.allocate(dlen+16);
  dataPos = dataPtr;
  readPos = dataPtr;
  fcs = 0xffff;
}

// Allocate a new packet with a data buffer as set
Packet *PacketBuffer::makePacket(unsigned short dlen) {
	Packet *p;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	  //Packet *p = findPooledPacket();
	  p = new Packet(); //(Packet *)malloc(sizeof(Packet));
	  if(p) // If allocated
		  p->init(dlen);
  }
  return p; // Passes through a null on failure.
}

// Free a packet struct, mainly convenience
void PacketBuffer::freePacket(Packet *p) {
  if(!p)
    return;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  p->free();
  /*unsigned char i;
  for(i = 0; i < PPOOL_SIZE; ++i)
    if(p == &(pPool[i]))
      break;
  if(i < PPOOL_SIZE)
    pStatus &= ~(1<<i);*/
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
	  nextPacketIn = ++nextPacketIn % PACKET_BUFFER_SIZE;
	  ++inBuffer;
	}  
  return true;
}

// Print a single byte to the data array
size_t Packet::write(uint8_t c) {
	return (appendFCS(c)?1:0);
}

size_t Packet::write(const uint8_t *ptr, size_t len) {
	size_t i;
	for(i = 0; i < len; ++i)
	  if(!appendFCS(ptr[i]))
	    break;
	return i;
}