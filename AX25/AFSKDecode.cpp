#include <Arduino.h>
#include <digitalWriteFast.h>
#include "Modem.h"
#include "AFSKDecode.h"
#include "SimpleFIFO.h"
#include "SRAM.h"

#define PHASE_BIT 8
#define PHASE_INC 1

#define PHASE_MAX (SAMPLEPERBIT * PHASE_BIT)
#define PHASE_THRES (PHASE_MAX / 2)

#define BIT_DIFFER(bitline1, bitline2) (((bitline1) ^ (bitline2)) & 0x01)
#define EDGE_FOUND(bitline)            BIT_DIFFER((bitline), (bitline) >> 1)

#define BV(s) (1<<(s))

AFSKDecode::AFSKDecode() {
  // Initialize the sampler delay line (phase shift)
  for(unsigned char i = 0; i < SAMPLEPERBIT/2; i++)
    delay_fifo.enqueue(0);

  // Allocate our first receive packet buffer
  // We basically trust this won't come back NULL..
  //currentPacket = pBuf.makePacket(PACKET_MAX_LEN);
}

bool HDLCDecode::hdlcParse(bool bit, SimpleFIFO<uint8_t,RX_FIFO_LEN> *fifo) {
  bool ret = true;

  demod_bits <<= 1;
  demod_bits |= bit ? 1 : 0;

  // Flag
  if(demod_bits == HDLC_FRAME) {
    fifo->enqueue(HDLC_FRAME);
    rxstart = true;
    currchar = 0;
    bit_idx = 0;
    digitalWriteFast(2, HIGH);
    return ret;
  }

  // Reset
  if((demod_bits & HDLC_RESET) == HDLC_RESET) {
    rxstart = false;
    digitalWriteFast(2, LOW);
    lastRx = millis();
    return ret;
  }
  if(!rxstart) {
    digitalWriteFast(2, LOW);
    return ret;
  }

  // Stuffed?
  if((demod_bits & 0x3f) == 0x3e)
    return ret;

  if(demod_bits & 0x01)
    currchar |= 0x80;

  if(++bit_idx >= 8) {
    if(currchar == HDLC_FRAME ||
      currchar == HDLC_RESET ||
      currchar == HDLC_ESCAPE) {
      fifo->enqueue(HDLC_ESCAPE);
    }
    fifo->enqueue(currchar & 0xff);
    currchar = 0;
    bit_idx = 0;
  } 
  else {
    currchar >>= 1;
  }

  return ret;
}

// Handle the A/D converter interrupt (hopefully quickly :)
void AFSKDecode::process(int8_t curr_sample) {
  // Run the same through the phase multiplier and butterworth filter
  iir_x[0] = iir_x[1];
  iir_x[1] = ((int8_t)delay_fifo.dequeue() * curr_sample) >> 2;
  iir_y[0] = iir_y[1];
  iir_y[1] = iir_x[0] + iir_x[1] + (iir_y[0] >> 1) + (iir_y[0]>>3) + (iir_y[0]>>5);
  
  // Shift the bit into place based on the output of the discriminator
  sampled_bits <<= 1;
  sampled_bits |= (iir_y[1] > 0) ? 1 : 0;
  
  // Place this ADC sample into the delay line
  delay_fifo.enqueue(curr_sample);
  
  // If we found a 0/1 transition, adjust phases to track
  if(EDGE_FOUND(sampled_bits)) {
    if(curr_phase < PHASE_THRES)
      curr_phase += PHASE_INC;
    else
      curr_phase -= PHASE_INC;
  }
  
  // Move ahead in phase
  curr_phase += PHASE_BIT;

  // If we've gone over the phase maximum, we should now have some data
  if(curr_phase >= PHASE_MAX) {
    curr_phase %= PHASE_MAX;
    found_bits <<= 1;
    
    // If we have 3 bits or more set, it's a positive bit
    register uint8_t bits = sampled_bits & 0x07;
    if(bits == 0x07 || bits == 0x06 || bits == 0x05 || bits == 0x03) {
      found_bits |= 1;
    }
    
    hdlc.hdlcParse(!EDGE_FOUND(found_bits), &rx_fifo); // Process it
  }
}

// This routine uses a pre-allocated Packet structure
// to save on the memory requirements of the stream data
bool AFSKDecode::read() {
  bool retVal = false;
  if(!currentPacket) { // We failed a prior memory allocation
    currentPacket = pBuf.makePacket(PACKET_MAX_LEN);
    if(!currentPacket) // Still nothing
      return false;
  }
  // While we have AFSK receive FIFO bytes...
  while(rx_fifo.count()) {
    // Grab the character
    char c = rx_fifo.dequeue();
    
    bool escaped = false;
    if(c == HDLC_ESCAPE) { // We received an escaped byte, mark it
      escaped = true;
      currentPacket->append(HDLC_ESCAPE); // Append without FCS
      c = rx_fifo.dequeue(); // Reset to the next character
    }
    
    // Append all the bytes
    // This will include unescaped HDLC_FRAME bytes
    //if(c == HDLC_FRAME && !escaped)
      //currentPacket->append(c); // Framing bytes don't get FCS updates
      //else
    if(c != HDLC_FRAME)
      currentPacket->appendFCS(c); // Escaped characters and all else go into FCS
    
    if(currentPacket->len > PACKET_MAX_LEN) {
      // We've now gone too far and picked up far too many bytes
      // Cancel this frame, start back at the beginning
      currentPacket->clear();
      continue;
    }
    
    // We have a frame boundary, if it isn't escaped
    // If it's escaped, it was part of the data stream
    if(c == HDLC_FRAME && !escaped) {
      if(!currentPacket->len) {
        currentPacket->clear(); // There wasn't any data, restart stream
        continue;
      } else {
        // We have some bytes in stream, check it meets minimum payload length
        // Min payload is 1 (flag) + 14 (addressing) + 2 (control/PID) + 1 (flag)
        if(currentPacket->len >= 16) {
          digitalWriteFast(6, HIGH); // Simple "parsing" strobe
          
          // We should end up here with a valid FCS due to the appendFCS
          if(currentPacket->crcOK()) { // Magic number for the CRC check passing
            // Valid frame, so, let's filter for control + PID
            // Maximum search distance is 71 bytes to end of the address fields
            // Skip the HDLC frame start
            bool filtered = false;
            for(unsigned char i = 0; i < (currentPacket->len<70?currentPacket->len:71); ++i) {
              if((currentPacket->getByte() & 0x1) == 0x1) { // Found a byte with LSB set
                // which marks the final address payload
                // next two bytes should be the control/PID
                if(currentPacket->getByte() == 0x03 && currentPacket->getByte() == 0xf0) {
                  filtered = true;
                  break; // Found it
                }
              }
            }
            
            if(!filtered) {
              // Frame wasn't one we care about, discard
              currentPacket->clear();
              digitalWriteFast(6, LOW); // Simple "parsing" strobe
              continue;
            }
            
            /*
            // Reallocate to shrink the memory use
            Packet *reallocPacket = (Packet *)realloc(currentPacket, sizeof(Packet) + currentPacket->len);
            
            // realloc() can return NULL
            // NULL means it didn't touch our memory allocation
            if(reallocPacket != NULL) {
              Serial.print("Moved from ");
              Serial.print((unsigned long)currentPacket, HEX);
              Serial.print(" to ");
              Serial.println((unsigned long)reallocPacket, HEX);
              currentPacket = reallocPacket; // It wasn't, so reassign our packet position
            }
            
            // The data pointer may have moved in the reallocation
            currentPacket->data = (unsigned char *)(currentPacket + sizeof(Packet));
            */
 
            // It's all done and formatted, ready to go
            currentPacket->ready = 1;
            if(!pBuf.putPacket(currentPacket)) // Put it in the receive FIFO
              pBuf.freePacket(currentPacket); // Out of FIFO space, so toss it
            
            // Allocate a new one of maximum length
            currentPacket = pBuf.makePacket(PACKET_MAX_LEN);
            retVal = true;
          }
          
          digitalWriteFast(6, LOW);
        }
      }
      // Restart the stream
      currentPacket->clear();
    }
  }
  return retVal; // This is true if we parsed a packet in this flow
}

void AFSKDecode::start() {
  // Do this in start to allocate our first packet
  currentPacket = pBuf.makePacket(PACKET_MAX_LEN);
  // Configure the ADC and Timer1 to trigger automatic interrupts
  TCCR1A = 0;
  TCCR1B = BV(CS11) | BV(WGM13) | BV(WGM12);
  ICR1 = ((16000000 / 8) / 9600) - 1;
  ADMUX = BV(REFS0) | BV(ADLAR) | 0; // Channel 0, shift result left (ADCH used)
  DDRC &= ~BV(0);
  PORTC &= ~BV(0);
  DIDR0 |= BV(0);
  ADCSRB = BV(ADTS2) | BV(ADTS1) | BV(ADTS0);
  ADCSRA = BV(ADEN) | BV(ADSC) | BV(ADATE) | BV(ADIE) | BV(ADPS2); // | BV(ADPS0);  
}
