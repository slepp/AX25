#include <Arduino.h>
#include <digitalWriteFast.h>
#include "Modem.h"
#include "AFSKDecode.h"
#include "SimpleFIFO.h"
#include "SRAM.h"
#include "config.h"

#define PHASE_BIT 8
#define PHASE_INC 1

#define PHASE_MAX (SAMPLEPERBIT * PHASE_BIT)
#define PHASE_THRES (PHASE_MAX / 2)

#define BIT_DIFFER(bitline1, bitline2) (((bitline1) ^ (bitline2)) & 0x01)
#define EDGE_FOUND(bitline)            BIT_DIFFER((bitline), (bitline) >> 1)

AFSKDecode::AFSKDecode() {
  for(unsigned char i = 0; i < SAMPLEPERBIT/2; i++)
    delay_fifo.enqueue(0);
}

bool HDLCDecode::hdlcParse(bool bit, SimpleFIFO<uint8_t,RX_FIFO_LEN> *fifo) {
  bool ret = true;

  demod_bits <<= 1;
  demod_bits |= bit ? 1 : 0;

  if(demod_bits == HDLC_FRAME) {
    fifo->enqueue(HDLC_FRAME);
    rxstart = true;
    currchar = 0;
    bit_idx = 0;
    digitalWriteFast(PIN_LED_RX, HIGH);
    return ret;
  }

  if((demod_bits & HDLC_RESET) == HDLC_RESET) {
    rxstart = false;
    digitalWriteFast(PIN_LED_RX, LOW);
    lastRx = millis();
    return ret;
  }
  if(!rxstart) {
    digitalWriteFast(PIN_LED_RX, LOW);
    return ret;
  }

  // Bit stuffing: skip after five consecutive 1s
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

void AFSKDecode::process(int8_t curr_sample) {
  // IQ demodulation with Butterworth filter
  iir_x[0] = iir_x[1];
  iir_x[1] = ((int8_t)delay_fifo.dequeue() * curr_sample) >> 2;
  iir_y[0] = iir_y[1];
  iir_y[1] = iir_x[0] + iir_x[1] + (iir_y[0] >> 1) + (iir_y[0]>>3) + (iir_y[0]>>5);

  sampled_bits <<= 1;
  sampled_bits |= (iir_y[1] > 0) ? 1 : 0;

  delay_fifo.enqueue(curr_sample);

  // Track phase from edges
  if(EDGE_FOUND(sampled_bits)) {
    if(curr_phase < PHASE_THRES)
      curr_phase += PHASE_INC;
    else
      curr_phase -= PHASE_INC;
  }

  curr_phase += PHASE_BIT;

  if(curr_phase >= PHASE_MAX) {
    curr_phase %= PHASE_MAX;
    found_bits <<= 1;

    // Majority voting on 3 samples
    uint8_t bits = sampled_bits & 0x07;
    if(bits == 0x07 || bits == 0x06 || bits == 0x05 || bits == 0x03) {
      found_bits |= 1;
    }

    hdlc.hdlcParse(!EDGE_FOUND(found_bits), &rx_fifo);
  }
}

bool AFSKDecode::read() {
  bool retVal = false;
  if(!currentPacket) {
    currentPacket = pBuf.makePacket(PACKET_MAX_LEN);
    if(!currentPacket)
      return false;
  }

  while(rx_fifo.count()) {
    char c = rx_fifo.dequeue();

    bool escaped = false;
    if(c == HDLC_ESCAPE) {
      escaped = true;
      currentPacket->append(HDLC_ESCAPE);
      c = rx_fifo.dequeue();
    }

    if(c != HDLC_FRAME)
      currentPacket->appendFCS(c);

    if(currentPacket->len > PACKET_MAX_LEN) {
      currentPacket->clear();
      continue;
    }

    if(c == HDLC_FRAME && !escaped) {
      if(!currentPacket->len) {
        currentPacket->clear();
        continue;
      } else {
        // Min payload: 14 (addressing) + 2 (control/PID)
        if(currentPacket->len >= 16) {
          digitalWriteFast(PIN_LED_MODEM, HIGH);

          if(currentPacket->crcOK()) {
            // Filter for UI frames (control=0x03, PID=0xF0)
            bool filtered = false;
            uint8_t searchLen = (currentPacket->len < 70) ? currentPacket->len : 71;
            for(unsigned char i = 0; i < searchLen; ++i) {
              if((currentPacket->getByte() & 0x1) == 0x1) {
                if(currentPacket->getByte() == 0x03 && currentPacket->getByte() == 0xf0) {
                  filtered = true;
                  break;
                }
              }
            }

            if(!filtered) {
              currentPacket->clear();
              digitalWriteFast(PIN_LED_MODEM, LOW);
              continue;
            }

            currentPacket->ready = 1;
            if(!pBuf.putPacket(currentPacket))
              pBuf.freePacket(currentPacket);

            currentPacket = pBuf.makePacket(PACKET_MAX_LEN);
            retVal = true;
          }

          digitalWriteFast(PIN_LED_MODEM, LOW);
        }
      }
      currentPacket->clear();
    }
  }
  return retVal;
}

void AFSKDecode::start() {
  currentPacket = pBuf.makePacket(PACKET_MAX_LEN);

  // Configure Timer1 to trigger ADC at sample rate
  TCCR1A = 0;
  TCCR1B = _BV(CS11) | _BV(WGM13) | _BV(WGM12);
  ICR1 = ((F_CPU / 8) / AFSK_REFCLK) - 1;

  // Configure ADC: channel 0, left-adjusted, AVcc reference
  ADMUX = _BV(REFS0) | _BV(ADLAR) | 0;
  DDRC &= ~_BV(0);
  PORTC &= ~_BV(0);
  DIDR0 |= _BV(0);
  ADCSRB = _BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0);
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2);
}
