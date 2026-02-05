#include "AFSKEncode.h"
#include "Modem.h"
#include "config.h"

#define ACCUMULATOR_BITS 24

static const uint8_t PROGMEM sinetable[256] = {
  128,131,134,137,140,143,146,149,152,156,159,162,165,168,171,174,
  176,179,182,185,188,191,193,196,199,201,204,206,209,211,213,216,
  218,220,222,224,226,228,230,232,234,236,237,239,240,242,243,245,
  246,247,248,249,250,251,252,252,253,254,254,255,255,255,255,255,
  255,255,255,255,255,255,254,254,253,252,252,251,250,249,248,247,
  246,245,243,242,240,239,237,236,234,232,230,228,226,224,222,220,
  218,216,213,211,209,206,204,201,199,196,193,191,188,185,182,179,
  176,174,171,168,165,162,159,156,152,149,146,143,140,137,134,131,
  128,124,121,118,115,112,109,106,103,99, 96, 93, 90, 87, 84, 81,
  79, 76, 73, 70, 67, 64, 62, 59, 56, 54, 51, 49, 46, 44, 42, 39,
  37, 35, 33, 31, 29, 27, 25, 23, 21, 19, 18, 16, 15, 13, 12, 10,
  9,  8,  7,  6,  5,  4,  3,  3,  2,  1,  1,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  1,  1,  2,  3,  3,  4,  5,  6,  7,  8,
  9,  10, 12, 13, 15, 16, 18, 19, 21, 23, 25, 27, 29, 31, 33, 35,
  37, 39, 42, 44, 46, 49, 51, 54, 56, 59, 62, 64, 67, 70, 73, 76,
  79, 81, 84, 87, 90, 93, 96, 99, 103,106,109,112,115,118,121,124
};

// Phase accumulator steps: pow(2,32) * freq / refclk
static const unsigned long toneStep[2] = {
  (unsigned long)(4294967296.0 * AFSK_SPACE_FREQ / AFSK_REFCLK),
  (unsigned long)(4294967296.0 * AFSK_MARK_FREQ / AFSK_REFCLK)
};

// Set to an arbitrary frequency
void AFSKEncode::setFreq(unsigned long freq, byte vol) {
  rStep = (unsigned long)(4294967296.0 * freq / AFSK_REFCLK);
}

#define T_BIT ((unsigned int)(AFSK_REFCLK / AFSK_BAUD))

void AFSKEncode::process() {
  if(bitClock == 0) {
    if(bitPosition == 0) {
      if(preamble) {
        currentByte = HDLC_PREAMBLE;
        --preamble;
      } else {
        if(!packet) {
          if(maxTx-- == 0) {
            stop();
            lastTxEnd = millis();
            return;
          }
          packet = pBuf.getPacket();
          if(!packet) {
            stop();
            lastTxEnd = millis();
            return;
          }
          lastTx = millis();
          currentBytePos = 0;
        }

        if(currentBytePos++ == packet->len) {
          pBuf.freePacket(packet);
          packet = pBuf.getPacket();
          currentBytePos = 0;
          currentByte = HDLC_FRAME;
          hdlc = true;
        } else {
          currentByte = packet->getByte();
          if(currentByte == HDLC_ESCAPE) {
            currentByte = packet->getByte();
            hdlc = true;
          } else {
            hdlc = false;
          }
        }
      }
    }

    currentBit = currentByte & 0x1;

    if(lastZero == 5) {
      currentBit = 0; // Bit stuffing: force a zero
    } else {
      currentByte >>= 1;
      ++bitPosition;
    }

    // Count consecutive 1s for NRZI bit stuffing
    if(!currentBit || hdlc)
      lastZero = 0;
    else
      ++lastZero;

    // NRZI: toggle tone on 0, no change on 1
    if(!currentBit)
      currentTone = !currentTone;
  }

  if(++bitClock == T_BIT)
    bitClock = 0;

  accumulator += toneStep[currentTone];
  uint8_t phAng = (accumulator >> ACCUMULATOR_BITS);
  OCR2B = pgm_read_byte_near(sinetable + phAng);
}

bool AFSKEncode::start() {
  if(!done || sending)
    return false;

  if(randomWait > millis())
    return false;

  accumulator = 0;
  currentBit = 0;
  lastZero = 0;
  bitPosition = 0;
  bitClock = 0;
  preamble = TX_PREAMBLE_FLAGS;
  done = false;
  hdlc = true;
  packet = nullptr;
  currentBytePos = 0;
  maxTx = TX_MAX_PACKETS;
  sending = true;
  return true;
}

void AFSKEncode::stop() {
  randomWait = 0;
  sending = false;
  done = true;
}
