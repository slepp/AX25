#include <Arduino.h>
#include "Modem.h"

volatile unsigned long lastTx = 0;
volatile unsigned long lastTxEnd = 0;
volatile unsigned long lastRx = 0;

void Modem::start() {
  decoder.start();
}

void Modem::timer() {
  TIFR1 = _BV(ICF1);
  
  if(encoder.isSending())
    encoder.process(); // We have output to do

  // We setup for left shifted data, so we just need the high byte
  decoder.process(ADCH - 128);
}
