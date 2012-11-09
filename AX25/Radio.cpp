#include "Radio.h"
#include <Arduino.h>

Radio::Radio(unsigned char pin) {
  setPTTPin(pin);
  pttOn = false;
}

Radio::Radio() {
  pttPin = 0;
  pttOn = false;
}

void Radio::ptt(bool enable) {
  if(enable)
    digitalWrite(pttPin, HIGH);
  else
    digitalWrite(pttPin, LOW);
  pttOn = enable;
}

bool Radio::ptt() {
  return pttOn;
}

void Radio::setPTTPin(unsigned char pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  pttPin = pin;
}
