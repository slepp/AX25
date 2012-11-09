#include "GPS.h"

bool GPS::process() {
  bool retVal = false;
  while(stream->available()) {
    unsigned char c = stream->read();
    if(encode(c)) {
      // State change
      retVal = true;
    }
  }
  return retVal;
}

