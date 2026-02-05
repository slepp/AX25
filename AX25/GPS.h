#ifndef GPS_H
#define GPS_H

#include <TinyGPS.h>

class GPS : public TinyGPS {
public:
  GPS() : TinyGPS(), stream(nullptr) {}
  bool setStream(Stream *s) {
    stream = s;
    return true;
  }
  bool process();
private:
  Stream *stream;
};

#endif
