#ifndef RADIO_H
#define RADIO_H

class Radio {
  public:
    Radio(unsigned char); // Initiate with initial PTT pin
    Radio();
    void ptt(bool); // Enable/disable the PTT
    bool ptt(); // Determine PTT state
    void setPTTPin(unsigned char); // Set the output pin for PTT
  private:
    bool pttOn;
    unsigned char pttPin;
};

#endif
