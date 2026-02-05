#define F_CPU 16000000
#define ARDUINO 100
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <digitalWriteFast.h>

#include "config.h"
#include "Modem.h"
#include "SimpleFIFO.h"
#include "Packet.h"
#include "Radio.h"
#include "KISS.h"
#include "AFSKDecode.h"
#include "GPS.h"
#include "APRS.h"
#include "SRAM.h"
#include "SerialCommand.h"

SerialCommand sCmd;

KISS kiss = KISS(PIN_KISS_RX, PIN_KISS_TX);

Modem modem;

Radio radio(PIN_PTT);

GPS gps;

unsigned char unprotoDigis[][7] = {
  {"WIDE1 "},
  {0x0}
};

static unsigned char lastEEPROMRevision = 0;

#include <avr/eeprom.h>
#define eeprom_read_to(dst_p, eeprom_field, dst_size) eeprom_read_block(dst_p, (void *)offsetof(__eeprom_data, eeprom_field), MIN(dst_size, sizeof((__eeprom_data*)0)->eeprom_field)); eeprom_busy_wait();
#define eeprom_read(dst, eeprom_field) eeprom_read_to(&dst, eeprom_field, sizeof(dst))
#define eeprom_write_from(src_p, eeprom_field, src_size) eeprom_write_block(src_p, (void *)offsetof(__eeprom_data, eeprom_field), MIN(src_size, sizeof((__eeprom_data*)0)->eeprom_field)); eeprom_busy_wait();
#define eeprom_write(src, eeprom_field) { decltype(src) x = src; eeprom_write_from(&x, eeprom_field, sizeof(x)); }
#define MIN(x,y) ((y)<(x)?(y):(x))

struct __eeprom_data {
  byte ver;
  byte revision;
  char srcCallsign[AX25_CALLSIGN_LEN];
  unsigned char srcSSID;
  char dstCallsign[AX25_CALLSIGN_LEN];
  unsigned char dstSSID;
  unsigned int txInterval;
  long serialBaud;
  long kissBaud;
};

// EEPROM defaults
const uint8_t def_ver PROGMEM = EEPROM_VERSION;
const uint8_t def_revision PROGMEM = 0;
const char  def_srcCallsign[AX25_CALLSIGN_LEN] PROGMEM = {'V','E','6','S','L','P'};
const uint8_t def_srcSSID PROGMEM = 0;
const char  def_dstCallsign[AX25_CALLSIGN_LEN] PROGMEM = {'A','P','Z','1','0','0'};
const uint8_t def_dstSSID PROGMEM = 0;
const uint16_t def_txInterval PROGMEM = (uint16_t)MIN_TX_INTERVAL;
const uint32_t def_serialBaud PROGMEM = (uint32_t)DEFAULT_SERIAL_BAUD;
const uint32_t def_kissBaud PROGMEM = (uint32_t)DEFAULT_KISS_BAUD;

// Current device configuration
static struct __eeprom_data configData;

// Load config from EEPROM, writing defaults if version doesn't match
void loadConfigData() {
  eeprom_read(configData.ver, ver);
  if(configData.ver == EEPROM_VERSION) {
    eeprom_read(configData.revision, revision);
    eeprom_read_to(configData.srcCallsign, srcCallsign, AX25_CALLSIGN_LEN);
    eeprom_read(configData.srcSSID, srcSSID);
    eeprom_read_to(configData.dstCallsign, dstCallsign, AX25_CALLSIGN_LEN);
    eeprom_read(configData.dstSSID, dstSSID);
    eeprom_read(configData.txInterval, txInterval);
    eeprom_read(configData.serialBaud, serialBaud);
    eeprom_read(configData.kissBaud, kissBaud);
    if(configData.txInterval < MIN_TX_INTERVAL) {
      configData.txInterval = MIN_TX_INTERVAL;
      eeprom_write(configData.txInterval, txInterval);
    }
    lastEEPROMRevision = configData.revision;
  } else {
    configData.ver = pgm_read_byte_near(&def_ver);
    configData.revision = pgm_read_byte_near(&def_revision);
    configData.srcSSID = pgm_read_byte_near(&def_srcSSID);
    configData.dstSSID = pgm_read_byte_near(&def_dstSSID);
    configData.txInterval = pgm_read_word_near(&def_txInterval);
    configData.serialBaud = pgm_read_dword_near(&def_serialBaud);
    configData.kissBaud = pgm_read_dword_near(&def_kissBaud);
    for(byte i = 0; i < AX25_CALLSIGN_LEN; ++i) {
      configData.srcCallsign[i] = pgm_read_byte_near(def_srcCallsign + i);
      configData.dstCallsign[i] = pgm_read_byte_near(def_dstCallsign + i);
    }
    eeprom_write(configData.ver, ver);
    eeprom_write(configData.revision, revision);
    eeprom_write_from(configData.srcCallsign, srcCallsign, AX25_CALLSIGN_LEN);
    eeprom_write(configData.srcSSID, srcSSID);
    eeprom_write_from(configData.dstCallsign, dstCallsign, AX25_CALLSIGN_LEN);
    eeprom_write(configData.dstSSID, dstSSID);
    eeprom_write(configData.txInterval, txInterval);
    eeprom_write(configData.serialBaud, serialBaud);
    eeprom_write(configData.kissBaud, kissBaud);
  }
}

// Output a string from program memory to the serial port
void cmdSerialOut(PGM_P p) {
  uint8_t c;
  while((c = pgm_read_byte(p++)) != '\0') {
    Serial.write((uint8_t)c);
  }
}

// Helper to pad/copy a callsign into a 6-byte field
static void setCallsign(char *dst, const char *src) {
  unsigned char slen = strlen(src);
  if(slen >= AX25_CALLSIGN_LEN) {
    memcpy(dst, src, AX25_CALLSIGN_LEN);
  } else {
    memcpy(dst, src, slen);
    for(unsigned char i = slen; i < AX25_CALLSIGN_LEN; ++i) {
      dst[i] = ' ';
    }
  }
}

// Set this device's own callsign
void cmdSetMyCall() {
  char *arg = sCmd.next();
  if(arg == nullptr) {
    cmdSerialOut(PSTR("MYCALL <callsign>"));
  } else {
    setCallsign(configData.srcCallsign, arg);
    eeprom_write(++configData.revision, revision);
    eeprom_write_from(configData.srcCallsign, srcCallsign, AX25_CALLSIGN_LEN);
    cmdSerialOut(PSTR("Set MYCALL to "));
    Serial.println(arg);
  }
}

// Set the default beacon interval
void cmdSetInterval() {
  char *arg = sCmd.next();
  if(arg == nullptr) {
    cmdSerialOut(PSTR("INTVL <milliseconds>"));
    return;
  }
  int interval = atoi(arg);
  if(interval < (int)MIN_TX_INTERVAL)
    interval = MIN_TX_INTERVAL;
  eeprom_write(++configData.revision, revision);
  configData.txInterval = interval;
  eeprom_write(interval, txInterval);
  cmdSerialOut(PSTR("Transmit beacon interval set to "));
  Serial.println(interval);
}

// Set the destination callsign
void cmdSetDstCall() {
  char *arg = sCmd.next();
  if(arg == nullptr) {
    cmdSerialOut(PSTR("DSTCALL <callsign>"));
  } else {
    setCallsign(configData.dstCallsign, arg);
    eeprom_write(++configData.revision, revision);
    eeprom_write_from(configData.dstCallsign, dstCallsign, AX25_CALLSIGN_LEN);
    cmdSerialOut(PSTR("Set DSTCALL to "));
    Serial.println(arg);
  }
}

// Prototype for packet header creation
static Packet *ax25MakePacketHeader(const char dst[6], unsigned char dstSSID,
                                    const char src[6], unsigned char srcSSID,
                                    unsigned int len,
                                    const unsigned char digis[][7]);

// Serial TX packet being assembled
Packet *serTxPacket = nullptr;

// Start collecting bytes for serTxPacket
void cmdTx() {
  cmdSerialOut(PSTR("Enter your packet:"));
  Serial.println();
  serTxPacket = ax25MakePacketHeader(
    configData.dstCallsign, configData.dstSSID,
    configData.srcCallsign, configData.srcSSID,
    512, nullptr);
  Serial.flush();
}

// Unknown command handler
void cmdUnknown(const char *cmd) {
  cmdSerialOut(PSTR("Unknown command '"));
  Serial.print(cmd);
  Serial.println('\'');
}

void setup() {
  __malloc_margin = 64;

  SRAM.begin();

  loadConfigData();
  Serial.begin(configData.serialBaud);
  kiss.begin(configData.kissBaud);
  cmdSerialOut(PSTR("Boot!"));
  Serial.println();

  sCmd.addCommand("MYCALL", cmdSetMyCall);
  sCmd.addCommand("INTVL", cmdSetInterval);
  sCmd.addCommand("DSTCALL", cmdSetDstCall);
  sCmd.addCommand("TX", cmdTx);
  sCmd.setDefaultHandler(cmdUnknown);

  // AFSK audio output (OC2B PWM)
  pinModeFast(PIN_AFSK_OUT, OUTPUT);

  // Radio PTT
  pinModeFast(PIN_PTT, OUTPUT);
  digitalWriteFast(PIN_PTT, LOW);

  // KISS port
  pinModeFast(PIN_KISS_RX, INPUT);
  pinModeFast(PIN_KISS_TX, OUTPUT);

  // Indicator LEDs
  pinModeFast(PIN_LED_RX, OUTPUT);
  pinModeFast(PIN_LED_MODEM, OUTPUT);
  digitalWriteFast(PIN_LED_RX, LOW);
  digitalWriteFast(PIN_LED_MODEM, LOW);

  // Source timer2 from clkIO (datasheet p.164)
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));

  // Non-inverting PWM on pin OC2B (pin 3)
  TCCR2A = (TCCR2A | _BV(COM2B1)) & ~(_BV(COM2B0) | _BV(COM2A1) | _BV(COM2A0));

  // Mode 1: Phase Correct PWM
  TCCR2A |= _BV(WGM20);
  TCCR2A &= ~_BV(WGM21);
  TCCR2B &= ~_BV(WGM22);

  // No prescaler (p.162)
  TCCR2B = (TCCR2B & ~(_BV(CS22) | _BV(CS21))) | _BV(CS20);

  unprotoDigis[0][6] = 1;

  gps.setStream(&kiss);
  modem.start();
}

static Packet *ax25MakePacketHeader(const char dst[6], unsigned char dstSSID,
                                    const char src[6], unsigned char srcSSID,
                                    unsigned int len,
                                    const unsigned char digis[][7]) {
  Packet *packet = PacketBuffer::makePacket(AX25_PACKET_OVERHEAD + len);

  if(packet == nullptr)
    return nullptr;

  packet->start();

  // Destination address
  for(unsigned char i = 0; i < AX25_CALLSIGN_LEN; ++i) {
    packet->appendFCS(dst[i] << 1);
  }
  packet->appendFCS(0b11100000 | ((dstSSID & 0xf) << 1));

  // Source address
  for(unsigned char i = 0; i < AX25_CALLSIGN_LEN; ++i) {
    packet->appendFCS(src[i] << 1);
  }
  if(digis == nullptr) {
    packet->appendFCS(0b01100001 | ((srcSSID & 0xf) << 1)); // Last address
  } else {
    packet->appendFCS(0b01100000 | ((srcSSID & 0xf) << 1)); // More addresses follow
    unsigned char d = 0;
    while(digis[d][0]) {
      for(unsigned char i = 0; i < AX25_CALLSIGN_LEN; ++i) {
        packet->appendFCS(digis[d][i] << 1);
      }
      unsigned char ssidByte = 0b01100000 | ((digis[d][6] & 0xf) << 1);
      if(!digis[d + 1][0]) // No more digis
        packet->appendFCS(ssidByte | 1); // Final flag
      else
        packet->appendFCS(ssidByte);
      ++d;
    }
  }
  // UI frame: control=0x03, PID=0xF0
  packet->appendFCS(0x03);
  packet->appendFCS(0xf0);

  return packet;
}

Packet *ax25MakePacket(const char dst[6], unsigned char dstSSID,
                       const char src[6], unsigned char srcSSID,
                       const char *data) {
  const unsigned char strLen = strlen(data);

  Packet *packet = ax25MakePacketHeader(dst, dstSSID, src, srcSSID, strLen, nullptr);

  if(packet == nullptr)
    return nullptr;

  for(unsigned char i = 0; i < strLen; ++i) {
    packet->appendFCS(data[i]);
  }

  packet->finish();
  return packet;
}

void printPacket(Stream *s, Packet *p) {
  cmdSerialOut(PSTR("RX: "));
  s->println(p->len);
  for(unsigned short i = 0; i < p->len; ++i)
    s->write((uint8_t)p->getByte());
  Serial.println();
}

ISR(ADC_vect) {
  modem.timer();
}

void loop() {
  if(modem.txReady()) {
    if(modem.txStart())
      radio.ptt(true);
  } else if(modem.isDone()) {
    radio.ptt(false);
    if(millis() - lastTx >= configData.txInterval) {
      eeprom_read(configData.revision, revision);
      if(configData.revision != lastEEPROMRevision) {
        loadConfigData();
      }
      Packet *packet = ax25MakePacketHeader(
          configData.dstCallsign, configData.dstSSID,
          configData.srcCallsign, configData.srcSSID,
          64, nullptr);
      if(packet != nullptr) {
        packet->print("Time: ");
        packet->print(millis());
        packet->finish();
        if(!modem.putTXPacket(packet))
          PacketBuffer::freePacket(packet);
      }
    }
  }

  if(!serTxPacket) {
    sCmd.readSerial();
  } else {
    uint8_t c;
    while(Serial.available()) {
      c = Serial.read();
      if(c != '\r' && c != '\n') {
        serTxPacket->appendFCS(c);
      } else {
        Serial.flush();
        serTxPacket->finish();
        modem.putTXPacket(serTxPacket);
        cmdSerialOut(PSTR("Queued."));
        serTxPacket = nullptr;
        break;
      }
    }
  }

  // Handle received AFSK data
  if(modem.read() || modem.rxPacketCount()) {
    while(modem.rxPacketCount()) {
      Packet *packet = modem.getRXPacket();
      if(packet) {
        printPacket(&Serial, packet);
        kiss.writePacket(packet);
        PacketBuffer::freePacket(packet);
      }
    }
  }

  // Parse GPS data and build APRS position packet
  if(gps.process()) {
    long lat, lon, alt;
    unsigned long fix_age, course, speed, hdop;
    unsigned short sats;
    gps.get_position(&lat, &lon, &fix_age);
    if(fix_age < 5000) {
      alt = gps.altitude() * 0.0328084; // cm to feet
      course = gps.course() / 100;
      speed = gps.speed();
      sats = gps.satellites();
      hdop = gps.hdop() / 100;
      Packet *p = ax25MakePacketHeader(
        configData.dstCallsign, configData.dstSSID,
        configData.srcCallsign, configData.srcSSID,
        128, unprotoDigis);
      if(p == nullptr)
        return;
      p->write('!');
      p->write('/');
      p->write((const uint8_t *)Base91::encode(lat), (size_t)4);
      p->write((const uint8_t *)Base91::encode(lon), (size_t)4);
      p->write('>');
      if(speed > 0) {
        unsigned char s;
        p->write((uint8_t)((course / 4) + 33)); // Compressed course
        for(s = 0; s < 85; ++s) {
          unsigned short speedRef = pgm_read_word_near(speedTable + s);
          if(speedRef >= speed) // Bug fix: was comparing speedRef >= s (loop index)
            break;
        }
        p->write(s + 33);
        p->write('G'); // Current, other sentence, other tracker
      } else {
        p->print(" sT");
      }
      if(alt) {
        p->print("/A=");
        p->print((unsigned short)alt);
      }
      p->finish();
      modem.putTXPacket(p);
    }
  }
}
