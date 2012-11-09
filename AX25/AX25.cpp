#define F_CPU 16000000
#define ARDUINO 100
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <stdarg.h>
#include <digitalWriteFast.h>

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

// RX on 4, TX on 5
KISS kiss = KISS(4,5);

// Create our AFSK modem
Modem modem;

// Enable the radio PTT on pin 13
Radio radio(7);

// GPS routines
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
#define eeprom_write(src, eeprom_field) { typeof(src) x = src; eeprom_write_from(&x, eeprom_field, sizeof(x)); }
#define MIN(x,y) (y<x?y:x)

#define EEPROM_VERSION 8
struct __eeprom_data {
  byte ver; // Version
  byte revision; // Revision of EEPROM data
  char srcCallsign[6];
  unsigned char srcSSID;
  char dstCallsign[6];
  unsigned char dstSSID;
  unsigned int txInterval;
  long serialBaud;
  long kissBaud;
};

// EEPROM defaults
const uint8_t def_ver PROGMEM = EEPROM_VERSION;
const uint8_t def_revision PROGMEM = 0;
const char  def_srcCallsign[6] PROGMEM = {'V','E','6','S','L','P'};
const uint8_t def_srcSSID PROGMEM = 0;
const char  def_dstCallsign[6] PROGMEM = {'A','P','Z','1','0','0'};
const uint8_t def_dstSSID PROGMEM = 0;
const uint16_t def_txInterval PROGMEM = (uint16_t)1000;
const uint32_t def_serialBaud PROGMEM = (uint32_t)57600;
const uint32_t def_kissBaud PROGMEM = (uint32_t)4800;

// Store the current device configuration
static struct __eeprom_data configData;

// Compare versions of EEPROM config data and compiled variant,
// and reprogram as necessary.
void loadConfigData() {
  eeprom_read(configData.ver, ver);
  if(configData.ver == EEPROM_VERSION) {
    eeprom_read(configData.revision, revision);
    eeprom_read_to(configData.srcCallsign, srcCallsign, 6);
    eeprom_read(configData.srcSSID, srcSSID);
    eeprom_read_to(configData.dstCallsign, dstCallsign, 6);
    eeprom_read(configData.dstSSID, dstSSID);
    eeprom_read(configData.txInterval, txInterval);
    eeprom_read(configData.serialBaud, serialBaud);
    eeprom_read(configData.kissBaud, kissBaud);
    if(configData.txInterval < 1000) {
      configData.txInterval = 1000;
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
    for(byte i = 0; i < 6; ++i) {
      configData.srcCallsign[i] = pgm_read_byte_near(def_srcCallsign+i);
      configData.dstCallsign[i] = pgm_read_byte_near(def_dstCallsign+i);
    }
    eeprom_write(configData.ver, ver);
    eeprom_write(configData.revision, revision);
    eeprom_write_from(configData.srcCallsign, srcCallsign, 6);
    eeprom_write(configData.srcSSID, srcSSID);
    eeprom_write_from(configData.dstCallsign, dstCallsign, 6);
    eeprom_write(configData.dstSSID, dstSSID);
    eeprom_write(configData.txInterval, txInterval);
    eeprom_write(configData.serialBaud, serialBaud);
    eeprom_write(configData.kissBaud, kissBaud);
  }
}

// Simply output a string from program memory to the serial port
void cmdSerialOut(PGM_P p) {
	register uint8_t c;
	while((c = pgm_read_byte(p++)) != '\0') {
		Serial.write((uint8_t)c);
	}
}

// Clear bit
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
// Set bit
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Set this device's own callsign
void cmdSetMyCall() {
	char *arg;
	arg = sCmd.next();
	if(arg == NULL) {
		cmdSerialOut(PSTR("MYCALL <callsign>"));
	} else {
		if(strlen(arg) >= 6) {
			memcpy(configData.srcCallsign, arg, 6);
		} else {
			memcpy(configData.srcCallsign, arg, strlen(arg));
			for(unsigned char i = strlen(arg); i < 6; ++i) {
				configData.srcCallsign[i] = ' ';
			}
		}		
	    eeprom_write(++configData.revision, revision);
		eeprom_write_from(configData.srcCallsign, srcCallsign, 6);
		cmdSerialOut(PSTR("Set MYCALL to "));
		Serial.println(arg);
	}
}

// Set the default beacon interval
void cmdSetInterval() {
    int interval = atoi(sCmd.next());
    if(interval < 1000)
		interval = 1000;
    eeprom_write(++configData.revision, revision);
	configData.txInterval = interval;
    eeprom_write(interval, txInterval);	
	cmdSerialOut(PSTR("Transmit beacon interval set to "));
	Serial.println(interval);
}

// Set the destination callsign
void cmdSetDstCall() {
	
}

// Prototype for the packet header creation
static Packet *ax25MakePacketHeader(const char dst[6], const unsigned char dstSSID,
									const char src[6], const unsigned char srcSSID,
									const unsigned int len,
									const unsigned char digis[][7]);

// We'll store our serial output packet here received via TX
Packet *serTxPacket = NULL;

// Start collecting bytes for serTxPacket
void cmdTx() {
	cmdSerialOut(PSTR("Enter your packet:"));
	Serial.println();
	serTxPacket = ax25MakePacketHeader(configData.dstCallsign, configData.dstSSID, configData.srcCallsign, configData.srcSSID,
				512, 0x0);
	Serial.flush();
}

// Unknown command handler, just reply we don't know.
void cmdUnknown(const char *cmd) {
	cmdSerialOut(PSTR("Unknown command '"));
	Serial.print(cmd);
	Serial.println('\'');
}

void setup()  {
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
  
  // Use pin 3 for the audio output
  pinModeFast(3, OUTPUT);
  
  // Radio PTT
  pinModeFast(7, OUTPUT);
  digitalWriteFast(7, LOW);
  
  // KISS port
  pinModeFast(4, INPUT);
  pinModeFast(5, OUTPUT);
  
  // HDLC received notice
  pinModeFast(2, OUTPUT);
  pinModeFast(6, OUTPUT);
  digitalWriteFast(2, LOW);
  digitalWriteFast(6, LOW);
  
  SRAM.begin(); // Startup the external RAM
  
  // Source timer2 from clkIO (datasheet p.164)
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));

  // Do non-inverting PWM on pin OC2B (arduino pin 3) (p.159).
  // OC2A (arduino pin 11) stays in normal port operation:
  // COM2B1=1, COM2B0=0, COM2A1=0, COM2A0=0
  TCCR2A = (TCCR2A | _BV(COM2B1)) & ~(_BV(COM2B0) | _BV(COM2A1) | _BV(COM2A0));
  
  sbi (TCCR2A, WGM20);  // Mode 1  / Phase Correct PWM
  cbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);
  
  // No prescaler (p.162)
  TCCR2B = (TCCR2B & ~(_BV(CS22) | _BV(CS21))) | _BV(CS20);

  unprotoDigis[0][6] = 1;

  gps.setStream(&kiss);
  modem.start();
} 

SimpleFIFO<char,8> serPacket;

// 2 bytes HDLC, 14 bytes addressing, 2 bytes control, data, 2 bytes FCS, 2 bytes HDLC
#define PACKET_OVERHEAD 22

static Packet *ax25MakePacketHeader(const char dst[6], const unsigned char dstSSID,
									const char src[6], const unsigned char srcSSID,
									const unsigned int len,
									const unsigned char digis[][7]) {
  Packet *packet = PacketBuffer::makePacket(PACKET_OVERHEAD + len);
  unsigned char i;
  
  if(packet == NULL) // Check that we had memory
    return NULL;
    
  packet->start();
 
  // Destination address
  for(i = 0; i < 6; ++i) {
    packet->appendFCS(dst[i] << 1);
  }
  packet->appendFCS(0b11100000 | (dstSSID & 0xf) << 1);
  
  // Source address
  for(i = 0; i < 6; ++i) {
    packet->appendFCS(src[i] << 1);
  }
  if(digis == NULL) {
	  packet->appendFCS(0b01100001 | (srcSSID & 0xf) << 1); // "Last address"
  } else {
	  unsigned char d = 0;
	  packet->appendFCS(0b01100000 | (dstSSID & 0xf) << 1); // Still more for digipeaters
	  while(digis[d]) { // For each digi
		  for(i = 0; i < 6; ++i) {
			  packet->appendFCS(digis[d][i] << 1);
		  }
		  unsigned char final = 0b01100000 | (digis[d][i]&0xf)<<1;
		  if(!digis[++d]) // No more digis
			  packet->appendFCS(final | 1); // No more, final flag
		  else
			  packet->appendFCS(final);
	  }		  
  }  
  // Frame control parameters
  packet->appendFCS(0x03);
  packet->appendFCS(0xf0);
  
  return packet;
}

Packet *ax25MakePacket(const char dst[6], const unsigned char dstSSID, const char src[6], const unsigned char srcSSID, const char *data) {
  const unsigned char strLen = strlen(data);

  Packet *packet = ax25MakePacketHeader(
      dst, dstSSID,
      src, srcSSID,
      strLen, 0x0);
      
  if(packet == NULL)
    return NULL; // Check that we had memory
  
  // Append the actual data string
  for(unsigned char i = 0; i < strLen; ++i) {
    packet->appendFCS(data[i]);
  }
  
  packet->finish();
  
  // Return the packet
  return packet;
}

void printPacket(Stream *s, Packet *p) {
  cmdSerialOut(PSTR("RX: "));
  s->println(p->len);
  for(unsigned short i = 0; i < p->len; ++i)
    s->write((uint8_t)p->getByte());
  //PacketBuffer::freePacket(p);
  char callsign[7];
/*
  memcpy(callsign,p->data,6);
  callsign[6] = 0;
  for(int i = 0; i < 6; i++) {
    callsign[i] >>= 1;
    callsign[i] &= 0x7f;
  }
  s->print("To: ");
  s->print(callsign);
  if(p->data[6] >> 1 & 0xf) {
    s->print('-');
    s->print(p->data[6] >> 1 & 0x0f);
  }
  
  memcpy(callsign,p->data+7,6);
  callsign[6] = 0;
  for(int i = 0; i < 6; i++) {
    callsign[i] >>= 1;
    callsign[i] &= 0x7f;
  }
  s->print(" From: ");
  s->print(callsign);
  if(p->data[13] >> 1 & 0xf) {
    s->print('-');
    s->print(p->data[13] >> 1 & 0x0f);
  }
  
  p->data[p->len - 2] = 0;  
  s->print(" Message: ");
  s->print((char *)p->data+16);
  s->print("\n");
*/  
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
        // Something changed in the EEPROM configs
        loadConfigData();
      }
      Packet *packet = ax25MakePacketHeader(
          configData.dstCallsign, configData.dstSSID,
          configData.srcCallsign, configData.srcSSID,
          64, 0x0
        );
      //kiss.writePacket(packet);
      if(packet != NULL) { // Make sure we had memory
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
			serTxPacket = NULL;
			break;
		}
	}
  }	
  /*
  // Main USART data is available
  while(Serial.available()) {
      byte b = Serial.read();
      if(b!='\n' && b!='\r') { // Until we get end of line
        serPacket.enqueue(b);
      } else if(serPacket.count()) {
        if(serPacket.peek() == 'T') {
          // Handle the TxInterval command
          char tmp[12];
          int i = 0;
          serPacket.dequeue(); // Drop the T
          while(serPacket.count())
            tmp[i++] = serPacket.dequeue();
          tmp[i] = 0;
          int interval = atoi(tmp);
          if(interval < 1000)
            interval = 1000;
          eeprom_write(++configData.revision, revision);
          eeprom_write(interval, txInterval);
          Serial.print("Now set to ");
          Serial.println(interval);
        } else if(serPacket.peek() == 'F') {
          serPacket.dequeue();
        } else {
          // Otherwise, this is likely a serial payload to transmit
          Packet *packet = ax25MakePacketHeader(
              configData.dstCallsign, configData.dstSSID,
              configData.srcCallsign, configData.srcSSID,
              serPacket.count(), unprotoDigis);
          if(packet != NULL) { // Only if we really made a packet
            while(serPacket.count()) {
              packet->appendFCS(serPacket.dequeue());
            }
            packet->ready = 1;
            packet->finish();
            if(!modem.putTXPacket(packet)) // No room?
              PacketBuffer::freePacket(packet); // Scrap it
          } else {
            serPacket.flush();
          }
        }
      }
    }
    */
  
    // Handle the receive AFSK data
    if(modem.read() || modem.rxPacketCount()) { // A true return means something was put onto the packet FIFO
      // If we actually have data packets in the buffer, process them all now
      while(modem.rxPacketCount()) {
        Packet *packet = modem.getRXPacket();
        if(packet) {
          printPacket(&Serial, packet);
          kiss.writePacket(packet);
          PacketBuffer::freePacket(packet);
        }
      }
    }
    
    // Parse GPS data
    if(gps.process()) {
      // We got something here from the GPS
      long lat, lon, alt;
      unsigned long fix_age, course, speed, hdop;
      unsigned short sats;
      gps.get_position(&lat, &lon, &fix_age);
      if(fix_age < 5000) {
        alt = gps.altitude()*0.0328084;
        course = gps.course()/100;
        speed = gps.speed();
        sats = gps.satellites();
        hdop = gps.hdop()/100;
		Packet *p = ax25MakePacketHeader(
			configData.dstCallsign, configData.dstSSID,
			configData.srcCallsign, configData.srcSSID,
			128,
			unprotoDigis); // Create a packet header with standard digis
		p->write('!');
		p->write('/');
		p->write((const uint8_t *)Base91::encode(lat), (size_t)4);
		p->write((const uint8_t *)Base91::encode(lon), (size_t)4);
		p->write('>');
		if(speed > 0) {
			unsigned char s;
			p->write((uint8_t)((course/4)+33)); // Compressed course
			for(s = 0; s < 85; ++s) {
				unsigned short int speedRef = pgm_read_word_near(speedTable + s);
				if(speedRef >= s)
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
		//p->print(" Trial APRS Tracker - ve6slp@slepp.ca");
		p->finish();
		modem.putTXPacket(p);			
        //Serial.print(p("GPS: %ld, %ld, %ld, %lu, %lu, %lu, %lu, %u", lat, lon, alt, fix_age, course, speed, hdop, sats));
      }
    }
}
