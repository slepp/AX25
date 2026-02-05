#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// Hardware Pin Assignments
// ============================================================
// These must match the physical board wiring.

// UART (hardware serial)
// Pins 0, 1 are used by Serial (hardware UART)

// KISS TNC software serial
static constexpr uint8_t PIN_KISS_RX       = 4;
static constexpr uint8_t PIN_KISS_TX       = 5;

// AFSK audio output (must be OC2B for PWM)
static constexpr uint8_t PIN_AFSK_OUT      = 3;

// Radio push-to-talk
static constexpr uint8_t PIN_PTT           = 7;

// Indicator LEDs
static constexpr uint8_t PIN_LED_RX        = 2;
static constexpr uint8_t PIN_LED_MODEM     = 6;

// SPI (hardware, directly manipulated via registers)
// Pin 10 = SS, 11 = MOSI, 12 = MISO, 13 = SCK

// ============================================================
// EEPROM Configuration
// ============================================================
#define EEPROM_VERSION 8

// Minimum allowed TX beacon interval (ms)
static constexpr uint16_t MIN_TX_INTERVAL  = 1000;

// ============================================================
// Serial Defaults
// ============================================================
static constexpr uint32_t DEFAULT_SERIAL_BAUD = 57600;
static constexpr uint32_t DEFAULT_KISS_BAUD   = 4800;

// ============================================================
// Protocol Constants
// ============================================================

// AX.25 packet overhead: 2 HDLC + 14 addressing + 2 control + 2 FCS + 2 HDLC
static constexpr uint8_t AX25_PACKET_OVERHEAD = 22;

// AFSK modem parameters
static constexpr uint16_t AFSK_REFCLK         = 9600;
static constexpr uint16_t AFSK_BAUD           = 1200;
static constexpr uint16_t AFSK_MARK_FREQ      = 1200;
static constexpr uint16_t AFSK_SPACE_FREQ     = 2200;

// TX preamble flags (6.7ms each, 23 = ~153ms total preamble)
static constexpr uint8_t TX_PREAMBLE_FLAGS    = 23;

// Maximum packets to send per TX burst
static constexpr uint8_t TX_MAX_PACKETS       = 3;

// Callsign field length in AX.25
static constexpr uint8_t AX25_CALLSIGN_LEN    = 6;

// ============================================================
// CPU / Timing
// ============================================================
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#endif
