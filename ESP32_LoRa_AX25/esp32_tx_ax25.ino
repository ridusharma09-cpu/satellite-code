// =====================================================================
//  ESP32 LoRa AX.25 Transmitter (TX)
//  ─────────────────────────────────────────────────────────────────
//  Flow: DHT11 → ESP32 → AX.25 Packetize → SPI (ISP) → LoRa SX1278
//        → Antenna (quarter-wave copper wire) → Wireless TX
//
//  ISP Protocol = SPI (we use it to talk to the LoRa radio module)
//  Antenna length auto-calculated from FREQ_MHZ below.
// =====================================================================

#include <SPI.h>    // ISP protocol — Serial Peripheral Interface
#include <LoRa.h>
#include <DHT.h>

// ======================== CONFIGURATION ============================

#define FREQ_MHZ    433                 // Change this to your frequency
#define FREQ        (FREQ_MHZ * 1E6)    // Convert to Hz for LoRa lib

// --- DHT11 Pin ---
#define DHT_PIN     4
#define DHT_TYPE    DHT11

// --- LoRa (SPI / ISP) Pin Connections ---
#define LORA_NSS    5
#define LORA_RST    14
#define LORA_DIO0   2
#define LORA_SCK    18
#define LORA_MOSI   23
#define LORA_MISO   19

// --- LoRa Radio Parameters ---
#define SF          12       // Spreading Factor (6-12)
#define BW          125E3    // Bandwidth (7.8E3 to 500E3)
#define CR          8        // Coding Rate (5-8)
#define TX_POWER    20       // dBm (2-20)

// --- AX.25 Configuration ---
#define MY_CALL     "SAT1"
#define MY_SSID     1
#define DEST_CALL   "GND"
#define DEST_SSID   0

#define TX_INTERVAL 10000    // milliseconds between transmissions

// ==================== ANTENNA LENGTH CALCULATION ===================
//
//  Quarter-wave (λ/4) monopole antenna formula:
//     Length (cm) = (300 / Freq_MHz) / 4 × 0.95 (velocity factor)
//                   └────── λ ──────┘
//
//  Examples:
//  ┌──────────┬──────────┬──────────────┐
//  │ Frequency│ λ (meters)│ Antenna (cm)  │
//  ├──────────┼──────────┼──────────────┤
//  │ 433 MHz  │  0.692 m │   16.44 cm   │
//  │ 868 MHz  │  0.345 m │    8.20 cm   │
//  │ 915 MHz  │  0.328 m │    7.78 cm   │
//  │ 169 MHz  │  1.775 m │   42.16 cm   │
//  └──────────┴──────────┴──────────────┘
//
//  For a half-wave dipole: Length = (300 / Freq_MHz) / 2 × 0.95
// ==================================================================
#define ANTENNA_CM  ((300.0 / FREQ_MHZ) / 4.0 * 0.95)

// ======================== GLOBALS ==================================

DHT dht(DHT_PIN, DHT_TYPE);
uint16_t packetNum = 0;

// ======================== CRC-16/CCITT =============================

uint16_t calcCRC(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0x8408;
      else              crc >>= 1;
    }
  }
  return ~crc;
}

// ======================== AX.25 ADDRESS ENCODING ===================

void encodeAddress(const char* call, uint8_t ssid, uint8_t* buf, uint8_t isLast) {
  for (uint8_t i = 0; i < 6; i++) {
    char c = (i < strlen(call)) ? call[i] : ' ';
    buf[i] = (uint8_t)(c << 1);              // left-shift by 1 per AX.25
  }
  // SSID byte: bits 1-4 = SSID, bit 0 = 1 if last address
  buf[6] = 0x60 | ((ssid & 0x0F) << 1) | (isLast ? 0x01 : 0x00);
}

// ======================== BUILD AX.25 FRAME & TRANSMIT =============

void transmitAX25(float temperature, float humidity) {
  // --- Payload: comma-separated values ---
  char payload[64];
  snprintf(payload, sizeof(payload),
    "T%.1f,H%.0f,P%u", temperature, humidity, packetNum++);

  uint8_t payloadLen = strlen(payload);

  // --- Build complete AX.25 frame in buffer ---
  uint8_t buf[256];
  uint16_t idx = 0;

  // Flag byte
  buf[idx++] = 0x7E;

  // Destination address (7 bytes)
  uint8_t destAddr[7];
  encodeAddress(DEST_CALL, DEST_SSID, destAddr, 0);
  memcpy(&buf[idx], destAddr, 7); idx += 7;

  // Source address (7 bytes) — mark as last
  uint8_t srcAddr[7];
  encodeAddress(MY_CALL, MY_SSID, srcAddr, 1);
  memcpy(&buf[idx], srcAddr, 7); idx += 7;

  // Control field: 0x03 = UI (Unnumbered Information) frame
  buf[idx++] = 0x03;

  // PID field: 0xF0 = No Layer 3 protocol
  buf[idx++] = 0xF0;

  // Copy payload
  memcpy(&buf[idx], payload, payloadLen); idx += payloadLen;

  // CRC-16 over everything between flag and CRC
  // (i.e. from buf[1] to buf[idx-1])
  uint16_t crc = calcCRC(&buf[1], idx - 1);
  buf[idx++] = crc & 0xFF;       // CRC low byte
  buf[idx++] = (crc >> 8) & 0xFF; // CRC high byte

  // --- Transmit via LoRa (SPI/ISP protocol) ---
  LoRa.beginPacket();
  LoRa.write(buf, idx);
  LoRa.endPacket(true);          // true = async, non-blocking

  Serial.print("[TX] #"); Serial.print(packetNum - 1);
  Serial.print(" | "); Serial.print(payload);
  Serial.print(" | "); Serial.print(idx); Serial.print(" bytes");
  Serial.println();
}

// ======================== SETUP ====================================

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n=============================================");
  Serial.println("  ESP32 → AX.25 → LoRa SX1278 → Antenna");
  Serial.println("=============================================");
  Serial.printf("Frequency:      %d MHz\n", FREQ_MHZ);
  Serial.printf("Antenna length: %.2f cm (quarter-wave)\n", ANTENNA_CM);
  Serial.printf("Spreading Fact: SF%d\n", SF);
  Serial.printf("Bandwidth:      %.0f kHz\n", BW / 1000);
  Serial.printf("Coding Rate:    4/%d\n", CR);
  Serial.printf("TX Power:       %d dBm\n", TX_POWER);
  Serial.printf("Interval:       %d s\n\n", TX_INTERVAL / 1000);

  Serial.println("Cut a copper wire to the length above and solder");
  Serial.println("it to the ANT pin on the LoRa SX1278 module.\n");

  dht.begin();

  // --- Initialize SPI (ISP protocol) and LoRa ---
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(FREQ)) {
    Serial.println("[FAIL] LoRa init failed!");
    Serial.println("Check wiring: NSS=5, RST=14, DIO0=2, SCK=18, MOSI=23, MISO=19");
    while (1);
  }

  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BW);
  LoRa.setCodingRate4(CR);
  LoRa.setTxPower(TX_POWER);

  Serial.println("[OK] LoRa initialized. Starting transmissions...\n");
}

// ======================== MAIN LOOP ================================

void loop() {
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();

  if (isnan(temp) || isnan(hum)) {
    Serial.println("[ERR] DHT11 read failed — check wiring");
  } else {
    transmitAX25(temp, hum);
  }

  // --- Iterate transmission at the configured interval ---
  delay(TX_INTERVAL);
}
