// =====================================================================
//  ESP32 LoRa AX.25 Receiver (RX)
//  ─────────────────────────────────────────────────────────────────
//  Flow: Antenna → LoRa SX1278 → SPI (ISP) → ESP32 → AX.25 Decode
//        → Display telemetry on Serial
//
//  ISP Protocol = SPI (we use it to read from the LoRa radio module)
//  Must match TX frequency and LoRa parameters.
// =====================================================================

#include <SPI.h>
#include <LoRa.h>

// ======================== CONFIGURATION ============================

#define FREQ_MHZ    433                 // Must MATCH the transmitter!
#define FREQ        (FREQ_MHZ * 1E6)

// --- LoRa (SPI / ISP) Pin Connections ---
#define LORA_NSS    5
#define LORA_RST    14
#define LORA_DIO0   2
#define LORA_SCK    18
#define LORA_MOSI   23
#define LORA_MISO   19

// --- LoRa Radio Parameters (must match TX) ---
#define SF          12
#define BW          125E3
#define CR          8

// --- Status LED ---
#define LED_PIN     2

// ==================== ANTENNA LENGTH ==============================
#define ANTENNA_CM  ((300.0 / FREQ_MHZ) / 4.0 * 0.95)

// ======================== STATISTICS ===============================

unsigned long packetsSeen   = 0;
unsigned long packetsGood   = 0;
unsigned long packetsBad    = 0;
long lastRSSI = 0;
float lastSNR = 0;

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

// ======================== AX.25 ADDRESS DECODE =====================

void decodeAddress(uint8_t* addrField, char* call, uint8_t* ssid) {
  for (uint8_t i = 0; i < 6; i++) {
    call[i] = (addrField[i] >> 1) & 0x7F;  // right-shift to restore ASCII
    if (call[i] == ' ') call[i] = '\0';
  }
  call[6] = '\0';
  *ssid = (addrField[6] >> 1) & 0x0F;
}

// ======================== PROCESS AX.25 FRAME ======================

void processAX25Frame(uint8_t* buf, int len) {
  // Minimum valid AX.25: flag(1) + dst(7) + src(7) + ctrl(1) + pid(1) + CRC(2) = 19
  if (len < 19) return;

  // Find start flag
  int start = 0;
  for (int i = 0; i < len; i++) {
    if (buf[i] == 0x7E) { start = i; break; }
  }
  if (buf[start] != 0x7E) return;

  int idx = start + 1;

  // Decode addresses
  char destCall[7], srcCall[7];
  uint8_t destSSID, srcSSID;
  decodeAddress(&buf[idx], destCall, &destSSID); idx += 7;
  decodeAddress(&buf[idx], srcCall, &srcSSID);   idx += 7;

  uint8_t control = buf[idx++];
  uint8_t pid     = buf[idx++];

  int payloadLen = len - idx - 2;  // -2 for CRC
  if (payloadLen < 0) return;

  char payload[128];
  memcpy(payload, &buf[idx], payloadLen);
  payload[payloadLen] = '\0';

  // CRC check
  uint16_t crcRecv = buf[len - 2] | (buf[len - 1] << 8);
  uint16_t crcCalc = calcCRC(&buf[start + 1], len - start - 3);
  bool crcOK = (crcRecv == crcCalc);

  if (crcOK) packetsGood++;
  else       packetsBad++;

  // ---- DISPLAY DECODED FRAME ----
  Serial.println("┌──────────────────────────────────────────────┐");
  Serial.println("│              AX.25 Frame Decoded             │");
  Serial.println("├──────────────────────────────────────────────┤");
  Serial.printf("│ From:  %-4s-%u                          │\n", srcCall, srcSSID);
  Serial.printf("│ To:    %-4s-%u                          │\n", destCall, destSSID);
  Serial.printf("│ Ctrl:  0x%02X  UI Frame                   │\n", control);
  Serial.printf("│ PID:   0x%02X                            │\n", pid);
  Serial.printf("│ Payload: %-27s  │\n", payload);
  Serial.printf("│ CRC:   %s                             │\n", crcOK ? "OK" : "FAIL");
  Serial.printf("│ RSSI:  %-3d dBm   SNR: %-5.1f dB         │\n", lastRSSI, lastSNR);
  Serial.println("└──────────────────────────────────────────────┘");

  // Parse telemetry payload: "T<temp>,H<hum>,P<pktnum>"
  if (payload[0] == 'T') {
    float temp, hum;
    unsigned int pkt;
    if (sscanf(payload, "T%f,H%f,P%u", &temp, &hum, &pkt) == 3) {
      Serial.printf("📡 TELEMETRY → Temp: %.1f°C  Hum: %.0f%%  Pkt: %u\n\n",
        temp, hum, pkt);
    }
  }
}

// ======================== LORA CALLBACK ============================

void onLoRaReceive(int packetSize) {
  if (packetSize == 0) return;

  uint8_t buf[256];
  int len = 0;

  while (LoRa.available() && len < 255) {
    buf[len++] = LoRa.read();
  }

  lastRSSI = LoRa.packetRssi();
  lastSNR  = LoRa.packetSnr();
  packetsSeen++;

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  processAX25Frame(buf, len);
}

// ======================== SETUP ====================================

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n=============================================");
  Serial.println("  ESP32 ← LoRa SX1278 ← Antenna (RX)");
  Serial.println("  AX.25 Frame Decoder");
  Serial.println("=============================================");
  Serial.printf("Frequency:      %d MHz\n", FREQ_MHZ);
  Serial.printf("Antenna length: %.2f cm (quarter-wave)\n", ANTENNA_CM);
  Serial.printf("SF: %d  BW: %.0f kHz  CR: 4/%d\n\n", SF, BW/1000, CR);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // --- Initialize SPI (ISP) and LoRa ---
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(FREQ)) {
    Serial.println("[FAIL] LoRa init failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BW);
  LoRa.setCodingRate4(CR);
  LoRa.enableCrc();

  LoRa.onReceive(onLoRaReceive);
  LoRa.receive();

  Serial.println("[OK] Listening for AX.25 packets...\n");
}

// ======================== LOOP =====================================

void loop() {
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 10000) {
    lastStats = millis();
    Serial.printf("[STATS] Seen: %lu | OK: %lu | Bad: %lu | RSSI: %ld | SNR: %.1f\n\n",
      packetsSeen, packetsGood, packetsBad, lastRSSI, lastSNR);
    LoRa.receive(); // keep listening
  }
}
