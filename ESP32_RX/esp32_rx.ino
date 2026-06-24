#include <SPI.h>
#include <LoRa.h>

// ============================================================
//  ESP32 Receiver: Antenna → LoRa → SPI → AX.25 Decode → Display
// ============================================================

// ---- LoRa Pin Connections (SPI / ISP) ----
#define LORA_NSS     5
#define LORA_RST     14
#define LORA_DIO0    2
#define LORA_SCK     18
#define LORA_MOSI    23
#define LORA_MISO    19

// ---- Radio Config (must match TX) ----
#define FREQ         433E6
#define SF           12
#define BW           125E3
#define CR           8

// ---- Status LED ----
#define LED_PIN      2

// ---- Statistics ----
unsigned long packetsReceived = 0;
unsigned long packetsValid    = 0;
unsigned long packetsInvalid  = 0;
long lastRSSI = 0;
float lastSNR = 0;

// ============================================================
//  CRC-16/CCITT (verify AX.25 frame integrity)
// ============================================================
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

// ============================================================
//  Decode AX.25 Address Field
//  Right-shift by 1 to restore ASCII
// ============================================================
void decodeAddress(uint8_t* addrField, char* call, uint8_t* ssid) {
  for (uint8_t i = 0; i < 6; i++) {
    call[i] = (addrField[i] >> 1) & 0x7F;
    if (call[i] == ' ') call[i] = '\0';
  }
  call[6] = '\0';
  *ssid = (addrField[6] >> 1) & 0x0F;
}

// ============================================================
//  Parse AX.25 Frame
// ============================================================
void processAX25Frame(uint8_t* buf, int len) {
  if (len < 18) return;  // flag(1) + dest(7) + src(7) + ctrl(1) + pid(1) + crc(2) = 19 minimum

  // Find the flag (0x7E) start
  int start = 0;
  for (int i = 0; i < len; i++) {
    if (buf[i] == 0x7E) { start = i; break; }
  }
  if (start == 0 && buf[0] != 0x7E) return;

  int idx = start + 1;
  if (idx + 14 > len) return;

  // Decode addresses
  char destCall[7], srcCall[7];
  uint8_t destSSID, srcSSID;

  decodeAddress(&buf[idx], destCall, &destSSID);
  idx += 7;
  decodeAddress(&buf[idx], srcCall, &srcSSID);
  idx += 7;

  // Control & PID
  uint8_t control = buf[idx++];
  uint8_t pid     = buf[idx++];

  // Payload
  int payloadLen = len - idx - 2; // -2 for CRC
  if (payloadLen < 0) return;

  char payload[128];
  memcpy(payload, &buf[idx], payloadLen);
  payload[payloadLen] = '\0';
  idx += payloadLen;

  // CRC check
  uint16_t crcRecv = buf[idx] | (buf[idx + 1] << 8);
  uint16_t crcCalc = calcCRC(&buf[start + 1], len - start - 3); // skip flag and 2 CRC bytes

  bool crcOK = (crcRecv == crcCalc);
  if (crcOK) packetsValid++;
  else       packetsInvalid++;

  // ---- Display ----
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║          AX.25 Frame Received        ║");
  Serial.println("╠══════════════════════════════════════╣");
  Serial.printf("║ From:   %s-%u                   ║\n", srcCall, srcSSID);
  Serial.printf("║ To:     %s-%u                   ║\n", destCall, destSSID);
  Serial.printf("║ Ctrl:   0x%02X                         ║\n", control);
  Serial.printf("║ PID:    0x%02X                         ║\n", pid);
  Serial.printf("║ Payload: %s                     ║\n", payload);
  Serial.printf("║ CRC:    %s                        ║\n", crcOK ? "OK" : "FAIL");
  Serial.printf("║ RSSI:   %d dBm                      ║\n", lastRSSI);
  Serial.printf("║ SNR:    %.1f dB                       ║\n", lastSNR);
  Serial.println("╚══════════════════════════════════════╝");

  // Parse telemetry from payload
  if (payload[0] == 'T') {
    float temp, hum;
    unsigned int pktNum;
    if (sscanf(payload, "T%f,H%f,P%u", &temp, &hum, &pktNum) == 3) {
      Serial.printf("[TELEMETRY] Temp: %.1f°C | Humidity: %.0f%% | Packet: %u\n\n",
        temp, hum, pktNum);
    }
  }
}

// ============================================================
//  LoRa Receive Handler (interrupt-driven via SPI)
// ============================================================
void onLoRaReceive(int packetSize) {
  if (packetSize == 0) return;

  uint8_t buf[256];
  int len = 0;

  while (LoRa.available() && len < 255) {
    buf[len++] = LoRa.read();
  }

  lastRSSI = LoRa.packetRssi();
  lastSNR  = LoRa.packetSnr();
  packetsReceived++;

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  processAX25Frame(buf, len);
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\nESP32 Ground Station Receiver — AX.25 over LoRa\n");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ---- Init SPI (ISP) & LoRa ----
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(FREQ)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BW);
  LoRa.setCodingRate4(CR);
  LoRa.enableCrc();

  // Register callback (interrupt-driven)
  LoRa.onReceive(onLoRaReceive);
  LoRa.receive();

  Serial.print("Listening on ");
  Serial.print(FREQ / 1E6);
  Serial.println(" MHz");
  Serial.println("Waiting for transmissions...\n");
}

// ============================================================
//  Loop — status updates only
// ============================================================
void loop() {
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 15000) {
    lastStats = millis();
    Serial.printf("[STATS] RX: %lu | Valid: %lu | Invalid: %lu | RSSI: %ld | SNR: %.1f\n\n",
      packetsReceived, packetsValid, packetsInvalid, lastRSSI, lastSNR);
    LoRa.receive(); // re-arm
  }
}
