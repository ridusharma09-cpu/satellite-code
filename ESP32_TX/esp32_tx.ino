#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>

// ============================================================
//  ESP32 Transmitter: DHT11 → SPI (LoRa) → AX.25 → Antenna
// ============================================================

// ---- DHT11 Pin ----
#define DHT_PIN      4
#define DHT_TYPE     DHT11

// ---- LoRa Pin Connections (SPI / ISP) ----
#define LORA_NSS     5
#define LORA_RST     14
#define LORA_DIO0    2
#define LORA_SCK     18
#define LORA_MOSI    23
#define LORA_MISO    19

// ---- Radio Config ----
#define FREQ         433E6
#define SF           12
#define BW           125E3
#define CR           8
#define TX_POWER     20

// ---- AX.25 Callsigns ----
#define MY_CALL      "SAT1"
#define MY_SSID      1
#define DEST_CALL    "GND"
#define DEST_SSID    0

// ---- Timing ----
#define TX_INTERVAL  30000   // 30 seconds

// ---- Objects ----
DHT dht(DHT_PIN, DHT_TYPE);

// ---- Packet Counter ----
uint16_t packetNum = 0;

// ============================================================
//  CRC-16/CCITT
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
//  Encode AX.25 Address Field
//  Each callsign byte is left-shifted by 1 (AX.25 spec)
// ============================================================
void encodeAddress(char* call, uint8_t ssid, uint8_t* buf, uint8_t isLast) {
  for (uint8_t i = 0; i < 6; i++) {
    char c = (i < strlen(call)) ? call[i] : ' ';
    buf[i] = (uint8_t)(c << 1);
  }
  // SSID byte: bits 0-3 = SSID, bit 5 = AEA?, bit 7 = HDLC address bit
  // For destinations: bit 7 = 0 (command), 1 = 0
  // For sources: bit 7 = 0, bit 0 = C/R
  // Last address byte: bit 0 = 1 if this is the last address field
  buf[6] = 0x60 | (ssid << 1) | (isLast ? 0x01 : 0x00);
}

// ============================================================
//  Build & Transmit AX.25 Frame
// ============================================================
void transmitAX25(float temperature, float humidity) {
  // Build payload: "TEMPERATURE,HUMIDITY,PACKET_NUM,RSSI"
  char payload[64];
  snprintf(payload, sizeof(payload),
    "T%.1f,H%.0f,P%u",
    temperature, humidity, packetNum++);

  uint8_t payloadLen = strlen(payload);

  // Calculate total frame size for CRC
  // Flag(1) + Dest(7) + Src(7) + Ctrl(1) + PID(1) + Payload(n) + CRC(2)
  uint16_t frameLen = 1 + 7 + 7 + 1 + 1 + payloadLen;
  uint8_t* frame = (uint8_t*)malloc(frameLen + 2); // +2 for CRC
  uint16_t idx = 0;

  // Flag
  frame[idx++] = 0x7E;

  // Destination address
  uint8_t destAddr[7];
  encodeAddress(DEST_CALL, DEST_SSID, destAddr, 0);
  memcpy(&frame[idx], destAddr, 7);
  idx += 7;

  // Source address (last = 1, so bit 0 = 1)
  uint8_t srcAddr[7];
  encodeAddress(MY_CALL, MY_SSID, srcAddr, 1);
  memcpy(&frame[idx], srcAddr, 7);
  idx += 7;

  // Control field (0x03 = UI frame, Unnumbered Information)
  frame[idx++] = 0x03;

  // PID field (0xF0 = no layer 3)
  frame[idx++] = 0xF0;

  // Payload
  memcpy(&frame[idx], payload, payloadLen);
  idx += payloadLen;

  // CRC-16 (over everything except the flag)
  uint16_t crc = calcCRC(&frame[1], idx - 1);
  frame[idx++] = crc & 0xFF;
  frame[idx++] = (crc >> 8) & 0xFF;

  // ---- Transmit via LoRa (SPI protocol) ----
  LoRa.beginPacket();
  LoRa.write(frame, idx);
  LoRa.endPacket();

  free(frame);

  Serial.print("[TX] Packet #");
  Serial.print(packetNum - 1);
  Serial.print(" | ");
  Serial.print(payload);
  Serial.print(" | RSSI: ");
  Serial.println(LoRa.packetRssi());
}

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\nESP32 Satellite Transmitter — AX.25 over LoRa");

  dht.begin();

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
  LoRa.setTxPower(TX_POWER);

  Serial.print("Listening on ");
  Serial.print(FREQ / 1E6);
  Serial.println(" MHz");
  Serial.println("Waiting for transmissions...\n");
}

// ============================================================
//  Loop — Read DHT11 & Transmit
// ============================================================
void loop() {
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();

  if (isnan(temp) || isnan(hum)) {
    Serial.println("DHT11 read error");
  } else {
    transmitAX25(temp, hum);
  }

  delay(TX_INTERVAL);
}
