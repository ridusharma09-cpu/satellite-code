#include <SPI.h>
#include <LoRa.h>

// ---- Pin Definitions (Nucleo Arduino Header) ----
#define LORA_NSS   D10   // PB6
#define LORA_RST   D8    // PA9
#define LORA_DIO0  D2    // PA10
#define LORA_SCK   D13   // PA5
#define LORA_MOSI  D11   // PA7
#define LORA_MISO  D12   // PA6
#define STATUS_LED D7    // internal LED

// ---- Radio Config ----
#define FREQ        433E6
#define SF          12
#define BW          125E3
#define CR          8
#define TX_POWER    20

// ---- Serial Protocol from Uno (same struct) ----
#define PKT_SYNC    0xBC
#define SERIAL_BAUD 115200

struct TelemetryPacket {
  uint8_t  sync;
  uint8_t  version;
  uint16_t counter;
  uint16_t timestamp;
  int16_t  temperature;
  uint16_t humidity;
  int16_t  accelX, accelY, accelZ;
  int16_t  gyroX,  gyroY,  gyroZ;
  int16_t  magX,   magY,   magZ;
  uint8_t  ldr;
  uint8_t  battery;
  uint8_t  status;
  uint16_t crc;
} __attribute__((packed));

// ---- AX.25-like Frame Header ----
struct AX25Frame {
  uint8_t  flag;          // 0x7E
  uint8_t  destAddr[7];   // destination callsign + SSID
  uint8_t  srcAddr[7];    // source callsign + SSID
  uint8_t  control;       // 0x03 = UI frame
  uint8_t  pid;           // 0xF0 = no layer 3
  // payload follows (variable length)
} __attribute__((packed));

// ---- CCSDS-like Primary Header ----
struct CCSDSHeader {
  uint8_t  versionType;   // v=00, type=1, secHdr=0
  uint8_t  secHdrFlag;    // 0
  uint16_t apid;          // Application Process ID
  uint8_t  seqFlags;      // 0b11 = unsegmented
  uint8_t  seqCount;      // sequence count
  uint16_t dataLength;    // payload length - 1
} __attribute__((packed));

// ---- Globals ----
TelemetryPacket pkt;
uint16_t radioPacketCounter = 0;
unsigned long lastTxTime = 0;
const unsigned long TX_INTERVAL = 10000; // 10 seconds

// ---- Init LoRa ----
bool initLoRa() {
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(FREQ)) {
    return false;
  }
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BW);
  LoRa.setCodingRate4(CR);
  LoRa.setTxPower(TX_POWER);
  LoRa.enableCrc();
  return true;
}

// ---- Build AX.25 Frame ----
void transmitAX25(uint8_t* payload, uint8_t payloadLen) {
  LoRa.beginPacket();

  // Flag
  LoRa.write(0x7E);

  // Destination address: callsign (6 chars) + SSID
  uint8_t dest[7] = {'G', 'R', 'N', 'D', 'S', 'T', 0x60}; // GRNDST-0
  for (int i = 0; i < 7; i++) LoRa.write(dest[i]);

  // Source address
  uint8_t src[7] = {'S', 'A', 'T', '1', 0x20, 0x20, 0x61}; // SAT1-1
  for (int i = 0; i < 7; i++) LoRa.write(src[i]);

  // Control + PID
  LoRa.write(0x03);  // UI frame
  LoRa.write(0xF0);  // no layer 3

  // Payload
  LoRa.write(payload, payloadLen);

  LoRa.endPacket();
}

// ---- Build CCSDS Frame ----
void transmitCCSDS(uint8_t* payload, uint8_t payloadLen) {
  LoRa.beginPacket();

  // Primary header (6 bytes)
  uint8_t versionType = 0x10;  // v=00, type=1 (telemetry)
  uint16_t apid = 0x0001;
  uint8_t seqFlags = 0xC0;     // unsegmented
  uint8_t seqCount = radioPacketCounter & 0x3F;
  uint16_t dataLen = payloadLen - 1;

  LoRa.write(versionType);
  LoRa.write(apid >> 8);
  LoRa.write(apid & 0xFF);
  LoRa.write(seqFlags | seqCount);
  LoRa.write(dataLen >> 8);
  LoRa.write(dataLen & 0xFF);

  // Payload
  LoRa.write(payload, payloadLen);

  LoRa.endPacket();
}

// ---- CRC Check ----
uint16_t calcCRC(uint8_t* data, uint16_t len) {
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

bool verifyCRC(TelemetryPacket* p) {
  uint16_t receivedCRC = p->crc;
  p->crc = 0;
  uint16_t computedCRC = calcCRC((uint8_t*)p, sizeof(TelemetryPacket));
  p->crc = receivedCRC;
  return receivedCRC == computedCRC;
}

// ---- Read from Uno over Serial ----
bool readSerialPacket() {
  static uint8_t buf[sizeof(TelemetryPacket)];
  static uint8_t idx = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (idx == 0 && b != PKT_SYNC) continue;  // sync hunt
    buf[idx++] = b;
    if (idx >= sizeof(TelemetryPacket)) {
      idx = 0;
      memcpy(&pkt, buf, sizeof(TelemetryPacket));

      if (pkt.sync == PKT_SYNC && verifyCRC(&pkt)) {
        return true;
      }
    }
  }
  return false;
}

// ---- Setup ----
void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  if (!initLoRa()) {
    while (1) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(100);
    }
  }

  digitalWrite(STATUS_LED, HIGH);
  delay(500);
  digitalWrite(STATUS_LED, LOW);
}

// ---- Loop ----
void loop() {
  if (readSerialPacket()) {
    uint8_t* payload = (uint8_t*)&pkt;
    uint8_t payloadLen = sizeof(TelemetryPacket);

    // Transmit using AX.25 framing (or toggle with CCSDS)
    transmitAX25(payload, payloadLen);
    // transmitCCSDS(payload, payloadLen);

    radioPacketCounter++;
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
  }
}
