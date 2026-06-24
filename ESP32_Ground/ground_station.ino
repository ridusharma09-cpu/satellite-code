#include <SPI.h>
#include <LoRa.h>

// ---- Pin Definitions (ESP32 default LoRa pins) ----
#define LORA_NSS   5
#define LORA_RST   14
#define LORA_DIO0  2
#define LORA_SCK   18
#define LORA_MOSI  23
#define LORA_MISO  19

// ---- Radio Config (match satellite) ----
#define FREQ        433E6
#define SF          12
#define BW          125E3
#define CR          8

// ---- Telemetry Packet Structure (mirrors Uno) ----
struct TelemetryPacket {
  uint8_t  sync;           // 0xBC
  uint8_t  version;        // 0x01
  uint16_t counter;        // packet sequence
  uint16_t timestamp;      // seconds since boot
  int16_t  temperature;    // °C * 100
  uint16_t humidity;       // % * 100
  int16_t  accelX, accelY, accelZ;
  int16_t  gyroX,  gyroY,  gyroZ;
  int16_t  magX,   magY,   magZ;
  uint8_t  ldr;
  uint8_t  battery;
  uint8_t  status;
  uint16_t crc;
} __attribute__((packed));

// ---- Packet Counter (for display) ----
unsigned long packetsReceived = 0;
unsigned long lastRSSI = 0;
float lastSNR = 0;

// ---- Serial output (for PC / OpenC3) ----
char csvBuffer[256];

// ---- Hex Dump ----
void printHex(uint8_t* data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
}

// ---- Display Packet ----
void displayPacket(TelemetryPacket* p) {
  float temp = p->temperature / 100.0;
  float hum  = p->humidity / 100.0;
  float ax   = p->accelX / 100.0;
  float ay   = p->accelY / 100.0;
  float az   = p->accelZ / 100.0;
  float gx   = p->gyroX / 1000.0;
  float gy   = p->gyroY / 1000.0;
  float gz   = p->gyroZ / 1000.0;
  float mx   = p->magX / 10.0;
  float my   = p->magY / 10.0;
  float mz   = p->magZ / 10.0;
  float batt = p->battery / 10.0;

  Serial.println("╔══════════════════════════════════╗");
  Serial.println("║     SATELLITE TELEMETRY          ║");
  Serial.println("╠══════════════════════════════════╣");
  Serial.printf("║ Packet #:    %5u              ║\n", p->counter);
  Serial.printf("║ Time:        %5u s            ║\n", p->timestamp);
  Serial.printf("║ RSSI:        %4d dBm           ║\n", lastRSSI);
  Serial.printf("║ SNR:         %5.1f dB           ║\n", lastSNR);
  Serial.printf("║ Temp:        %6.2f °C           ║\n", temp);
  Serial.printf("║ Humidity:    %5.1f %%            ║\n", hum);
  Serial.printf("║ LDR:         %3u/255            ║\n", p->ldr);
  Serial.printf("║ Battery:     %4.1f V             ║\n", batt);
  Serial.println("╠─────────── ATTITUDE ─────────────╣");
  Serial.printf("║ Accel:  %6.2f  %6.2f  %6.2f m/s²║\n", ax, ay, az);
  Serial.printf("║ Gyro:   %6.2f  %6.2f  %6.2f rad/s║\n", gx, gy, gz);
  Serial.printf("║ Mag:    %6.1f  %6.1f  %6.1f µT  ║\n", mx, my, mz);
  Serial.println("╚══════════════════════════════════╝");
  Serial.println();
}

// ---- CSV Output for OpenC3 ----
void csvOutput(TelemetryPacket* p) {
  sprintf(csvBuffer,
    "%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u",
    p->counter, p->timestamp,
    p->temperature, p->humidity,
    p->accelX, p->accelY, p->accelZ,
    p->gyroX, p->gyroY, p->gyroZ,
    p->magX, p->magY, p->magZ,
    p->ldr, p->battery, p->status,
    lastRSSI, (int)(lastSNR * 10)
  );
  Serial.println(csvBuffer);
}

// ---- CRC-16 Check ----
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

bool verifyPacket(TelemetryPacket* p) {
  if (p->sync != 0xBC) return false;
  uint16_t receivedCRC = p->crc;
  p->crc = 0;
  uint16_t computedCRC = calcCRC((uint8_t*)p, sizeof(TelemetryPacket));
  p->crc = receivedCRC;
  return receivedCRC == computedCRC;
}

// ---- LoRa Receive Handler ----
void onLoRaReceive(int packetSize) {
  if (packetSize == 0) return;

  uint8_t buf[sizeof(TelemetryPacket)];
  int len = 0;

  while (LoRa.available() && len < sizeof(TelemetryPacket)) {
    buf[len++] = LoRa.read();
  }
  if (len < sizeof(TelemetryPacket)) return;

  lastRSSI = LoRa.packetRssi();
  lastSNR  = LoRa.packetSnr();

  TelemetryPacket* pkt = (TelemetryPacket*)buf;
  if (!verifyPacket(pkt)) return;

  packetsReceived++;

  Serial.print("PKT,");
  csvOutput(pkt);
  displayPacket(pkt);
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

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

  LoRa.onReceive(onLoRaReceive);
  LoRa.receive();

  Serial.println("Ground Station Ready — listening on 433 MHz");
  Serial.println("Format: PKT,count,time,temp,hum,ax,ay,az,gx,gy,gz,mx,my,mz,ldr,batt,status,rssi,snr");
  Serial.println();
}

// ---- Loop ----
void loop() {
  // LoRa receive is interrupt-driven via onReceive callback
  // Just update status every 10 seconds
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 10000) {
    lastStatus = millis();
    Serial.printf("[STATUS] %lu packets received | RSSI: %d | SNR: %.1f\n",
      packetsReceived, lastRSSI, lastSNR);
    LoRa.receive(); // keep listening
  }
}
