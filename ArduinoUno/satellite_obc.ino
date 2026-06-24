#include <Wire.h>
#include <MPU9250.h>
#include <DHT.h>

// ---- Pin Definitions ----
#define DHT_PIN      7
#define DHT_TYPE     DHT22
#define LDR_PIN      A0
#define BATTERY_PIN  A1
#define STATUS_LED   13

// ---- Serial Protocol to Nucleo ----
#define PKT_SYNC     0xBC
#define PKT_VERSION  0x01
#define SERIAL_BAUD  115200

// ---- Global Objects ----
MPU9250 mpu;
DHT dht(DHT_PIN, DHT_TYPE);

// ---- Packet Structure (33 bytes total) ----
struct TelemetryPacket {
  uint8_t  sync;           // 0xBC
  uint8_t  version;        // 0x01
  uint16_t counter;        // packet sequence
  uint16_t timestamp;      // seconds since boot
  int16_t  temperature;    // °C * 100
  uint16_t humidity;       // % * 100
  int16_t  accelX;         // mg
  int16_t  accelY;         // mg
  int16_t  accelZ;         // mg
  int16_t  gyroX;          // mdps
  int16_t  gyroY;          // mdps
  int16_t  gyroZ;          // mdps
  int16_t  magX;           // mgauss
  int16_t  magY;           // mgauss
  int16_t  magZ;           // mgauss
  uint8_t  ldr;            // 0-255
  uint8_t  battery;        // voltage * 10
  uint8_t  status;         // flags
  uint16_t crc;            // CRC-16/CCITT
} __attribute__((packed));

TelemetryPacket pkt;
uint16_t packetCounter = 0;
unsigned long bootTime;

// ---- CRC-16/CCITT ----
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

// ---- Sensor Reads ----
void readSensors() {
  mpu.readSensor();

  pkt.temperature = (int16_t)(dht.readTemperature() * 100);
  pkt.humidity    = (uint16_t)(dht.readHumidity() * 100);

  pkt.accelX = (int16_t)(mpu.getAccelX_mss() * 100);   // m/s² → cm/s²
  pkt.accelY = (int16_t)(mpu.getAccelY_mss() * 100);
  pkt.accelZ = (int16_t)(mpu.getAccelZ_mss() * 100);

  pkt.gyroX  = (int16_t)(mpu.getGyroX_rads() * 1000);   // rad/s → mrad/s
  pkt.gyroY  = (int16_t)(mpu.getGyroY_rads() * 1000);
  pkt.gyroZ  = (int16_t)(mpu.getGyroZ_rads() * 1000);

  pkt.magX   = (int16_t)(mpu.getMagX_uT() * 10);        // µT → 0.1 µT
  pkt.magY   = (int16_t)(mpu.getMagY_uT() * 10);
  pkt.magZ   = (int16_t)(mpu.getMagZ_uT() * 10);

  pkt.ldr    = (uint8_t)(analogRead(LDR_PIN) >> 2);     // 0-1023 → 0-255

  int battRaw = analogRead(BATTERY_PIN);
  pkt.battery = (uint8_t)(battRaw * 5.0 / 1023.0 * 10); // volts * 10
}

// ---- Packetize & Send ----
void sendPacket() {
  pkt.sync     = PKT_SYNC;
  pkt.version  = PKT_VERSION;
  pkt.counter  = packetCounter++;
  pkt.timestamp = (uint16_t)((millis() - bootTime) / 1000);
  pkt.status   = 0x01;
  pkt.crc      = calcCRC((uint8_t*)&pkt, sizeof(pkt) - 2);

  Serial.write((uint8_t*)&pkt, sizeof(pkt));
  digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
}

// ---- Setup ----
void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(BATTERY_PIN, INPUT);

  Wire.begin();
  dht.begin();

  if (mpu.begin() < 0) {
    while (1) { digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); delay(200); }
  }
  mpu.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  mpu.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  mpu.setMagRange(MPU9250::MAG_RANGE_4912UT);

  bootTime = millis();
  memset(&pkt, 0, sizeof(pkt));
}

// ---- Loop ----
void loop() {
  readSensors();
  sendPacket();
  delay(10000);  // transmit every 10 seconds
}
