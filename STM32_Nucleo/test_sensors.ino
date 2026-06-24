// ============================================================
//  STM32 Nucleo-F401RE — Detect All Sensors & LoRa
//  Upload this first to verify every module works.
//  Open Serial Monitor at 115200 baud.
// ============================================================

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <MPU9250.h>
#include <DHT.h>

// ======================== WIRING TABLE ==========================
//  ┌──────────────────┬─────────────────┬──────────────┐
//  │ Module           │ Nucleo Pin      │ Label        │
//  ├──────────────────┼─────────────────┼──────────────┤
//  │ LoRa NSS         │ D10             │ PB6          │
//  │ LoRa SCK         │ D13             │ PA5          │
//  │ LoRa MOSI        │ D11             │ PA7          │
//  │ LoRa MISO        │ D12             │ PA6          │
//  │ LoRa RST         │ D9              │ PC7          │
//  │ LoRa DIO0        │ D2              │ PA10         │
//  │ LoRa VCC         │ 3.3V            │              │
//  │ LoRa GND         │ GND             │              │
//  ├──────────────────┼─────────────────┼──────────────┤
//  │ MPU9250 SDA      │ D14             │ PB9          │
//  │ MPU9250 SCL      │ D15             │ PB8          │
//  │ MPU9250 VCC      │ 3.3V            │              │
//  │ MPU9250 GND      │ GND             │              │
//  ├──────────────────┼─────────────────┼──────────────┤
//  │ DHT11 DATA       │ D7              │ PA8          │
//  │ DHT11 VCC        │ 5V              │              │
//  │ DHT11 GND        │ GND             │              │
//  │ (4.7kΩ pull-up)  │ D7 → 5V         │              │
//  ├──────────────────┼─────────────────┼──────────────┤
//  │ LDR AO           │ A0              │ PA0          │
//  │ LDR VCC          │ 5V              │              │
//  │ LDR GND          │ GND             │              │
//  └──────────────────┴─────────────────┴──────────────┘
// ============================================================

// ---- Pin Definitions ----
#define LORA_NSS     D10
#define LORA_RST     D9
#define LORA_DIO0    D2
#define LORA_SCK     D13
#define LORA_MOSI    D11
#define LORA_MISO    D12

#define DHT_PIN      D7
#define DHT_TYPE     DHT11

#define LDR_PIN      A0

#define FREQ         433E6

// ---- Objects ----
MPU9250 mpu;
DHT dht(DHT_PIN, DHT_TYPE);

// ============================================================
//  Test Results
// ============================================================
bool loraOK = false;
bool mpuOK  = false;
bool dhtOK  = false;

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("\n========================================"));
  Serial.println(F("  STM32 Nucleo — Sensor & LoRa Detector"));
  Serial.println(F("========================================\n"));

  // ---- 1. Test DHT11 ----
  Serial.println(F("[1/4] DHT11..."));
  dht.begin();
  delay(1500);
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    Serial.println(F("  ✗ NOT FOUND — check DATA→D7, VCC→5V, pull-up 4.7kΩ"));
  } else {
    dhtOK = true;
    Serial.print(F("  ✓ DHT11 — Temp: "));
    Serial.print(t); Serial.print(F("°C  Hum: ")); Serial.print(h); Serial.println(F("%"));
  }

  // ---- 2. Test MPU9250 ----
  Serial.println(F("\n[2/4] MPU9250 (I2C)..."));
  Wire.begin();
  if (mpu.begin() < 0) {
    Serial.println(F("  ✗ NOT FOUND — check SDA→D14, SCL→D15, VCC→3.3V"));
  } else {
    mpuOK = true;
    mpu.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    mpu.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    mpu.readSensor();
    Serial.print(F("  ✓ MPU9250 — Accel: "));
    Serial.print(mpu.getAccelX_mss(), 1); Serial.print(F(", "));
    Serial.print(mpu.getAccelY_mss(), 1); Serial.print(F(", "));
    Serial.print(mpu.getAccelZ_mss(), 1); Serial.println(F(" m/s²"));
  }

  // ---- 3. Test LoRa SX1278 (SPI) ----
  Serial.println(F("\n[3/4] LoRa SX1278 (SPI)..."));
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(FREQ)) {
    Serial.println(F("  ✗ NOT FOUND — check NSS→D10, RST→D9, DIO0→D2"));
    Serial.println(F("              SCK→D13, MOSI→D11, MISO→D12, VCC→3.3V"));
  } else {
    loraOK = true;
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(8);
    LoRa.setTxPower(20);

    uint8_t ver = LoRa.readRegister(0x42);
    Serial.print(F("  ✓ LoRa SX1278 — Version: 0x"));
    Serial.println(ver, HEX);
    Serial.print(F("    RSSI: ")); Serial.print(LoRa.packetRssi()); Serial.println(F(" dBm"));
  }

  // ---- 4. Test LDR ----
  Serial.println(F("\n[4/4] LDR Module..."));
  pinMode(LDR_PIN, INPUT);
  int ldrVal = analogRead(LDR_PIN);
  Serial.print(F("  LDR analog value: "));
  Serial.print(ldrVal);
  if (ldrVal < 10) Serial.print(F(" (very dark — cover sensor?)"));
  else if (ldrVal > 1000) Serial.print(F(" (bright light)"));
  Serial.println();

  // ---- Summary ----
  Serial.println(F("\n========================================"));
  Serial.println(F("  DETECTION SUMMARY"));
  Serial.println(F("========================================"));
  Serial.print(F("  DHT11:    ")); Serial.println(dhtOK ? F("✓ DETECTED") : F("✗ MISSING"));
  Serial.print(F("  MPU9250:  ")); Serial.println(mpuOK ? F("✓ DETECTED") : F("✗ MISSING"));
  Serial.print(F("  LoRa:     ")); Serial.println(loraOK ? F("✓ DETECTED") : F("✗ MISSING"));
}

// ============================================================
//  Loop — Live sensor readings every 5 seconds
// ============================================================
void loop() {
  Serial.println(F("\n--- Live Readings ---"));

  if (dhtOK) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      Serial.print(F("  DHT11:   ")); Serial.print(t); Serial.print(F("°C  ")); Serial.print(h); Serial.println(F("%"));
    }
  }

  if (mpuOK) {
    mpu.readSensor();
    Serial.print(F("  MPU9250: "));
    Serial.print(mpu.getAccelX_mss(), 1); Serial.print(F(" "));
    Serial.print(mpu.getAccelY_mss(), 1); Serial.print(F(" "));
    Serial.print(mpu.getAccelZ_mss(), 1); Serial.print(F(" m/s²  "));
    Serial.print(mpu.getGyroX_rads(), 2); Serial.print(F(" "));
    Serial.print(mpu.getGyroY_rads(), 2); Serial.print(F(" "));
    Serial.print(mpu.getGyroZ_rads(), 2); Serial.println(F(" rad/s"));
  }

  if (loraOK) {
    Serial.print(F("  LoRa:    RSSI ")); Serial.print(LoRa.packetRssi()); Serial.print(F(" dBm"));
  }

  int ldrVal = analogRead(LDR_PIN);
  Serial.print(F("  LDR:     ")); Serial.println(ldrVal);

  delay(5000);
}
