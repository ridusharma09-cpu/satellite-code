// ============================================================
//  Arduino Uno — Test DHT11 + LoRa SX1278 (RA-02)
//  Upload this first to verify all wiring works.
//  Open Serial Monitor at 9600 baud to see results.
// ============================================================

#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>

// ---- Pin Connections ----
#define DHT_PIN      7       // DHT11 DATA pin → Uno D7
#define DHT_TYPE     DHT11

#define LORA_NSS     10      // LoRa NSS  → Uno D10
#define LORA_RST     9       // LoRa RST  → Uno D9
#define LORA_DIO0    2       // LoRa DIO0 → Uno D2
#define LORA_SCK     13      // LoRa SCK  → Uno D13
#define LORA_MOSI    11      // LoRa MOSI → Uno D11
#define LORA_MISO    12      // LoRa MISO → Uno D12

// ---- Radio Config ----
#define FREQ         433E6   // 433 MHz — change to 868E6 or 915E6 if needed

DHT dht(DHT_PIN, DHT_TYPE);

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  Serial.println(F("==================================="));
  Serial.println(F("  UNO + DHT11 + LoRa SX1278 TEST"));
  Serial.println(F("==================================="));

  // ---- Test 1: DHT11 ----
  Serial.println(F("\n[1/3] Testing DHT11..."));
  dht.begin();
  delay(2000);  // DHT needs 1-2s after power-up

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (isnan(t) || isnan(h)) {
    Serial.println(F("  FAIL — DHT11 not responding"));
    Serial.println(F("  Check: VCC→5V, GND→GND, DATA→D7, pull-up 4.7kΩ?"));
  } else {
    Serial.print(F("  OK — Temp: "));
    Serial.print(t);
    Serial.print(F("°C  Humidity: "));
    Serial.print(h);
    Serial.println(F("%"));
  }

  // ---- Test 2: LoRa SPI Communication ----
  Serial.println(F("\n[2/3] Testing LoRa SX1278 (SPI)..."));
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(FREQ)) {
    Serial.println(F("  FAIL — LoRa not found on SPI bus"));
    Serial.println(F("  Check: NSS→D10, RST→D9, DIO0→D2"));
    Serial.println(F("         SCK→D13, MOSI→D11, MISO→D12"));
    Serial.println(F("         VCC→3.3V (NOT 5V!), GND→GND"));
  } else {
    Serial.println(F("  OK — LoRa module detected"));
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(8);
    LoRa.setTxPower(20);
    Serial.println(F("  Radio configured: SF12, 125kHz, 4/8, 20dBm"));
  }

  // ---- Test 3: Read Version Register ----
  Serial.println(F("\n[3/3] Reading LoRa version register..."));
  uint8_t version = LoRa.readRegister(0x42);  // RegVersion
  Serial.print(F("  Version register (0x42) = 0x"));
  if (version < 0x10) Serial.print("0");
  Serial.print(version, HEX);

  if (version == 0x12) {
    Serial.println(F("  → SX1278 confirmed! ✓"));
  } else if (version == 0x22) {
    Serial.println(F("  → SX1276 detected"));
  } else {
    Serial.println(F("  → Unexpected value — SPI issue?"));
  }
}

// ============================================================
//  Loop — Send a test packet every 10 seconds
// ============================================================
void loop() {
  Serial.println(F("\n--- Sending test packet ---"));

  // Read DHT11
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();

  char payload[64];
  if (isnan(temp) || isnan(hum)) {
    snprintf(payload, sizeof(payload), "DHT11_ERROR");
  } else {
    snprintf(payload, sizeof(payload), "T%.1f_H%.0f", temp, hum);
  }

  // Send via LoRa
  Serial.print(F("Payload: "));
  Serial.println(payload);

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();

  Serial.println(F("Packet sent! Waiting 10s..."));

  // Blink built-in LED to show transmission
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);

  delay(10000);
}
