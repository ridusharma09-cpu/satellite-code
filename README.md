# Satellite Model — Firmware

Multi-microcontroller satellite model with sensor telemetry, AX.25/CCSDS packet framing, and LoRa SX1278 433 MHz wireless transmission.

## Hardware

| Component | Role |
|-----------|------|
| Arduino Uno | Main flight computer — reads MPU9250, DHT11, LDR |
| STM32 Nucleo-F401RE | Radio processor — AX.25 framing + LoRa TX/RX |
| ESP32 | Ground station receiver — decodes & displays telemetry |
| LoRa SX1278 (RA-02) | 433 MHz radio module (SPI interface) |
| MPU-9250/6500 | 9-DOF IMU (I2C) |
| DHT11/DHT22 | Temperature & humidity sensor |
| LDR Module | Light sensor (analog) |

## Project Structure

```
SatelliteCode/
├── ArduinoUno/              # Main OBC — sensor reading & packetization
│   ├── satellite_obc.ino    # Reads MPU9250, DHT11, LDR → sends via Serial
│   └── test_connections.ino # Quick wiring verification sketch
├── STM32_Nucleo/            # Radio processor — AX.25/CCSDS framing
│   ├── satellite_radio.ino  # Receives from Uno, frames, transmits via LoRa
│   └── test_sensors.ino     # Detect & test all connected modules
├── ESP32_TX/                # Standalone ESP32 transmitter
│   └── esp32_tx.ino         # DHT11 → AX.25 → LoRa (SPI) → Antenna
├── ESP32_RX/                # ESP32 ground receiver
│   └── esp32_rx.ino         # LoRa → AX.25 decode → Serial display
├── ESP32_Ground/            # ESP32 ground station (full telemetry)
│   └── ground_station.ino   # CSV output + formatted telemetry display
└── ESP32_LoRa_AX25/         # Combined TX+RX (recommended starting point)
    ├── esp32_tx_ax25.ino    # TX with configurable frequency + auto antenna calc
    └── esp32_rx_ax25.ino    # RX with AX.25 frame decoder + CRC validation
```

## Wiring

### LoRa SX1278 → Any Board (SPI)

| LoRa Pin | Arduino Uno | STM32 Nucleo | ESP32 |
|----------|-------------|--------------|-------|
| NSS (CS) | D10 | D10 (PB6) | GPIO5 |
| SCK | D13 | D13 (PA5) | GPIO18 |
| MOSI | D11 | D11 (PA7) | GPIO23 |
| MISO | D12 | D12 (PA6) | GPIO19 |
| RST | D9 | D9 (PC7) | GPIO14 |
| DIO0 | D2 | D2 (PA10) | GPIO2 |
| VCC | 3.3V | 3.3V | 3.3V |
| GND | GND | GND | GND |

### Sensors → Arduino Uno

| Sensor | Pin |
|--------|-----|
| MPU9250 SDA | A4 (I2C) |
| MPU9250 SCL | A5 (I2C) |
| DHT11 DATA | D7 (+ 4.7kΩ pull-up) |
| LDR AO | A0 |

## Telemetry Packet Format

All telemetry is packed into a 33-byte binary frame:

```
Byte  0:     Sync (0xBC)
Byte  1:     Version (0x01)
Byte  2-3:   Packet counter (uint16)
Byte  4-5:   Timestamp (uint16, seconds)
Byte  6-7:   Temperature (int16, °C × 100)
Byte  8-9:   Humidity (uint16, % × 100)
Byte 10-11:  Accel X (int16, mg)
Byte 12-13:  Accel Y (int16, mg)
Byte 14-15:  Accel Z (int16, mg)
Byte 16-17:  Gyro X (int16, mdps)
Byte 18-19:  Gyro Y (int16, mdps)
Byte 20-21:  Gyro Z (int16, mdps)
Byte 22-23:  Mag X (int16, 0.1 µT)
Byte 24-25:  Mag Y (int16, 0.1 µT)
Byte 26-27:  Mag Z (int16, 0.1 µT)
Byte 28:     LDR (uint8, 0-255)
Byte 29:     Battery (uint8, V × 10)
Byte 30:     Status flags (uint8)
Byte 31-32:  CRC-16/CCITT (uint16)
```

## Building & Uploading

All projects use [PlatformIO](https://platformio.org/).

```bash
pip install platformio

# Upload to Arduino Uno
cd ArduinoUno && platformio run --target upload

# Upload to STM32 Nucleo
cd STM32_Nucleo && platformio run --target upload

# Upload to ESP32
cd ESP32_LoRa_AX25 && platformio run --target upload
```

## Antenna

Quarter-wave copper wire antenna for 433 MHz:

```
Frequency → Length = (300 / 433) / 4 × 0.95 = 16.44 cm
```

Cut a solid copper wire to 16.5 cm and solder to the LoRa module's ANT pin. For other frequencies, change `FREQ_MHZ` in the code — the antenna length is calculated automatically and printed at startup.

## OpenC3 COSMOS Telemetry

The ESP32 ground station outputs CSV-formatted telemetry that can be ingested by [OpenC3 COSMOS](https://openc3.com/). A full target definition for COSMOS is included in `Satellite-Guide.md`.

## License

MIT
