# Satellite Model - Complete Guide

## System Architecture

```
┌─────────────────┐     Serial (UART)     ┌──────────────────┐
│  Arduino Uno    │◄─────────────────────►│  STM32 Nucleo    │
│  (Main OBC)     │    TX→RX, RX→TX       │  (Radio Proc.)   │
│                 │    GND→GND             │                  │
│  Sensors:       │                        │  LoRa SX1278     │
│  • MPU9250 (I2C)│                        │  (via SPI)       │
│  • DHT22 (GPIO) │                        │                  │
│  • LDR (Analog) │                        │  → AX.25 framing │
└─────────────────┘                        │  → CCSDS framing │
                                           │  → 433 MHz TX/RX│
                                           └────────┬─────────┘
                                                    │
                                              Copper wire
                                              antenna (17.3 cm)
```

**Role split**: Uno handles sensor reading (uses its limited RAM better). Nucleo handles radio + packet framing (more RAM/Flash for protocol stacks). ESP32 can act as ground station or WiFi telemetry bridge.

---

## 1. Wiring Connections

### 1.1 Arduino Uno ←→ STM32 Nucleo (Serial UART)

| Arduino Uno | STM32 Nucleo-F401RE |
|-------------|---------------------|
| D0 (RX)     | D1 (TX) = PA10     |
| D1 (TX)     | D0 (RX) = PA9      |
| GND         | GND                |
| 5V          | Not connected      |

> **Note**: Nucleo is 5V-tolerant on Arduino header. Use common GND. Power separately.

### 1.2 MPU-9250/6500 (I2C) → Arduino Uno

| MPU9250 | Arduino Uno |
|---------|-------------|
| VCC     | 3.3V        |
| GND     | GND         |
| SCL     | A5 (SCL)    |
| SDA     | A4 (SDA)    |
| AD0     | GND (addr 0x68) or 3.3V (addr 0x69) |

### 1.3 DHT22 (digital) → Arduino Uno

| DHT22 | Arduino Uno |
|-------|-------------|
| VCC   | 5V          |
| GND   | GND         |
| DATA  | D7          |
| (NC)  | -           |

> Pull-up: 4.7kΩ between DATA and VCC.

### 1.4 LDR Module → Arduino Uno

| LDR Module | Arduino Uno |
|------------|-------------|
| VCC        | 5V          |
| GND        | GND         |
| AO (Analog)| A0          |
| DO (Digital)| (optional) |

### 1.5 LoRa SX1278 (RA-02) ←→ STM32 Nucleo (SPI)

| SX1278 | STM32 Nucleo-F401RE | Nucleo Pin |
|--------|---------------------|------------|
| VCC    | 3.3V                | -          |
| GND    | GND                 | -          |
| NSS    | D10                 | PB6        |
| SCK    | D13                 | PA5        |
| MOSI   | D11                 | PA7        |
| MISO   | D12                 | PA6        |
| RST    | D8                  | PA9        |
| DIO0   | D2                  | PA10       |

> SX1278 is **3.3V only** — do NOT connect to 5V. Nucleo's Arduino header operates at 3.3V, so direct connection is safe.

### 1.6 ESP32 (optional) → Ground Relay or WiFi Bridge

Can be used as:
- **Ground station receiver**: ESP32 + second LoRa reads transmissions and forwards via WiFi/Serial to your PC running OpenC3
- **Onboard WiFi downlink**: ESP32 connected to Nucleo via UART, relays telemetry over WiFi as a backup

---

## 2. Copper Wire Antenna (433 MHz)

### Quarter-Wave Monopole (simplest, effective)

```
Copper wire (12-18 AWG)
         │
         │  17.3 cm
         │
         └───┬───
             │ Connect to SX1278 ANT pin
             │
         ┌───┴───┐
         │ Ground │  (ground plane: copper clad board
         │ Plane  │   or sheet metal ~30cm diameter)
         └───────┘
```

**Specs**:
- **Length**: λ/4 = (3×10⁸ / 433×10⁶) / 4 = **0.173 m = 17.3 cm**
- **Wire**: Solid copper wire, 1.5mm² (AWG 16) or similar
- **Ground plane**: Copper clad board ≥30×30 cm or sheet metal

**Steps**:
1. Cut copper wire to **17.3 cm** (add 2mm for soldering margin)
2. Solder one end to SX1278 module's **ANT** pin
3. Solder a ground wire from SX1278 GND to the ground plane
4. Keep the antenna vertical (perpendicular to ground plane)
5. For a dipole (no ground plane needed): cut **two** 16.5 cm wires, one to ANT, one to GND, extend in opposite directions

**Tuning**: If you have an SWR meter or NanoVNA, trim 1mm at a time for best SWR at 433 MHz.

---

## 3. AX.25 & CCSDS Packet Implementation

### Physical Reality: LoRa is not AX.25/CCSDS-native

LoRa SX1278 uses **proprietary spread-spectrum modulation**. It does NOT support direct AX.25 or CCSDS at the physical layer. You have two approaches:

### Approach A: AX.25-over-LoRa (Recommended for simplicity)

Frame AX.25 packets **inside** LoRa payloads:

```
┌─────────────────────────────────────────────────────┐
│ LoRa Packet (max 256 bytes)                         │
│ ┌─────────────────────────────────────────────────┐ │
│ │ AX.25 Frame                                     │ │
│ │ ┌──────┬──────┬──────┬────────┬──────┬────────┐ │ │
│ │ │Flag  │Dest  │Src   │Control │PID   │Info    │ │ │
│ │ │0x7E  │Addr  │Addr  │Field   │0xF0  │Payload │ │ │
│ │ └──────┴──────┴──────┴────────┴──────┴────────┘ │ │
│ │ CRC is handled by LoRa hardware                  │ │
│ └─────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────┘
```

**Library**: Use `sandeepmistry/LoRa` for the radio, wrap AX.25 framing in software.

### Approach B: CCSDS-over-LoRa (for space protocol learning)

```
┌─────────────────────────────────────────────────────┐
│ LoRa Packet                                         │
│ ┌─────────────────────────────────────────────────┐ │
│ │ CCSDS Transfer Frame                            │ │
│ │ ┌──────┬──────┬──────┬──────┬──────┬──────────┐ │ │
│ │ │Vers# │Type  │SCID  │VCID  │Frame │Data Field│ │ │
│ │ │ 00   │ 0    │0x001 │0x00  │#     │(payload) │ │ │
│ │ └──────┴──────┴──────┴──────┴──────┴──────────┘ │ │
│ └─────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────┘
```

### Packet Format: What to transmit

Build a **hybrid telemetry packet** combining best of both:

```
Byte 0:     Preamble / Sync word (0xBC)
Byte 1:     Protocol ID (0x01 = AX.25, 0x02 = CCSDS)
Byte 2-3:   Satellite ID (0x0001)
Byte 4:     Packet type (0x01=HK telemetry, 0x02=ADCS, 0x03=event)
Byte 5-6:   Packet counter (increments each TX)
Byte 7-8:   Timestamp (seconds since boot, 16-bit)
Byte 9-10:  Temperature (°C × 100, int16)
Byte 11-12: Humidity (% × 100, int16)
Byte 13-16: Pressure (Pa, float32) — if sensor present
Byte 17-20: MPU9250 Accel X (int16 × 100)
Byte 21-22: MPU9250 Accel Y
Byte 23-24: MPU9250 Accel Z
Byte 25-26: MPU9250 Gyro X
Byte 27-28: MPU9250 Gyro Y
Byte 29-30: MPU9250 Gyro Z
Byte 31-32: MPU9250 Mag X
Byte 33-34: MPU9250 Mag Y
Byte 35-36: MPU9250 Mag Z
Byte 37:    LDR value (0-255)
Byte 38:    Battery voltage (× 10, uint8)
Byte 39:    Status flags (bitfield: solar panel, errors, etc.)
Byte 40-41: CRC-16 (CCITT)
```

**Total: 42 bytes per packet** — well within LoRa's 256-byte limit.

### Arduino Code Outline (Nucleo side, TX)

```cpp
#include <SPI.h>
#include <LoRa.h>

void initRadio() {
  LoRa.setPins(NSS, RST, DIO0);
  if (!LoRa.begin(433E6)) {  // 433 MHz
    while (1);
  }
  LoRa.setSpreadingFactor(12);
  LoRa.setCodingRate4(8);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setTxPower(20);  // max
}

void sendPacket(uint8_t* data, uint8_t len) {
  LoRa.beginPacket();
  LoRa.write(data, len);
  LoRa.endPacket();
}
```

---

## 4. OpenC3 COSMOS Telemetry Setup

### What is OpenC3 COSMOS

OpenC3 COSMOS is an open-source ground station/telemetry software. It runs on a PC and receives telemetry from your satellite model for display and logging.

### OpenC3 Installation

```bash
# Windows (PowerShell)
docker pull registry.gitlab.com/openc3/openc3-operator:latest

# Run COSMOS
docker run -d -p 2900:2900 -p 2901:2901 -p 2902:2902 -p 2903:2903 -p 2904:2904 -p 2905:2905 -p 8080:8080 -p 8081:8081 --name openc3-cosmos registry.gitlab.com/openc3/openc3-operator:latest
```

Or use the COSMOS Installer from https://openc3.com/downloads.

### Target Definition: Satellite

Create `openc3/targets/SATELLITE/cmd_tlm/satellite_cmds.txt`:

```
COMMAND SATELLITE SET_MODE BIG_ENDIAN "Set Satellite Mode"
  PARAMETER MODE 0 1 UINT 0 5 "0=Safe,1=Idle,2=Science,3=TX"
  PARAMETER MODE_STRING 0 0 STRING "Mode string"

COMMAND SATELLITE TX_BEACON BIG_ENDIAN "Transmit beacon"
  PARAMETER ENABLE 0 8 DERIVED "Enable"
```

Create `openc3/targets/SATELLITE/cmd_tlm/satellite_tlm.txt`:

```
TELEMETRY SATELLITE HK_PACKET BIG_ENDIAN "Housekeeping Telemetry"
  APPEND_ID_ITEM PACKET_ID 8 UINT 0xBC "Packet sync byte"
  APPEND_ITEM PROTOCOL_ID 8 UINT "1=AX.25, 2=CCSDS"
  APPEND_ITEM SAT_ID 16 UINT "Satellite ID"
  APPEND_ITEM PACKET_TYPE 8 UINT "1=HK"
  APPEND_ITEM PACKET_COUNTER 16 UINT "Packet sequence"
  APPEND_ITEM TIMESTAMP 16 UINT "Seconds since boot"
  APPEND_ITEM TEMPERATURE 16 INT "<<temperature>>"
    FORMAT_STRING "%0.2f C"
    UNITS Celsius C
  APPEND_ITEM HUMIDITY 16 INT "<<humidity>>"
    FORMAT_STRING "%0.2f %%"
  APPEND_ITEM ACCEL_X 16 INT "m/s²"
  APPEND_ITEM ACCEL_Y 16 INT
  APPEND_ITEM ACCEL_Z 16 INT
  APPEND_ITEM GYRO_X 16 INT "°/s"
  APPEND_ITEM GYRO_Y 16 INT
  APPEND_ITEM GYRO_Z 16 INT
  APPEND_ITEM MAG_X 16 INT "µT"
  APPEND_ITEM MAG_Y 16 INT
  APPEND_ITEM MAG_Z 16 INT
  APPEND_ITEM LDR 8 UINT "Light level 0-255"
  APPEND_ITEM BATTERY 8 UINT "Voltage × 10"
  APPEND_ITEM STATUS_FLAGS 8 UINT "Status bitfield"
  APPEND_ITEM CRC 16 UINT "CRC-16/CCITT"
    HIDDEN true
```

### Receiving Telemetry

**Option 1**: Ground ESP32 + LoRa receiver → Serial → OpenC3

```
[Satellite] ──LoRa──► [ESP32 + LoRa] ──USB Serial──► [PC / OpenC3]
```

Configure OpenC3 interface: OpenC3 → Interfaces → Add Serial Interface → COM port, 115200 baud.

**Option 2**: Direct radio receiver → sound card → OpenC3 (with a proper SDR like RTL-SDR)

### Telemetry Screens

Create `openc3/targets/SATELLITE/screens/satellite.txt`:

```
SCREEN "Satellite Telemetry"

VERTICALBOX
  TITLE "Satellite Status"
  VERTICAL
    LABELVALUE SATELLITE HK_PACKET TEMPERATURE 5
    LABELVALUE SATELLITE HK_PACKET HUMIDITY 5
    LABELVALUE SATELLITE HK_PACKET LDR 5
    LABELVALUE SATELLITE HK_PACKET BATTERY 5
  END
  TITLE "Attitude"
  VERTICAL
    LABELVALUE SATELLITE HK_PACKET ACCEL_X 5
    LABELVALUE SATELLITE HK_PACKET ACCEL_Y 5
    LABELVALUE SATELLITE HK_PACKET ACCEL_Z 5
    LABELVALUE SATELLITE HK_PACKET GYRO_X 5
  END
END
```

---

## 5. Power Budget Quick Reference

| Component | Voltage | Current |
|-----------|---------|---------|
| Arduino Uno | 5V / 7-12V input | ~50 mA |
| STM32 Nucleo | 3.3V / USB | ~80 mA |
| ESP32 | 3.3V | ~80 mA (WiFi: up to 300 mA) |
| LoRa SX1278 TX | 3.3V | ~120 mA @ 20 dBm |
| LoRa SX1278 RX | 3.3V | ~12 mA |
| MPU9250 | 3.3V | ~3.5 mA |
| DHT22 | 3.3-5V | ~1.5 mA |
| LDR module | 3.3-5V | ~0.5 mA |

> Total TX peak: ~350 mA — a 2000 mAh LiPo lasts ~5 hours continuous.
> For longer operation, sleep between transmissions (duty cycle).

---

## 6. Transmission Strategy

For a satellite model, use a **beacon** approach:

1. **Boot**: Initialize all sensors, wait 30s for stabilization
2. **Read**: Collect sensor data → create HK packet
3. **Frame**: Wrap in AX.25 or custom header
4. **Transmit**: LoRa TX at 433 MHz (takes ~500ms at SF12)
5. **Sleep**: Go to low-power sleep for 5-30 seconds
6. **Repeat**: Loop

```
TX ─[5s sleep]─ TX ─[5s sleep]─ TX ─[5s sleep]─ TX ─►
```

For OpenC3 ground station, set the interface to listen continuously and decode packets as they arrive.
