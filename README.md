# ESP32-CAM Multi-Sensor Stream

Firmware for an **AI-Thinker ESP32-CAM** that combines an **OV2640 camera**, **MPU-6050 IMU** and **VL53L0X time-of-flight distance sensor** into one synchronized streaming module.

The ESP32 captures JPEG frames and streams them together with acceleration, angular velocity, IMU temperature, distance measurements, timestamps and frame identifiers over a single high-speed UART connection. A compact binary protocol with CRC16 protection is used for outbound data, while a simple ASCII command channel allows runtime configuration from a host computer.

> This repository contains embedded firmware and host-side protocol examples. It is a development / prototyping project, not a certified safety or metrology system.

## Features

- AI-Thinker ESP32-CAM support
- OV2640 JPEG capture
- MPU-6050 accelerometer + gyroscope + temperature
- VL53L0X ToF distance sensing
- Shared external I²C bus on GPIO13 / GPIO14
- 2,000,000 baud UART transport
- Binary framed protocol (`ECMS-1`)
- CRC16-CCITT packet validation
- 64-bit microsecond timestamps
- packet sequence counter
- JPEG frame packets
- synchronized telemetry packets
- runtime start/stop control
- runtime FPS control
- runtime JPEG quality control
- QQVGA / QVGA / VGA selection
- telemetry rate from 1 to 200 Hz
- MPU-6050 gyro calibration
- persistent configuration and gyro bias in NVS
- status / ACK / error packets
- PSRAM-aware camera buffering

## Hardware

| Component | Function |
|---|---|
| AI-Thinker ESP32-CAM | controller and streaming interface |
| OV2640 | JPEG camera |
| MPU-6050 / GY-521 | 3-axis accelerometer + 3-axis gyroscope |
| VL53L0X | time-of-flight distance sensor |
| 3.3 V USB-UART adapter | host / programming interface |

## Wiring

### MPU-6050 and VL53L0X

The two sensors share the same external I²C bus. The microSD interface is not used.

| ESP32-CAM | MPU-6050 | VL53L0X | Function |
|---|---|---|---|
| GPIO13 | SDA | SDA | I²C data |
| GPIO14 | SCL | SCL | I²C clock |
| 3.3 V | VCC | VCC* | sensor power |
| GND | GND | GND | common ground |

`*` Verify the exact VL53L0X breakout board. The project assumes 3.3 V compatible logic.

Default I²C addresses:

- MPU-6050: `0x68`
- VL53L0X: `0x29`

### Host UART

| ESP32-CAM | Host | Function |
|---|---|---|
| GPIO1 / U0T | RX | ESP32 → host data |
| GPIO3 / U0R | TX | host → ESP32 commands |
| GND | GND | common ground |

UART settings: **2,000,000 baud, 8-N-1, 3.3 V TTL**.

Do not connect directly to classic ±12 V RS-232 hardware.

## Camera pin map

The OV2640 camera pins are fixed by the AI-Thinker ESP32-CAM PCB:

| Signal | GPIO |
|---|---:|
| PWDN | 32 |
| RESET | -1 |
| XCLK | 0 |
| SIOD | 26 |
| SIOC | 27 |
| Y9 | 35 |
| Y8 | 34 |
| Y7 | 39 |
| Y6 | 36 |
| Y5 | 21 |
| Y4 | 19 |
| Y3 | 18 |
| Y2 | 5 |
| VSYNC | 25 |
| HREF | 23 |
| PCLK | 22 |

## Default operating profile

- resolution: **QVGA 320×240**
- camera FPS: **10**
- JPEG quality: **12**
- telemetry: **100 Hz**
- MPU accelerometer: **±8 g**
- MPU gyroscope: **±500 °/s**
- MPU DLPF: **44 Hz**
- I²C: **400 kHz**

## Protocol overview

The outbound stream uses protocol **ECMS-1**.

Each packet contains:

```text
SYNC             2 bytes  AA 55
Protocol version 1 byte
Packet type      1 byte
Flags            1 byte
Sequence         4 bytes
Timestamp        8 bytes  microseconds
Payload length   4 bytes
Payload          N bytes
CRC16-CCITT      2 bytes
```

All multi-byte integer fields are little-endian.

CRC configuration:

- polynomial: `0x1021`
- initial value: `0xFFFF`
- CRC covers fields from `Protocol Version` through the end of `Payload`
- sync bytes `AA 55` are not included

### Packet types

| Code | Type |
|---:|---|
| `0x01` | telemetry |
| `0x02` | JPEG frame |
| `0x03` | status |
| `0x10` | command ACK |
| `0x11` | error |

See [docs/PROTOCOL.md](docs/PROTOCOL.md) for the full binary layout.

## Telemetry payload

```c
float ax_mps2;
float ay_mps2;
float az_mps2;
float gx_rads;
float gy_rads;
float gz_rads;
float imu_temp_c;
uint16_t distance_mm;
uint8_t distance_valid;
uint8_t imu_valid;
uint32_t last_frame_id;
```

Units:

- acceleration: `m/s²`
- angular velocity: `rad/s`
- distance: `mm`
- temperature: `°C`

## Runtime commands

Commands are ASCII lines beginning with `#` and ending with newline:

```text
#STATUS
#START
#STOP
#FPS 10
#QUALITY 12
#FRAMESIZE QQVGA
#FRAMESIZE QVGA
#FRAMESIZE VGA
#TELEMETRY 100
#CALIBRATE_GYRO
#SAVE
#DEFAULTS
#REBOOT
```

## Flashing with Arduino IDE

Install:

1. ESP32 board package by Espressif Systems
2. Adafruit MPU6050
3. Adafruit Unified Sensor
4. VL53L0X by Pololu

Then:

1. Select **AI Thinker ESP32-CAM**.
2. USB-UART TX → GPIO3 / U0R.
3. USB-UART RX → GPIO1 / U0T.
4. Connect GND.
5. Power the ESP32-CAM from a stable 5 V source.
6. Connect GPIO0 to GND only while entering flash mode.
7. Reset / power-cycle and upload.
8. Disconnect GPIO0 from GND.
9. Reset again to run the firmware.

The boot ROM can emit startup text at its own baud rate before the application switches UART0 to 2 Mbaud. A host parser should ignore input until it finds the `AA 55` packet sync sequence.

## Gyroscope calibration

After rigidly mounting the camera and IMU:

1. keep the module completely still;
2. send `#CALIBRATE_GYRO`;
3. wait roughly 2 seconds;
4. the firmware averages 500 gyro samples;
5. X/Y/Z gyro bias values are stored in NVS.

This compensates static gyro offset. It does not replace accelerometer calibration or camera-to-IMU extrinsic calibration.

## Throughput note

A 2 Mbaud UART link has limited throughput compared with USB or Ethernet. QVGA JPEG at moderate quality is suitable for prototyping, but the achievable frame rate depends strongly on image complexity and JPEG size.

For higher video rates, a practical architecture is:

- UART: IMU + ToF + commands + synchronization
- Wi-Fi / USB / Ethernet: JPEG or MJPEG video

The same packet framing can still be retained for telemetry.

## Repository structure

```text
ESP32-CAM-Multi-Sensor-Stream/
├── ESP32_CAM_Multi_Sensor_Stream/
│   └── ESP32_CAM_Multi_Sensor_Stream.ino
├── docs/
│   ├── PROTOCOL.md
│   └── WIRING.md
├── tools/
│   ├── receiver_example.py
│   └── requirements.txt
├── THIRD_PARTY_LICENSES.md
├── LICENSE
└── README.md
```

## Host receiver example

`tools/receiver_example.py` is a small reference parser for the `ECMS-1` stream. It can read the binary UART stream, verify CRC, print telemetry and optionally save received JPEG frames.

Example:

```bash
pip install -r tools/requirements.txt
python tools/receiver_example.py COM5 --baud 2000000 --save-frames frames
```

Linux example:

```bash
python tools/receiver_example.py /dev/ttyUSB0 --baud 2000000
```

## Safety and electrical notes

- ESP32 GPIO is 3.3 V logic.
- Use a stable 5 V supply for the ESP32-CAM.
- Do not power motors or servos through the ESP32-CAM regulator.
- All connected devices must share a common ground.
- Confirm breakout-board voltage compatibility before wiring.
- Distance data from VL53L0X and camera imagery should not be treated as a sole safety channel for machinery or robotics.

## License

Original firmware and repository documentation are released under the **MIT License**. Third-party libraries retain their own licenses. See [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
