# Wiring Guide

## External sensors

Both sensors use the same 400 kHz I²C bus.

```text
AI-Thinker ESP32-CAM

GPIO13 (SDA) ─────┬── MPU-6050 SDA
                  └── VL53L0X SDA

GPIO14 (SCL) ─────┬── MPU-6050 SCL
                  └── VL53L0X SCL

3.3V ─────────────┬── MPU-6050 VCC
                  └── VL53L0X VCC*

GND ──────────────┬── MPU-6050 GND
                  ├── VL53L0X GND
                  └── Host / USB-UART GND
```

`*` Check the specific VL53L0X breakout board for regulator and level-shifter support.

## Host UART

```text
ESP32-CAM GPIO1 / U0T  ─────> Host RX
ESP32-CAM GPIO3 / U0R  <───── Host TX
ESP32-CAM GND          ────── Host GND
```

Use 3.3 V TTL logic.

## Programming mode

```text
GPIO0 ── GND   only while entering flash mode
```

After upload, remove the GPIO0-GND connection and reset the board.

## microSD

The microSD interface is intentionally unused because GPIO13 and GPIO14 are assigned to the external sensor I²C bus.
