# ECMS-1 Binary Protocol

`ECMS-1` transports JPEG images, sensor telemetry and firmware responses over one UART stream.

## Packet frame

```text
Offset  Size  Field
0       2     Sync: AA 55
2       1     Protocol version
3       1     Packet type
4       1     Flags
5       4     Sequence number (uint32 LE)
9       8     Timestamp in microseconds (uint64 LE)
17      4     Payload length (uint32 LE)
21      N     Payload
21+N    2     CRC16-CCITT (uint16 LE)
```

The current protocol version is `1`.

## CRC16

```text
Polynomial: 0x1021
Init:       0xFFFF
Input:      Version .. Payload
Sync bytes: excluded
```

## Packet types

```text
0x01 TELEMETRY
0x02 JPEG_FRAME
0x03 STATUS
0x10 COMMAND_ACK
0x11 ERROR
```

## TELEMETRY payload

Packed layout, little-endian:

```text
float    ax_mps2
float    ay_mps2
float    az_mps2
float    gx_rads
float    gy_rads
float    gz_rads
float    imu_temp_c
uint16   distance_mm
uint8    distance_valid
uint8    imu_valid
uint32   last_frame_id
```

Total payload size: 36 bytes on the ESP32 implementation.

`distance_mm = 0xFFFF` indicates unavailable / invalid distance.

## JPEG_FRAME payload

The payload is one complete JPEG file from the OV2640. After CRC validation the bytes can be written directly to a `.jpg` file or passed to a JPEG decoder.

## STATUS / ACK / ERROR

These packet payloads are UTF-8 / ASCII text.

Typical status:

```text
FW=ESP32_CAM_MULTI_SENSOR_STREAM_1.0;
BOARD=AI_THINKER_ESP32_CAM;
CAM=OV2640;
MPU6050=OK;
VL53L0X=OK;
PSRAM=YES;
UART=2000000;
PROTO=ECMS-1;
```

## Command channel

Host-to-device commands share the same UART but are text lines beginning with `#`.

```text
#STATUS
#START
#STOP
#FPS 1..30
#QUALITY 6..40
#FRAMESIZE QQVGA|QVGA|VGA
#TELEMETRY 1..200
#CALIBRATE_GYRO
#SAVE
#DEFAULTS
#REBOOT
```
