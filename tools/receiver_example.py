#!/usr/bin/env python3
"""Reference receiver for the ECMS-1 UART protocol."""

import argparse
import struct
from pathlib import Path

import serial

SYNC = b"\xAA\x55"
HEADER_AFTER_SYNC = struct.Struct("<BBB I Q I")
TELEMETRY = struct.Struct("<7fHBBI")

PKT_TELEMETRY = 0x01
PKT_JPEG = 0x02
PKT_STATUS = 0x03
PKT_ACK = 0x10
PKT_ERROR = 0x11


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def read_exact(ser: serial.Serial, count: int) -> bytes:
    data = bytearray()
    while len(data) < count:
        chunk = ser.read(count - len(data))
        if not chunk:
            raise TimeoutError("Serial timeout")
        data.extend(chunk)
    return bytes(data)


def seek_sync(ser: serial.Serial) -> None:
    prev = b""
    while True:
        cur = read_exact(ser, 1)
        if prev + cur == SYNC:
            return
        prev = cur


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="COM5, /dev/ttyUSB0, etc.")
    ap.add_argument("--baud", type=int, default=2_000_000)
    ap.add_argument("--save-frames", type=Path)
    args = ap.parse_args()

    if args.save_frames:
        args.save_frames.mkdir(parents=True, exist_ok=True)

    with serial.Serial(args.port, args.baud, timeout=2) as ser:
        print(f"Listening on {args.port} at {args.baud} baud")
        while True:
            try:
                seek_sync(ser)
                raw_header = read_exact(ser, HEADER_AFTER_SYNC.size)
                version, packet_type, flags, seq, timestamp_us, payload_len = HEADER_AFTER_SYNC.unpack(raw_header)

                if payload_len > 2_000_000:
                    print(f"Ignoring implausible payload length {payload_len}")
                    continue

                payload = read_exact(ser, payload_len)
                received_crc = struct.unpack("<H", read_exact(ser, 2))[0]
                calculated_crc = crc16_ccitt(raw_header + payload)

                if received_crc != calculated_crc:
                    print(f"CRC error seq={seq}: got {received_crc:04X}, expected {calculated_crc:04X}")
                    continue

                if packet_type == PKT_TELEMETRY and len(payload) == TELEMETRY.size:
                    ax, ay, az, gx, gy, gz, temp, distance, dvalid, ivalid, frame_id = TELEMETRY.unpack(payload)
                    distance_text = f"{distance} mm" if dvalid else "invalid"
                    print(
                        f"TEL seq={seq} frame={frame_id} "
                        f"a=({ax:+.2f},{ay:+.2f},{az:+.2f}) m/s² "
                        f"g=({gx:+.3f},{gy:+.3f},{gz:+.3f}) rad/s "
                        f"T={temp:.1f}°C distance={distance_text} imu={bool(ivalid)}"
                    )
                elif packet_type == PKT_JPEG:
                    if args.save_frames:
                        out = args.save_frames / f"frame_{seq:08d}_{timestamp_us}.jpg"
                        out.write_bytes(payload)
                        print(f"JPEG {len(payload)} bytes -> {out}")
                    else:
                        print(f"JPEG seq={seq} {len(payload)} bytes")
                elif packet_type in (PKT_STATUS, PKT_ACK, PKT_ERROR):
                    print(f"TEXT type=0x{packet_type:02X}: {payload.decode('utf-8', errors='replace')}")
                else:
                    print(f"Packet type=0x{packet_type:02X}, version={version}, flags={flags}, bytes={len(payload)}")

            except TimeoutError:
                continue
            except KeyboardInterrupt:
                break


if __name__ == "__main__":
    main()
