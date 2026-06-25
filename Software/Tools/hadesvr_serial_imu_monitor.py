#!/usr/bin/env python3
"""Monitor HadesVR UART IMU packets from a USB serial device."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import Optional, Tuple

try:
    import serial
except ImportError:
    print("pyserial is required. Install it with: python -m pip install pyserial", file=sys.stderr)
    raise SystemExit(2)


MAGIC = 0xAA
HMD_RAW_PACKET_ID = 3
HMD_TIMESTAMPED_IMU_PACKET_ID = 4
TIMESTAMPED_IMU_FORMAT = "<BBBHIhhhhhhhhhhHHB30s"


def read_exact(port: serial.Serial, count: int) -> Optional[bytes]:
    data = port.read(count)
    if len(data) != count:
        return None
    return data


def read_frame(port: serial.Serial) -> Optional[Tuple[int, bytes]]:
    while True:
        byte = port.read(1)
        if not byte:
            return None
        if byte[0] == MAGIC:
            break

    seq_bytes = read_exact(port, 1)
    size_bytes = read_exact(port, 1)
    if seq_bytes is None or size_bytes is None:
        return None

    payload = read_exact(port, size_bytes[0])
    if payload is None:
        return None

    return seq_bytes[0], payload


def scaled_imu(acc_x: int, acc_y: int, acc_z: int, gyro_x: int, gyro_y: int, gyro_z: int) -> str:
    return (
        f"acc_g=({acc_x / 2048.0:+.3f}, {acc_y / 2048.0:+.3f}, {acc_z / 2048.0:+.3f}) "
        f"gyro_dps=({gyro_x / 16.0:+.2f}, {gyro_y / 16.0:+.2f}, {gyro_z / 16.0:+.2f})"
    )


def describe_payload(payload: bytes) -> Optional[str]:
    if len(payload) < 2:
        return None

    packet_id = payload[1]
    if packet_id == HMD_TIMESTAMPED_IMU_PACKET_ID and len(payload) == 64:
        (
            report_id,
            packet_id,
            version,
            imu_seq,
            timestamp_us,
            acc_x,
            acc_y,
            acc_z,
            gyro_x,
            gyro_y,
            gyro_z,
            mag_x,
            mag_y,
            mag_z,
            temp_centi_c,
            hmd_data,
            sample_period_us,
            imu_status,
            _reserved,
        ) = struct.unpack(TIMESTAMPED_IMU_FORMAT, payload)
        sample_hz = 1_000_000.0 / sample_period_us if sample_period_us else 0.0
        return (
            f"report={report_id} packet={packet_id} v={version} imu_seq={imu_seq} "
            f"t={timestamp_us}us target={sample_hz:.1f}Hz status=0x{imu_status:02x} "
            f"{scaled_imu(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)} "
            f"mag=({mag_x}, {mag_y}, {mag_z}) temp={temp_centi_c / 100.0:.2f}C data=0x{hmd_data:04x}"
        )

    if packet_id == HMD_RAW_PACKET_ID and len(payload) == 64:
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, hmd_data = struct.unpack_from(
            "<hhhhhhhhhH", payload, 2
        )
        return (
            f"report={payload[0]} packet={packet_id} "
            f"{scaled_imu(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)} "
            f"mag=({mag_x}, {mag_y}, {mag_z}) data=0x{hmd_data:04x}"
        )

    return f"report={payload[0]} packet={packet_id} bytes={len(payload)}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--raw", action="store_true", help="print every decoded frame")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=1) as port:
        print(f"Listening on {args.port} at {args.baud} baud. Press Ctrl+C to stop.")
        frames = 0
        last_print = time.monotonic()
        last_description = "waiting for packets"

        while True:
            frame = read_frame(port)
            if frame is None:
                continue

            _frame_seq, payload = frame
            description = describe_payload(payload)
            if description is None:
                continue

            frames += 1
            last_description = description

            if args.raw:
                print(description)
                continue

            now = time.monotonic()
            if now - last_print >= 1.0:
                print(f"{frames / (now - last_print):6.1f} fps | {last_description}")
                frames = 0
                last_print = now


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print()
