from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from threading import Event, Thread
from typing import Any, Dict, Optional, Tuple
import json
import struct
import time


MAGIC = 0xAA
HMD_RAW_PACKET_ID = 3
HMD_TIMESTAMPED_IMU_PACKET_ID = 4
TIMESTAMPED_IMU_FORMAT = "<BBBHIhhhhhhhhhhHHB30s"


@dataclass
class ImuSample:
    host_receive_timestamp_ns: int
    wall_time_utc: str
    uart_frame_sequence: int
    packet_id: int
    version: int
    sequence: int
    device_timestamp_us: Optional[int]
    acc_x: int
    acc_y: int
    acc_z: int
    gyro_x: int
    gyro_y: int
    gyro_z: int
    mag_x: int
    mag_y: int
    mag_z: int
    temp_centi_c: Optional[int]
    hmd_data: int
    sample_period_us: Optional[int]
    imu_status: Optional[int]

    def to_json(self) -> Dict[str, Any]:
        return asdict(self)


def wall_time_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_exact(port: Any, count: int) -> Optional[bytes]:
    data = port.read(count)
    if len(data) != count:
        return None
    return data


def read_frame(port: Any) -> Optional[Tuple[int, bytes, int]]:
    while True:
        byte = port.read(1)
        if not byte:
            return None
        if byte[0] == MAGIC:
            break

    sequence = read_exact(port, 1)
    size = read_exact(port, 1)
    if sequence is None or size is None:
        return None

    payload = read_exact(port, size[0])
    if payload is None:
        return None

    return sequence[0], payload, time.monotonic_ns()


def decode_imu_sample(frame_sequence: int, payload: bytes, host_timestamp_ns: int) -> Optional[ImuSample]:
    if len(payload) < 2:
        return None

    packet_id = payload[1]
    if packet_id == HMD_TIMESTAMPED_IMU_PACKET_ID and len(payload) == 64:
        (
            _report_id,
            _packet_id,
            version,
            sequence,
            device_timestamp_us,
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
        return ImuSample(
            host_receive_timestamp_ns=host_timestamp_ns,
            wall_time_utc=wall_time_iso(),
            uart_frame_sequence=frame_sequence,
            packet_id=packet_id,
            version=version,
            sequence=sequence,
            device_timestamp_us=device_timestamp_us,
            acc_x=acc_x,
            acc_y=acc_y,
            acc_z=acc_z,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            mag_x=mag_x,
            mag_y=mag_y,
            mag_z=mag_z,
            temp_centi_c=temp_centi_c,
            hmd_data=hmd_data,
            sample_period_us=sample_period_us,
            imu_status=imu_status,
        )

    if packet_id == HMD_RAW_PACKET_ID and len(payload) == 64:
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, hmd_data = struct.unpack_from(
            "<hhhhhhhhhH", payload, 2
        )
        return ImuSample(
            host_receive_timestamp_ns=host_timestamp_ns,
            wall_time_utc=wall_time_iso(),
            uart_frame_sequence=frame_sequence,
            packet_id=packet_id,
            version=0,
            sequence=frame_sequence,
            device_timestamp_us=None,
            acc_x=acc_x,
            acc_y=acc_y,
            acc_z=acc_z,
            gyro_x=gyro_x,
            gyro_y=gyro_y,
            gyro_z=gyro_z,
            mag_x=mag_x,
            mag_y=mag_y,
            mag_z=mag_z,
            temp_centi_c=None,
            hmd_data=hmd_data,
            sample_period_us=None,
            imu_status=None,
        )

    return None


class SerialImuRecorder:
    def __init__(self, port: str, baud: int, output_path: Path):
        self.port_name = port
        self.baud = baud
        self.output_path = Path(output_path)
        self.stop_event = Event()
        self.thread: Optional[Thread] = None
        self.serial_port: Optional[Any] = None
        self.output_file: Optional[Any] = None
        self.samples_written = 0

    def __enter__(self) -> "SerialImuRecorder":
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("pyserial is required for --imu-port. Install it with: python -m pip install pyserial") from exc

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.serial_port = serial.Serial(self.port_name, self.baud, timeout=0.2)
        self.output_file = self.output_path.open("a", encoding="utf-8")
        self.thread = Thread(target=self._run, name="hadesvr-imu-recorder", daemon=True)
        self.thread.start()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.serial_port is not None:
            self.serial_port.close()
        if self.output_file is not None:
            self.output_file.close()

    def _run(self) -> None:
        assert self.serial_port is not None
        assert self.output_file is not None
        while not self.stop_event.is_set():
            frame = read_frame(self.serial_port)
            if frame is None:
                continue
            frame_sequence, payload, host_timestamp_ns = frame
            sample = decode_imu_sample(frame_sequence, payload, host_timestamp_ns)
            if sample is None:
                continue
            self.output_file.write(json.dumps(sample.to_json()) + "\n")
            self.output_file.flush()
            self.samples_written += 1
