from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Optional
import json
import time

import cv2


@dataclass
class CameraFrame:
    image: Any
    timestamp_ns: int
    camera_id: str
    frame_id: int


@dataclass
class StereoFrame:
    left: CameraFrame
    right: CameraFrame
    timestamp_ns: int
    frame_id: int
    sync_skew_ns: int


class CameraProvider(ABC):
    @abstractmethod
    def open(self) -> None:
        pass

    @abstractmethod
    def close(self) -> None:
        pass

    @abstractmethod
    def read(self) -> StereoFrame:
        pass

    def __enter__(self) -> "CameraProvider":
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()


def backend_from_name(name: str) -> int:
    if not name:
        return 0
    key = name.upper()
    if key == "ANY":
        return 0
    if key == "DSHOW":
        return cv2.CAP_DSHOW
    if key == "MSMF":
        return cv2.CAP_MSMF
    return int(name)


def fourcc_from_name(name: Optional[str]) -> Optional[int]:
    if not name:
        return None
    text = name.strip()
    if len(text) != 4:
        raise ValueError(f"fourcc must have exactly 4 characters, got {text!r}")
    return cv2.VideoWriter_fourcc(*text)


class UsbStereoCameraProvider(CameraProvider):
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.left_config = config["cameras"]["left"]
        self.right_config = config["cameras"]["right"]
        self.backend = backend_from_name(config.get("backend", "DSHOW"))
        self.frame_id = 0
        self.left_capture: Optional[cv2.VideoCapture] = None
        self.right_capture: Optional[cv2.VideoCapture] = None

    def open(self) -> None:
        self.left_capture = self._open_one(self.left_config, "left")
        self.right_capture = self._open_one(self.right_config, "right")

    def close(self) -> None:
        for capture in (self.left_capture, self.right_capture):
            if capture is not None:
                capture.release()
        self.left_capture = None
        self.right_capture = None

    def read(self) -> StereoFrame:
        if self.left_capture is None or self.right_capture is None:
            raise RuntimeError("provider is not open")

        left_ok = self.left_capture.grab()
        left_timestamp_ns = time.monotonic_ns()
        right_ok = self.right_capture.grab()
        right_timestamp_ns = time.monotonic_ns()

        if not left_ok or not right_ok:
            raise RuntimeError("failed to grab from one or both cameras")

        left_ok, left_image = self.left_capture.retrieve()
        right_ok, right_image = self.right_capture.retrieve()
        if not left_ok or not right_ok:
            raise RuntimeError("failed to retrieve from one or both cameras")

        if self.config.get("force_grayscale", True):
            left_image = ensure_gray(left_image)
            right_image = ensure_gray(right_image)

        timestamp_ns = (left_timestamp_ns + right_timestamp_ns) // 2
        sync_skew_ns = abs(right_timestamp_ns - left_timestamp_ns)
        self.frame_id += 1

        return StereoFrame(
            left=CameraFrame(left_image, left_timestamp_ns, "left", self.frame_id),
            right=CameraFrame(right_image, right_timestamp_ns, "right", self.frame_id),
            timestamp_ns=timestamp_ns,
            frame_id=self.frame_id,
            sync_skew_ns=sync_skew_ns,
        )

    def _open_one(self, camera_config: Dict[str, Any], side: str) -> cv2.VideoCapture:
        camera_id = camera_config["id"]
        capture = cv2.VideoCapture(camera_id, self.backend)
        if not capture.isOpened():
            raise RuntimeError(f"unable to open {side} camera id {camera_id}")

        fourcc = fourcc_from_name(camera_config.get("fourcc", self.config.get("fourcc")))
        if fourcc is not None:
            capture.set(cv2.CAP_PROP_FOURCC, fourcc)

        width = int(camera_config.get("width", self.config.get("width", 1280)))
        height = int(camera_config.get("height", self.config.get("height", 800)))
        fps = float(camera_config.get("fps", self.config.get("fps", 100)))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, fps)

        buffer_size = camera_config.get("buffer_size", self.config.get("buffer_size"))
        if buffer_size is not None:
            capture.set(cv2.CAP_PROP_BUFFERSIZE, int(buffer_size))

        self._set_optional(capture, cv2.CAP_PROP_AUTO_EXPOSURE, camera_config.get("auto_exposure"))
        self._set_optional(capture, cv2.CAP_PROP_EXPOSURE, camera_config.get("exposure"))
        self._set_optional(capture, cv2.CAP_PROP_GAIN, camera_config.get("gain"))
        self._set_optional(capture, cv2.CAP_PROP_AUTOFOCUS, camera_config.get("autofocus"))
        self._set_optional(capture, cv2.CAP_PROP_AUTO_WB, camera_config.get("auto_white_balance"))

        return capture

    @staticmethod
    def _set_optional(capture: cv2.VideoCapture, prop: int, value: Optional[Any]) -> None:
        if value is not None:
            capture.set(prop, float(value))


class OakDProvider(CameraProvider):
    def __init__(self, config: Dict[str, Any]):
        self.config = config

    def open(self) -> None:
        raise NotImplementedError("OakDProvider is a future provider stub")

    def close(self) -> None:
        pass

    def read(self) -> StereoFrame:
        raise NotImplementedError("OakDProvider is a future provider stub")


class RecordedDatasetCameraProvider(CameraProvider):
    def __init__(self, dataset_dir: Path):
        self.dataset_dir = Path(dataset_dir)
        self.frames: List[Dict[str, Any]] = []
        self.iterator: Optional[Iterator[Dict[str, Any]]] = None

    def open(self) -> None:
        metadata_path = self.dataset_dir / "metadata.jsonl"
        if not metadata_path.exists():
            raise FileNotFoundError(metadata_path)
        self.frames = [json.loads(line) for line in metadata_path.read_text().splitlines() if line.strip()]
        self.iterator = iter(self.frames)

    def close(self) -> None:
        self.iterator = None

    def read(self) -> StereoFrame:
        if self.iterator is None:
            raise RuntimeError("provider is not open")
        item = next(self.iterator)
        left_image = cv2.imread(str(self.dataset_dir / item["left_file"]), cv2.IMREAD_GRAYSCALE)
        right_image = cv2.imread(str(self.dataset_dir / item["right_file"]), cv2.IMREAD_GRAYSCALE)
        if left_image is None or right_image is None:
            raise RuntimeError(f"failed to load dataset frame {item.get('frame_id')}")
        frame_id = int(item["frame_id"])
        left_ts = int(item["left_timestamp_ns"])
        right_ts = int(item["right_timestamp_ns"])
        return StereoFrame(
            left=CameraFrame(left_image, left_ts, "left", frame_id),
            right=CameraFrame(right_image, right_ts, "right", frame_id),
            timestamp_ns=int(item["timestamp_ns"]),
            frame_id=frame_id,
            sync_skew_ns=abs(right_ts - left_ts),
        )


def ensure_gray(image: Any) -> Any:
    if len(image.shape) == 2:
        return image
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def create_camera_provider(config: Dict[str, Any]) -> CameraProvider:
    provider_type = config.get("provider", "UsbStereoCameraProvider")
    if provider_type == "UsbStereoCameraProvider":
        return UsbStereoCameraProvider(config)
    if provider_type == "OakDProvider":
        return OakDProvider(config)
    if provider_type == "RecordedDatasetCameraProvider":
        return RecordedDatasetCameraProvider(Path(config["dataset_dir"]))
    raise ValueError(f"unknown camera provider {provider_type!r}")
