from __future__ import annotations

import argparse
import json
import time
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    from .camera_providers import StereoFrame, create_camera_provider, ensure_gray
    from .imu_serial import SerialImuRecorder
except ImportError:
    from camera_providers import StereoFrame, create_camera_provider, ensure_gray
    from imu_serial import SerialImuRecorder


def load_json(path: Path) -> Dict[str, Any]:
    return json.loads(Path(path).read_text())


def save_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def wall_time_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def frame_metadata(frame: StereoFrame, left_file: Path, right_file: Path) -> Dict[str, Any]:
    return {
        "frame_id": frame.frame_id,
        "timestamp_ns": frame.timestamp_ns,
        "left_timestamp_ns": frame.left.timestamp_ns,
        "right_timestamp_ns": frame.right.timestamp_ns,
        "sync_skew_ns": frame.sync_skew_ns,
        "wall_time_utc": wall_time_iso(),
        "left_file": str(left_file).replace("\\", "/"),
        "right_file": str(right_file).replace("\\", "/"),
    }


def command_record(args: argparse.Namespace) -> int:
    config = load_json(Path(args.config))
    output_dir = Path(args.output)
    left_dir = output_dir / "left"
    right_dir = output_dir / "right"
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    metadata_path = output_dir / "metadata.jsonl"
    start = time.monotonic()
    saved = 0

    with ExitStack() as stack:
        provider = stack.enter_context(create_camera_provider(config))
        metadata = stack.enter_context(metadata_path.open("a", encoding="utf-8"))
        imu_recorder: Optional[SerialImuRecorder] = None
        if args.imu_port:
            imu_output = output_dir / args.imu_output
            imu_recorder = stack.enter_context(SerialImuRecorder(args.imu_port, args.imu_baud, imu_output))
            print(f"recording IMU packets from {args.imu_port} to {imu_output}")

        while True:
            frame = provider.read()
            elapsed = time.monotonic() - start
            if args.duration_sec is not None and elapsed >= args.duration_sec:
                break
            if args.max_frames is not None and saved >= args.max_frames:
                break
            if frame.frame_id % args.save_every != 0:
                continue

            stem = f"frame_{frame.frame_id:06d}_{frame.timestamp_ns}"
            left_path = left_dir / f"{stem}_left.png"
            right_path = right_dir / f"{stem}_right.png"
            cv2.imwrite(str(left_path), frame.left.image)
            cv2.imwrite(str(right_path), frame.right.image)

            item = frame_metadata(frame, left_path.relative_to(output_dir), right_path.relative_to(output_dir))
            metadata.write(json.dumps(item) + "\n")
            metadata.flush()
            saved += 1

            if args.preview:
                preview = np.hstack([frame.left.image, frame.right.image])
                cv2.imshow("HadesVR stereo capture", preview)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    cv2.destroyAllWindows()
    print(f"saved {saved} stereo frames to {output_dir}")
    if imu_recorder is not None:
        print(f"saved {imu_recorder.samples_written} IMU samples to {output_dir / args.imu_output}")
    return 0


def board_points(columns: int, rows: int, square_size: float) -> np.ndarray:
    points = np.zeros((rows * columns, 3), np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points *= float(square_size)
    return points


def find_checkerboard(gray: np.ndarray, pattern_size: Tuple[int, int]) -> Optional[np.ndarray]:
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(gray, pattern_size)
        if ok:
            return corners.astype(np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not ok:
        return None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return corners


def image_size_from_gray(gray: np.ndarray) -> Tuple[int, int]:
    height, width = gray.shape[:2]
    return width, height


def dataset_side_images(dataset_dir: Path, side: str) -> List[Path]:
    side_dir = dataset_dir / side
    images = sorted(list(side_dir.glob("*.png")) + list(side_dir.glob("*.jpg")) + list(side_dir.glob("*.jpeg")))
    if not images:
        raise FileNotFoundError(f"no images found in {side_dir}")
    return images


def dataset_pairs(dataset_dir: Path) -> List[Tuple[Path, Path, Dict[str, Any]]]:
    metadata_path = dataset_dir / "metadata.jsonl"
    if metadata_path.exists():
        pairs: List[Tuple[Path, Path, Dict[str, Any]]] = []
        for line in metadata_path.read_text().splitlines():
            if not line.strip():
                continue
            item = json.loads(line)
            pairs.append((dataset_dir / item["left_file"], dataset_dir / item["right_file"], item))
        return pairs

    left_images = dataset_side_images(dataset_dir, "left")
    right_images = dataset_side_images(dataset_dir, "right")
    count = min(len(left_images), len(right_images))
    return [(left_images[i], right_images[i], {"frame_id": i + 1}) for i in range(count)]


def collect_mono_points(
    images: Sequence[Path],
    pattern_size: Tuple[int, int],
    object_template: np.ndarray,
    preview: bool,
) -> Tuple[List[np.ndarray], List[np.ndarray], Tuple[int, int], List[str]]:
    object_points: List[np.ndarray] = []
    image_points: List[np.ndarray] = []
    used_files: List[str] = []
    image_size: Optional[Tuple[int, int]] = None

    for image_path in images:
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        gray = ensure_gray(image)
        if image_size is None:
            image_size = image_size_from_gray(gray)

        corners = find_checkerboard(gray, pattern_size)
        if corners is None:
            continue

        object_points.append(object_template.copy())
        image_points.append(corners)
        used_files.append(str(image_path))

        if preview:
            view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(view, pattern_size, corners, True)
            cv2.imshow("mono calibration corners", view)
            cv2.waitKey(50)

    cv2.destroyAllWindows()
    if image_size is None:
        raise RuntimeError("no readable calibration images")
    if len(object_points) < 5:
        raise RuntimeError(f"need at least 5 valid checkerboard images, got {len(object_points)}")
    return object_points, image_points, image_size, used_files


def calibrate_mono_from_images(
    images: Sequence[Path],
    columns: int,
    rows: int,
    square_size: float,
    preview: bool = False,
) -> Dict[str, Any]:
    pattern_size = (columns, rows)
    object_template = board_points(columns, rows, square_size)
    object_points, image_points, image_size, used_files = collect_mono_points(
        images, pattern_size, object_template, preview
    )

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )

    return {
        "model": "pinhole",
        "rms": float(rms),
        "image_size": list(image_size),
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "checkerboard": {
            "columns": columns,
            "rows": rows,
            "square_size_m": square_size,
        },
        "sample_count": len(object_points),
        "used_files": used_files,
        "created_utc": wall_time_iso(),
    }


def command_calibrate_mono(args: argparse.Namespace) -> int:
    images = dataset_side_images(Path(args.dataset), args.side)
    result = calibrate_mono_from_images(
        images,
        columns=args.columns,
        rows=args.rows,
        square_size=args.square_size,
        preview=args.preview,
    )
    result["camera_id"] = args.side
    save_json(Path(args.output), result)
    print(f"mono calibration saved to {args.output}, rms={result['rms']:.4f}, samples={result['sample_count']}")
    return 0


def load_mono_calibration(path: Optional[str]) -> Optional[Dict[str, Any]]:
    if not path:
        return None
    return load_json(Path(path))


def calibration_arrays(calibration: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    camera_matrix = np.asarray(calibration["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.asarray(calibration["dist_coeffs"], dtype=np.float64)
    return camera_matrix, dist_coeffs


def command_calibrate_stereo(args: argparse.Namespace) -> int:
    dataset_dir = Path(args.dataset)
    pairs = dataset_pairs(dataset_dir)
    pattern_size = (args.columns, args.rows)
    object_template = board_points(args.columns, args.rows, args.square_size)
    left_points: List[np.ndarray] = []
    right_points: List[np.ndarray] = []
    object_points: List[np.ndarray] = []
    used_pairs: List[Dict[str, Any]] = []
    image_size: Optional[Tuple[int, int]] = None

    for left_path, right_path, item in pairs:
        left = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            continue

        left_gray = ensure_gray(left)
        right_gray = ensure_gray(right)
        if image_size is None:
            image_size = image_size_from_gray(left_gray)

        left_corners = find_checkerboard(left_gray, pattern_size)
        right_corners = find_checkerboard(right_gray, pattern_size)
        if left_corners is None or right_corners is None:
            continue

        object_points.append(object_template.copy())
        left_points.append(left_corners)
        right_points.append(right_corners)
        used_pairs.append(
            {
                "frame_id": item.get("frame_id"),
                "left_file": str(left_path),
                "right_file": str(right_path),
                "sync_skew_ns": item.get("sync_skew_ns"),
            }
        )

        if args.preview:
            left_view = cv2.cvtColor(left_gray, cv2.COLOR_GRAY2BGR)
            right_view = cv2.cvtColor(right_gray, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(left_view, pattern_size, left_corners, True)
            cv2.drawChessboardCorners(right_view, pattern_size, right_corners, True)
            cv2.imshow("stereo calibration corners", np.hstack([left_view, right_view]))
            cv2.waitKey(50)

    cv2.destroyAllWindows()
    if image_size is None:
        raise RuntimeError("no readable stereo calibration images")
    if len(object_points) < 8:
        raise RuntimeError(f"need at least 8 valid stereo checkerboard pairs, got {len(object_points)}")

    left_calibration = load_mono_calibration(args.left_calibration)
    right_calibration = load_mono_calibration(args.right_calibration)

    if left_calibration and right_calibration:
        left_camera_matrix, left_dist_coeffs = calibration_arrays(left_calibration)
        right_camera_matrix, right_dist_coeffs = calibration_arrays(right_calibration)
        flags = cv2.CALIB_FIX_INTRINSIC
    else:
        left_rms, left_camera_matrix, left_dist_coeffs, _, _ = cv2.calibrateCamera(
            object_points, left_points, image_size, None, None
        )
        right_rms, right_camera_matrix, right_dist_coeffs, _, _ = cv2.calibrateCamera(
            object_points, right_points, image_size, None, None
        )
        left_calibration = {"rms": float(left_rms)}
        right_calibration = {"rms": float(right_rms)}
        flags = 0

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    rms, left_camera_matrix, left_dist_coeffs, right_camera_matrix, right_dist_coeffs, R, T, E, F = cv2.stereoCalibrate(
        object_points,
        left_points,
        right_points,
        left_camera_matrix,
        left_dist_coeffs,
        right_camera_matrix,
        right_dist_coeffs,
        image_size,
        criteria=criteria,
        flags=flags,
    )

    result = {
        "model": "stereo_pinhole",
        "rms": float(rms),
        "image_size": list(image_size),
        "checkerboard": {
            "columns": args.columns,
            "rows": args.rows,
            "square_size_m": args.square_size,
        },
        "left": {
            "camera_matrix": left_camera_matrix.tolist(),
            "dist_coeffs": left_dist_coeffs.reshape(-1).tolist(),
            "mono_rms": left_calibration.get("rms"),
        },
        "right": {
            "camera_matrix": right_camera_matrix.tolist(),
            "dist_coeffs": right_dist_coeffs.reshape(-1).tolist(),
            "mono_rms": right_calibration.get("rms"),
        },
        "R": R.tolist(),
        "T": T.reshape(-1).tolist(),
        "E": E.tolist(),
        "F": F.tolist(),
        "sample_count": len(object_points),
        "used_pairs": used_pairs,
        "created_utc": wall_time_iso(),
    }
    save_json(Path(args.output), result)
    print(f"stereo calibration saved to {args.output}, rms={result['rms']:.4f}, pairs={result['sample_count']}")
    return 0


def undistort_pair(frame: StereoFrame, calibration: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    left_matrix = np.asarray(calibration["left"]["camera_matrix"], dtype=np.float64)
    left_dist = np.asarray(calibration["left"]["dist_coeffs"], dtype=np.float64)
    right_matrix = np.asarray(calibration["right"]["camera_matrix"], dtype=np.float64)
    right_dist = np.asarray(calibration["right"]["dist_coeffs"], dtype=np.float64)
    left = cv2.undistort(frame.left.image, left_matrix, left_dist)
    right = cv2.undistort(frame.right.image, right_matrix, right_dist)
    return left, right


def command_view_undistorted(args: argparse.Namespace) -> int:
    config = load_json(Path(args.config))
    calibration = load_json(Path(args.calibration))
    with create_camera_provider(config) as provider:
        while True:
            frame = provider.read()
            left, right = undistort_pair(frame, calibration)
            if args.draw_horizontal_guides:
                draw_guides(left)
                draw_guides(right)
            cv2.imshow("HadesVR undistorted left/right", np.hstack([left, right]))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    cv2.destroyAllWindows()
    return 0


def draw_guides(image: np.ndarray) -> None:
    height = image.shape[0]
    width = image.shape[1]
    for y in range(height // 5, height, height // 5):
        cv2.line(image, (0, y), (width - 1, y), 180, 1)


def add_board_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--columns", type=int, default=9, help="checkerboard inner corners along columns")
    parser.add_argument("--rows", type=int, default=6, help="checkerboard inner corners along rows")
    parser.add_argument("--square-size", type=float, default=0.024, help="checkerboard square size in meters")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="HadesVR inside-out stereo camera MVP tools")
    subparsers = parser.add_subparsers(dest="command", required=True)

    record = subparsers.add_parser("record", help="capture timestamped stereo images")
    record.add_argument("--config", required=True)
    record.add_argument("--output", required=True)
    record.add_argument("--duration-sec", type=float)
    record.add_argument("--max-frames", type=int)
    record.add_argument("--save-every", type=int, default=1)
    record.add_argument("--imu-port", help="optional HMD USB serial port for synchronized IMU logging, for example COM7")
    record.add_argument("--imu-baud", type=int, default=230400)
    record.add_argument("--imu-output", default="imu.jsonl")
    record.add_argument("--preview", action="store_true")
    record.set_defaults(func=command_record)

    mono = subparsers.add_parser("calibrate-mono", help="calibrate one camera from checkerboard images")
    mono.add_argument("--dataset", required=True)
    mono.add_argument("--side", choices=["left", "right"], required=True)
    mono.add_argument("--output", required=True)
    mono.add_argument("--preview", action="store_true")
    add_board_args(mono)
    mono.set_defaults(func=command_calibrate_mono)

    stereo = subparsers.add_parser("calibrate-stereo", help="calibrate stereo extrinsics from paired checkerboard images")
    stereo.add_argument("--dataset", required=True)
    stereo.add_argument("--output", required=True)
    stereo.add_argument("--left-calibration")
    stereo.add_argument("--right-calibration")
    stereo.add_argument("--preview", action="store_true")
    add_board_args(stereo)
    stereo.set_defaults(func=command_calibrate_stereo)

    undistort = subparsers.add_parser("view-undistorted", help="display undistorted live stereo images")
    undistort.add_argument("--config", required=True)
    undistort.add_argument("--calibration", required=True)
    undistort.add_argument("--draw-horizontal-guides", action="store_true")
    undistort.set_defaults(func=command_view_undistorted)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
