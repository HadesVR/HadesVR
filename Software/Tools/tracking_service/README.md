# HadesVR Stereo Camera MVP

This tool is the first PC-side inside-out camera MVP. It targets two front
mounted OV9281/OV9282 monochrome global-shutter USB UVC cameras.

The current implementation does not run SLAM yet. It captures timestamped
stereo frames, calibrates mono/stereo checkerboard intrinsics/extrinsics, and
shows undistorted live images. That is the base needed before OpenVINS,
ORB-SLAM3, or OpenCV stereo visual odometry are connected.

## Install

Use Python with OpenCV and NumPy. Add pyserial when recording HMD IMU packets at
the same time as camera frames:

```powershell
python -m pip install opencv-python numpy
python -m pip install pyserial
```

## Camera Config

Start from:

```text
Software\Tools\tracking_service\camera_config.ov9281.example.json
```

Important fields:

- `cameras.left.id`, `cameras.right.id`: OpenCV camera indexes.
- `width`, `height`, `fps`: initial target is 1280x800 near 100 fps.
- `exposure`, `gain`, `auto_exposure`: values are UVC-driver dependent.
- `sync.mode`: currently `software_grab_retrieve`; future modes can add hardware trigger timestamps.

## Record Stereo Images

```powershell
python Software\Tools\tracking_service\camera_mvp.py record `
  --config Software\Tools\tracking_service\camera_config.ov9281.example.json `
  --output captures\ov9281_calib_001 `
  --max-frames 80 `
  --preview
```

To record the HMD IMU stream at the same time, add the XIAO Sense serial port:

```powershell
python Software\Tools\tracking_service\camera_mvp.py record `
  --config Software\Tools\tracking_service\camera_config.ov9281.example.json `
  --output captures\ov9281_calib_imu_001 `
  --max-frames 80 `
  --imu-port COM7 `
  --preview
```

Output layout:

```text
captures\ov9281_calib_001\
  metadata.jsonl
  imu.jsonl
  left\frame_000001_...\_left.png
  right\frame_000001_...\_right.png
```

Each metadata line contains host timestamps for left and right frames plus the
estimated software sync skew.
Each IMU line contains `host_receive_timestamp_ns` from the same PC monotonic
clock plus the XIAO firmware `device_timestamp_us` when `PacketID = 4` is used.
This is enough for the first software timestamp alignment pass; hardware trigger
or external timestamp support can be added behind the provider interfaces later.

## Mono Calibration

Use a printed checkerboard. Defaults are 9x6 inner corners and 24 mm squares.

```powershell
python Software\Tools\tracking_service\camera_mvp.py calibrate-mono `
  --dataset captures\ov9281_calib_001 `
  --side left `
  --output calibration\ov9281_left.json `
  --columns 9 --rows 6 --square-size 0.024 `
  --preview

python Software\Tools\tracking_service\camera_mvp.py calibrate-mono `
  --dataset captures\ov9281_calib_001 `
  --side right `
  --output calibration\ov9281_right.json `
  --columns 9 --rows 6 --square-size 0.024 `
  --preview
```

## Stereo Calibration

```powershell
python Software\Tools\tracking_service\camera_mvp.py calibrate-stereo `
  --dataset captures\ov9281_calib_001 `
  --left-calibration calibration\ov9281_left.json `
  --right-calibration calibration\ov9281_right.json `
  --output calibration\ov9281_stereo.json `
  --columns 9 --rows 6 --square-size 0.024 `
  --preview
```

The stereo JSON stores `R`, `T`, `E`, `F`, and the left/right camera matrices.

## Undistorted Live View

```powershell
python Software\Tools\tracking_service\camera_mvp.py view-undistorted `
  --config Software\Tools\tracking_service\camera_config.ov9281.example.json `
  --calibration calibration\ov9281_stereo.json `
  --draw-horizontal-guides
```

Press `q` in the OpenCV window to stop.

## Provider Structure

The provider interface lives in `camera_providers.py`.

- `UsbStereoCameraProvider`: implemented first.
- `OakDProvider`: reserved for an OAK-D/DepthAI backend.
- `RecordedDatasetCameraProvider`: reads captured datasets for repeatable tests.

The SteamVR driver keeps the existing external PSMoveService camera path as
`ExternalTrackingProvider`. The new camera path is `InsideOutTrackingProvider`;
for now it produces calibration/capture data and will later publish HMD pose to
the driver over IPC.
