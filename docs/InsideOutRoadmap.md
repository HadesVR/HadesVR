# HadesVR Inside-Out Roadmap

This document records the Phase 0 repository survey and the first MVP changes
for turning HadesVR into a staged inside-out HMD project.

Video note: the two referenced YouTube videos were not accessible from this
environment, so the user-provided target specification is treated as the source
of truth.

## Phase 0 survey

HadesVR is split into two main software areas:

- `Software/Firmware`: Arduino sketches for headset, RF receiver, and
  controller variants.
- `Software/Driver`: a Windows SteamVR/OpenVR driver written in C++.

The current HadesVR pipeline is:

```text
HMD IMU
  Software/Firmware/Headset/Headset.ino
  -> PacketID 3, 63 byte vendor HID/UART payload
  -> Software/Driver/.../DataTransport
  -> CdataHandler::ReadTransportData()
  -> Madgwick filter
  -> CdataHandler::GetHMDPose()
  -> C_HMDDeviceDriver::RunFrame()
  -> VRServerDriverHost()->TrackedDevicePoseUpdated()

Controllers
  Software/Firmware/Controller_INDEX or Controller_HTC
  -> MPU9250 + buttons + sticks + trigger + finger inputs
  -> NRF24L01 RF payload, one controller at a time
  -> Headset.ino or Receiver.ino merges right/left into PacketID 2
  -> CdataHandler::ReadTransportData()
  -> CdataHandler::GetControllersPose()
  -> C_ControllerDriver::RunFrame()
  -> pose update + VRDriverInput component updates

Outside-in position
  PSMoveServiceEX / PSMoveService
  -> ExternalTrackingProvider
  -> CdataHandler::connectToPSMOVE()
  -> CdataHandler::PSMUpdate()
  -> HMD/controller Kalman position updates
  -> GetHMDPose()/GetControllersPose()

Fallback position
  If PSMoveService is not connected, HMD/controller poses use fixed positions
  while orientation still comes from IMU.
```

## Where things live

- HMD IMU read: `Software/Firmware/Headset/Headset.ino`
- HMD IMU pose filter: `Software/Driver/src/samples/driver_HadesVR/src/dataHandler.cpp`
- HMD pose submitted to SteamVR: `Software/Driver/src/samples/driver_HadesVR/src/driver_main.cpp`
- Controller IMU/buttons read: `Software/Firmware/Controller_INDEX/*/*_controller_Filter.ino` and `Software/Firmware/Controller_HTC/*/*_controller_Filter.ino`
- NRF24L01 RF send/receive: controller sketches, `Software/Firmware/Headset/Headset.ino`, `Software/Firmware/Receiver/Receiver.ino`
- PC HID transport: firmware inline HID classes and `Software/Driver/src/samples/driver_HadesVR/src/HIDTransport.cpp`
- PC UART transport: firmware inline UART classes and `Software/Driver/src/samples/driver_HadesVR/src/UARTTransport.cpp`
- PSMoveService position provider: `CdataHandler::connectToPSMOVE()` and `CdataHandler::PSMUpdate()`
- Controller pose/input submitted to SteamVR: `C_ControllerDriver::RunFrame()` and `include/devices.hpp`
- Existing SteamVR skeleton approximation: `src/HandTracking.cpp` and `include/HandTracking.h`
- Settings: `Software/Driver/resources/settings/default.vrsettings`, `default-dev.vrsettings`, and `include/settingsAPIKeys.h`

## Tracking provider direction

HadesVR's existing outside-in PlayStation Eye / PSMoveService path is kept as:

- `ExternalTrackingProvider`: current HadesVR external cameras, color blobs, and PSMoveService position data.

The new front-camera path is added as:

- `InsideOutTrackingProvider`: HMD-mounted stereo cameras plus HMD IMU. The first MVP only captures synchronized images, calibrates cameras, and aligns timestamps. Full SLAM/VIO comes later.

Legacy `PoseProvider = "PSMove"` is still accepted by the driver for compatibility, but new configs should use `ExternalTrackingProvider` or `InsideOutTrackingProvider`.

## Phase 1 and 2 MVP implemented here

The first MVP keeps the driver architecture mostly intact:

- Add a XIAO nRF52840 Sense headset sketch:
  `Software/Firmware/Headset_XIAO_nRF52840_Sense/Headset_XIAO_nRF52840_Sense.ino`
- Read the onboard LSM6DS3TR-C through the Seeed `LSM6DS3` library.
- Configure accel/gyro ODR to 416 Hz and transmit at `TARGET_SAMPLE_RATE_HZ = 250`.
- Send a timestamped raw IMU packet through USB Serial using the existing HadesVR UART frame wrapper.
- Add `PacketID = 4` in the SteamVR driver and feed it into the existing Madgwick filter path.
- Keep old `PacketID = 3` support intact.
- Fix the UART frame-length check in `UARTTransport.cpp`.
- Add `PoseProvider` setting:
  - `ExternalTrackingProvider`: existing PSMoveService behavior.
  - `InsideOutTrackingProvider`: skip PSMoveService and reserve the driver path for the inside-out tracking service.
  - `PSMove`: legacy alias for `ExternalTrackingProvider`.
  - `None`: skip PSMoveService and use fixed 3DoF fallback position.
- Add `Software/Tools/hadesvr_serial_imu_monitor.py` to verify packets before running SteamVR.

## Inside-out camera MVP

The inside-out tracking cameras are no longer PlayStation Eye cameras. The preferred camera family is OV9281 or OV9282 monochrome global-shutter USB UVC cameras.

Initial MVP hardware assumption:

- Two front-mounted OV9281 USB UVC cameras, left and right.
- Monochrome global shutter.
- 60 fps minimum, around 100 fps preferred.
- 640x480 minimum, 1280x800 preferred.
- M12 wide-angle lenses.
- Target horizontal FOV: 100 degrees or more.
- OpenCV `VideoCapture` compatibility.
- Manual/fixed exposure and gain where the UVC driver allows it.
- White balance and autofocus disabled where exposed by the driver. OV9281/OV9282 mono modules normally do not need white balance or autofocus, but the config has fields for UVC devices that expose those controls.
- Host timestamps recorded for left and right frames.
- Provider structure ready for later hardware trigger timestamps or software frame matching.

Implemented PC-side MVP tools:

- `Software/Tools/tracking_service/camera_providers.py`
  - `CameraProvider` interface.
  - `UsbStereoCameraProvider`, implemented first.
  - `OakDProvider`, future stub.
  - `RecordedDatasetCameraProvider`, dataset playback.
- `Software/Tools/tracking_service/camera_mvp.py`
  - Simultaneous left/right OpenCV capture using `grab()` then `retrieve()`.
  - JSON camera config for camera IDs, resolution, fps, exposure, gain, backend, and UVC controls.
  - Timestamped stereo image recording with `metadata.jsonl`.
  - Optional HMD IMU serial logging to `imu.jsonl` using the same PC monotonic timestamp domain.
  - Checkerboard mono calibration.
  - Checkerboard stereo calibration.
  - JSON calibration result output.
  - Live undistorted left/right view.
- `Software/Tools/tracking_service/camera_config.ov9281.example.json`
  - OV9281/OV9282 stereo UVC starter config.

## PacketID 4 format

USB Serial frame:

```text
0xAA, frame_seq, payload_size=64, report_id=0x01, packet[63]
```

Packet payload:

```text
uint8  PacketID        = 4
uint8  Version         = 1
uint16 Sequence
uint32 TimestampUs     = micros()
int16  AccX/Y/Z        = accel_g * 2048
int16  GyroX/Y/Z       = gyro_dps * 16
int16  MagX/Y/Z        = 0 for LSM6DS3TR-C
int16  Temperature     = degC * 100
uint16 HMDData
uint16 SamplePeriodUs
uint8  ImuStatus       = bit0: IMU OK
uint8  Reserved[30]
```

The driver sees the report id as byte 0, so the C++ struct has an extra leading
`HIDID` field and totals 64 bytes.

## Test flow for the MVP

1. Install the Seeed nRF52 board support and `Seeed_Arduino_LSM6DS3`.
2. Flash `Headset_XIAO_nRF52840_Sense.ino` to the XIAO nRF52840 Sense.
3. Find the COM port.
4. Verify packets:

```powershell
python Software\Tools\hadesvr_serial_imu_monitor.py COM7
```

5. Set driver settings:

```json
"TransportMode" : "UART",
"PoseProvider" : "InsideOutTrackingProvider",
"UART_Port" : "COM7",
"UART_Baudrate" : 230400
```

6. Build/install the SteamVR driver and confirm that HMD rotation updates.
7. Use the stereo camera MVP to record calibration images and generate camera JSON files.

## Main difficulties

- XIAO Sense has a 6-axis IMU, so yaw will drift without magnetometer or camera/VIO correction.
- SteamVR drivers should stay lightweight; OpenCV, MediaPipe, and SLAM should run in a separate tracking service and communicate with the driver through a narrow local IPC protocol.
- Visual-inertial tracking needs precise camera intrinsics, camera-to-IMU extrinsics, and timestamp alignment.
- Monocular inside-out has scale ambiguity unless VIO uses IMU properly, stereo/depth is added, or a calibration constraint is used.
- Two independent UVC cameras are not hardware-synchronized by default. The first MVP records host timestamps and software skew; low-latency SLAM will eventually need better software matching or hardware trigger support.
- Wide FOV M12 lenses introduce noticeable distortion. Calibration quality matters more than nominal resolution.
- Camera pose and hand tracking will compete for camera frames and CPU/GPU time; capture should be centralized in the tracking service.
- The existing controller RF path assumes NRF24L01 and merged 63 byte packets. BLE should preserve the logical controller packet but replace the transport layer.

## Proposed module direction

```text
Firmware
  hmd_xiao_sense
    imu_driver
    packet_codec
    usb_serial_transport
    later: ble_transport

  controller_xiao_ble
    mpu9250_driver
    input_scan
    controller_packet_codec
    ble_transport

PC
  SteamVR driver
    DataTransport
    HMD/controller packet decoder
    PoseProvider switch
    OpenVR device + input submission

  tracking_service
    camera_provider
      UsbStereoCameraProvider
      OakDProvider
      RecordedDatasetCameraProvider
    camera_capture
    camera_calibration
    imu_receiver
    pose_fusion
    hand_tracking
    ipc_server
```

## SLAM/VIO candidates

- OpenCV-only MVP: best for camera calibration, frame capture, ArUco/Charuco debug, optical-flow experiments, and proving camera timing. It is not enough for robust 6DoF by itself.
- OpenVINS: strong match for a monocular/stereo visual-inertial odometry service. It is ROS-oriented but also has ROS-free components, so it is a good later candidate after timestamping and calibration are solid.
- ORB-SLAM3: supports visual, visual-inertial, and multi-map SLAM, but its reference build path is Linux-oriented and heavier to integrate on Windows.
- RTAB-Map: practical for RGB-D/stereo/robotics style SLAM and has a standalone application/library. It is useful for evaluation and mapping, but may be heavier than needed for first HMD pose MVP.

Recommended sequence:

1. OpenCV calibration and camera timestamp capture.
2. OpenCV optical-flow/feature VO prototype for learning and diagnostics only.
3. OpenVINS as the first serious VIO integration candidate.
4. Keep ORB-SLAM3 and RTAB-Map as comparison backends behind the same tracking-service output interface.

## Hand tracking path

Start with MediaPipe Hand Landmarker in the tracking service, not inside the
SteamVR driver. The service should output:

- handedness
- wrist/palm estimate in HMD camera coordinates
- 21 landmarks
- pinch confidence
- simple gesture states

MVP mapping:

- pinch -> trigger/click
- open palm -> grip released
- closed hand or pinch strength -> grip/finger values
- rough hand position -> controller pose fallback

Later mapping:

- convert landmarks into SteamVR Skeleton Input bone transforms, reusing
  `HandTracking.cpp` as the bridge from finger flexion to skeleton poses.

## Future phases

- Phase 3: OV9281/OV9282 stereo camera capture and calibration tool in a separate tracking service.
- Phase 4: camera/IMU timestamp alignment quality checks and richer dataset recording format.
- Phase 5: local IPC between tracking service and driver; `InsideOutTrackingProvider` reads HMD position from the service.
- Phase 6: OpenCV stereo VO diagnostics and timing validation.
- Phase 7: OpenVINS or another VIO backend for 6DoF HMD position.
- Phase 8: MediaPipe hand tracking to controller-like input.
- Phase 9: BLE controller firmware and PC/HMD BLE bridge.
- Phase 10: SteamVR Skeleton Input from camera hand landmarks.
