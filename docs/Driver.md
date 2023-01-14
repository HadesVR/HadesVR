# Driver

# Driver Installation
Driver installation is quite simple, the steps are as follows:
* Download the latest release of the driver from the [Releases Page](https://github.com/HadesVR/HadesVR/releases)
* Unzip the driver into your Steam\steamapps\common\SteamVR\drivers folder.
* Configure the driver

Assuming you have already followed the steps in the [Tracking](Tracking.md) doc, you should have PSMoveService already installed in your PC.

If you don't plan on making a 6dof headset (a 3dof headset for Sim racing, etc), you can skip the PSMoveService related steps.

# Driver Configuration

Driver configuration is pretty self explanatory, the driver configuration file is stored in hadesvr\resources\settings\default.vrsettings. To change the settings just open the file with the text editor of your liking.

If you're planning to use different drivers at the same time (HadesVR controllers with TOVR HMD for example), you'll also need to enable the `activateMultipleDrivers` setting in the Steam\config\steamvr.vrsettings file. 

The driver configuration is divided into a couple sections for tidyness, these are the following:

### "Driver" section where you'll find:
|Parameter|Type    |Description|
| ------  | ------ |------     |
| TransportMode | string | Either HID or UART depending on how HMD is connected, see below. |
| PSMSTrackerFrequency | int | The maximum update rate you have set on your psmoveservice trackers.|
| EnableDirectMode | bool | Enable direct mode? (Experimental, sort of broken. default value is 'false'.|
| EDID_VID | int | EDID VID for direct mode.|
| EDID_PID | int | EDID PID for direct mode.|

If `TransportMode` is set to `HID` (or left unset):

|Parameter|Type    |Description|
| ------  | ------ |------     |
| HID_VID | int | The VID value of your HID device, in decimal numbers. |
| HID_PID | int | The PID value of your HID device, in decimal numbers. |

If `TransportMode` is set to `UART`:

|Parameter|Type    |Description|
| ------  | ------ |------     |
| UART_Port | string | UART port name HMD is connected to, e.g. COM3. |
| UART_Baudrate | int | Baudrate used for UART communication, e.g. 115200 or 230400. |

### "Display" section where you'll find:
|Parameter|Type    |Description|
| ------  | ------ |------     |
| Stereo  | bool | Enables or disables Stereo display mode, keep enabled unless you're a cyclops.     |
| IsDisplayOnDesktop  | bool | This makes it so a warning telling you your display isn't in fullscreen mode pops up when your window focus isn't on the VR viewport.     |
| IsDisplayReal  | bool | ------     |
| IsSinglePanel  | bool | Set to true if using a single panel display.     |
| IPD  | float | IPD setting, in meters.     |
| FOV  | float | FOV setting, in degrees.    |
| windowWidth  | int | Resolution Width of your VR screen     |
| windowHeight  | int | Resolution Height of your VR screen     |
| renderWidth  | int | The width (per eye) of the desired render resolution.     |
| renderHeight  | int |The height (per eye) of the desired render resolution.     |
| displayFrequency  | float | Here goes the refresh rate of your display.     |
| windowX  | int | Here goes the Width resolution of your main screen.     |
| windowY  | int | Don't touch this one.     |
| ScreenOffsetX  | int | Viewport X offset.     |
| ScreenOffsetY  | int | Viewport Y offset.     |
| ViewportZoom  | float | Viewport Zoom, lower means higher zoom.     |
| DistanceBetweenLenses  | float | This is the physical distance between the two lenses, in meters     |
| DistanceBetweenViews  | float | Distance between views, in meters. This should be set as same as IPD in single panel mode and 0 in dual panel mode.     |
| DisplayWidth  | float | Physical width of the display, in meters     |
| DisplayCantAngle  | float | Cant angle in degrees of the displays (Experimental, slightly broken, things don't render properly at the edges of the views).     |
| secondsFromVsyncToPhotons  | float | ------     |

### "Distortion" section where you'll find:
|Parameter|Type    |Description|
| ------  | ------ |------     |
| Red_K1,K2,K3  | float | Red distortion coefficients     |
| Green_K1,K2,K3  | float | Green distortion coefficients|
| Blue_K1,K2,K3 |float| Blue distortion coefficients |

### "HMD" section where you'll find:
|Parameter|Type    |Description|
| ------  | ------ |------     |
| serialNumber  | string | The serial number of the HMD     |
| modelNumber  | string | The model of the HMD     |
| EnableHMD  | bool | Enables or disables the HMD part of the driver |
| MinFilterBeta | float | Minimum filter beta of the rotation filter |
| MaxFilterBeta | float | Maximum filter beta of the rotation filter |
| UseAccelerometers | bool | Enables the use of the IMU's accelerometer for position smoothing |
| Camera_Kalman_Measurement_Uncertainty | float | Camera measurement uncertainty of the Kalman position filter |
| Camera_Kalman_Estimation_Uncertainty | float | Camera estimation uncertainty of the Kalman position filter |
| Camera_Kalman_Process_Noise | float | Camera process noise of the Kalman position filter |
| IMU_Kalman_Measurement_Uncertainty | float | IMU measurement uncertainty of the Kalman position filter |
| IMU_Kalman_Estimation_Uncertainty | float | IMU estimation uncertainty of the Kalman position filter |
| IMU_Kalman_Process_Noise | float | IMU process noise of the Kalman position filter |
|HMDYawOffset|float| Yaw offset of the HMD in degrees|
|HMDPitchOffset|float| Pitch offset of the HMD in degrees|
|HMDRollOffset|float| Roll offset of the HMD in degres|
|HMDXOffset|float| X offset of the HMD in meters|
|HMDYOffset|float| Y offset of the HMD in meters|
|HMDZOffset|float| Z offset of the HMD in meters|

### "Controllers" section where you'll find:
|Parameter|Type    |Description|
| ------  | ------ |------     |
| EnableControllers  | bool | Variable to enable or disable controllers.     |
| ControllerMode  | int | Sets the controller type, 0 being Knuckles controllers and 1 being wand controllers.     |
| UseAccelerometers | bool | Enables the use of the IMU's accelerometer for position smoothing |
| Camera_Kalman_Measurement_Uncertainty | float | Camera measurement uncertainty of the Kalman position filter |
| Camera_Kalman_Estimation_Uncertainty | float | Camera estimation uncertainty of the Kalman position filter |
| Camera_Kalman_Process_Noise | float | Camera process noise of the Kalman position filter |
| IMU_Kalman_Measurement_Uncertainty | float | IMU measurement uncertainty of the Kalman position filter |
| IMU_Kalman_Estimation_Uncertainty | float | IMU estimation uncertainty of the Kalman position filter |
| IMU_Kalman_Process_Noise | float | IMU process noise of the Kalman position filter |
|CTRLRightYawOffset | float | Yaw offset of the Right controller in degrees|
|CTRLRightPitchOffset | float | Pitch offset of the Right controller in degrees|
|CTRLRightRollOffset | float | Roll offset of the Right controller in degrees|
|CTRLRightXOffset|float| X offset of the Right controller in meters|
|CTRLRightYOffset|float| Y offset of the Right controller in meters|
|CTRLRightZOffset|float| Z offset of the Right controller in meters|
|CTRLLeftYawOffset | float | Yaw offset of the Left controller in degrees|
|CTRLLeftPitchOffset | float | Pitch offset of the Left controller in degrees|
|CTRLLeftRollOffset | float | Roll offset of the Left controller in degrees|
|CTRLLeftXOffset|float| X offset of the Left controller in meters|
|CTRLLeftYOffset|float| Y offset of the Left controller in meters|
|CTRLLeftZOffset|float| Z offset of the Left controller in meters|

### "Trackers" section where you'll find:
|Parameter|Type    |Description|
| ------  | ------ |------     |
|EnableTrackers| bool| Enables or disables trackers (still work in progress, trackers do nothing as of right now.)
|TrackerMode| int | 0 meaning full body, 1 meaning waist tracking. |

### "Experimental" section where you'll find:
|Parameter|Type    |Description|
| ------  | ------ |------     |
|EnableDriftCorrection| bool| Enables or disables experimental drift correction
|LowerVelocityTreshold| float | The lower threshold of velocity in m/s in which the driver will attempt to correct for drift |
|UpperVelocityTreshold| float | The upper threshold of velocity in m/s in which the driver will attempt to correct for drift |
|Measurement_Uncertainty| float | Kalman filter measurement uncertainty for the smoothing filter used in drift correction. |
|Estimation_Uncertainty| float | Kalman filter estimation uncertainty for the smoothing filter used in drift correction. |
|HMD_Process_Noise| float | Kalman filter process noise for the smoothing filter used in drift correction on the HMD. 0 disables it |
|Controller_Process_Noise| float | Kalman filter process noise for the smoothing filter used in drift correction on the Controllers. 0 disables it |

# HID configuration
To configure the driver you will need the VID and PID values from the board you're using. The easiest way of getting them is going to the Arduino IDE, clicking on tools and clicking on get board info with the receiver plugged in:

![3](img/Driver/3.png)

Do note these values are in HEX so to use them in the driver config file you'll need to convert them to decimal numbers.
to do that you can use websites like [Rapidtables](https://www.rapidtables.com/convert/number/hex-to-decimal.html), just input your VID and PID values one at a time and convert them to decimal numbers.

Once done, you'll get values like these and then all you have to do is load them up in your driver config file, the values you're looking to change are `HID_VID` and `HID_PID`, under the `Driver` section.

![4](img/Driver/4.png)

Make sure not to delete any commas, and not to get the VID and PID values backwards and proceed with the Display configuration.


# Display configuration

for your headset display to be recognized properly you must align it flat with the top right corner of your main display as seen in the following example:

![1](img/Driver/1.png)

3 being your main display and 2 being your HMD's display. then you must input the Width of your main display (3 in this case) in pixels onto the `windowX` setting on the settings file but do not modify the windowY value of 0.

Then you've gotta set `windowWidth` and `windowHeight` to the respective width and height values of your vr screen, same goes for `renderWidth` and `renderHeight`. And last but not least: Set the `displayFrequency` to the one of your HMD's display.

If this doesn't work you might have messed up the alignment of the displays on the screen settings, remember the top line has to be flat like in the example picture.

you can also show the main vr output on your main screen for testing purposes by doing the following:
- Set `windowWidth` and `windowHeight` to the width and height of your main display
- Set the `renderWidth` and `renderHeight` to the width and height of your main display
- Set `windowX` to 0

# Usage and controller bindings

* Launch PSMoveService before opening SteamVR
* Launch SteamVR
* Close the "Running on monitor mode" notice
* Click on the Screen Viewport
* Calibrate controllers and headset by pressing F8 on your keyboard or the respective binding for it. (more on calibration below)

⚠️ if you just turned on your controllers/hmd it might drift for up to a minute, this is normal and you just need to keep it stil for a couple seconds until it settles.

## Initial calibration:
[TODO]
 
## Bindings:
### HMD only bindings:
Pressing F8 resets your yaw axis in case of IMU drift.

### Knuckles and Wand controllers:
Pressing Joystick click + trigger button on both controllers at the same time resets the yaw axis, make sure you're facing the correct way before doing so or else your controllers' axis will not match your headset axis.