# Driver

# Driver Installation
Driver installation is quite simple, the steps are as follows:
* Download the latest release of the driver from the [Releases Page](https://github.com/HadesVR/HadesVR/releases)
* Unzip the driver into your Steam\steamapps\common\SteamVR\drivers folder.
* Configure the driver

Assuming you have already followed the steps in the [Tracking](Tracking.md) doc, you should have PSMoveService already installed in your PC.

# Driver Configuration

Driver configuration is pretty self explanatory, the driver configuration file is  hadesvr\resources\settings\default.vrsettings. To change the settings just open the file with the text editor of your liking.

|Parameter|Type    |Description|
| ------  | ------ |------     |
| DebugMode  | bool | ------     |
| Stereo  | bool | ------     |
| IsDisplayOnDesktop  | bool | ------     |
| IsDisplayReal  | bool | ------     |
| DistanceBetweenEyes  | float | ------     |
| IPD  | float | ------     |
| FOV  | float | ------     |
| serialNumber  | string | ------     |
| windowWidth  | int | ------     |
| windowHeight  | int | ------     |
| renderWidth  | int | ------     |
| renderHeight  | int | ------     |
| displayFrequency  | float | ------     |
| windowX  | int | ------     |
| windowY  | int | ------     |
| ScreenOffsetX  | int | ------     |
| ScreenOffsetY  | int | ------     |
| ZoomWidth  | float | ------     |
| ZoomHeight  | float | ------     |
| DistortionK1  | float | ------     |
| DistortionK2  | float | ------     |
| secondsFromVsyncToPhotons  | float | ------     |
| ComPort  | int | ------     |
| EnableControllers  | bool | ------     |

# Usage and controller bindings

* Launch PSMoveService before opening SteamVR
* Launch SteamVR
* Close the "Running on monitor mode" notice
* Click on the Screen Viewport

Controller bindings:
Pressing F8 or Pressing Joystick click + trigger button on both controllers at the same time resets the yaw axis, make sure you're facing the correct way before doing so or else your controllers' axis will not match your headset axis.