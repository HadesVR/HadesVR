
![name](docs/img/name.png)

# HadesVR       [![Release](https://img.shields.io/github/v/release/HadesVR/HadesVR.svg)](../../releases/latest)
HadesVR is a SteamVR compatible VR setup made for tinkerers. 

The setup includes DIY controllers that are capable of emulating HTC vive wands or most aspects of the Valve Index Knuckles controllers (including finger tracking!). It also includes tracking electronics for a Headset, including an integrated wireless receiver to receive the controllers' data.

There's also custom controller hardware like 3d printable shells (still a bit of a WIP) with their respective custom PCBs to build your own Knuckles controllers.

The SteamVR driver used to be based off of [TrueOpenVR](https://github.com/TrueOpenVR) but it's modified so heavily I'm making it it's own thing.
This driver also uses [PSMoveService](https://github.com/psmoveservice/PSMoveService) (for now at least) for the positional tracking of HMD and controllers, using ping pong balls and different colours of LED's.

For more information on *everything*, check out the [docs](docs/DocsIndex.md)!

![1](docs/img/Headset.png)

# Custom hardware
Thinking of building your own set of controllers? check out the [guide](docs/DocsIndex.md#controllers), for more info on the custom pcbs and 3d printable parts!

Or maybe you want to design your own, check out the [pinouts](docs/ControllerPinouts.md) for building the controllers with regular Arduino boards!

# To-do list:

### Github TODO:
- Write a basic guide and troubleshooting
- Write the joystick calibration sketches

### Project TODO:
- Fix the 3d printable tabs
- Figure out a better way to mount the tp4056 that doesnt involve hot glue
- Implement hand bone finger tracking for games that use it
- **Upgrade Serial to HID**
