
![name](docs/img/name.png)

# HadesVR
HadesVR is a SteamVR compatible VR setup made for tinkerers. 

The setup includes 3d printable controllers with custom PCB's, capable of emulating most aspects of the Valve Index Knuckles controllers (including finger tracking!) and tracking electronics for a Headset, including a wireless receiver to receive the controllers' data.

The SteamVR driver used to be based off of [TrueOpenVR](https://github.com/TrueOpenVR) but it's modified so heavily I'm making it it's own thing.
This driver also uses [PSMoveService](https://github.com/psmoveservice/PSMoveService) (for now at least) for the positional tracking of HMD and controllers, using ping pong balls and different colours of LED's.

For more information on *everything*, check out the [docs](docs/DocsIndex.md)!

# Building your own controllers
Thinking of building your own set of controllers? check out the [guide](docs/DocsIndex.md#controllers), for more info on the custom pcbs and 3d printable parts!
Or maybe you want to design your own, check out the [pinouts](docs/ControllerPinouts.md) for building the controllers with a regular Arduino!

# To-do list:

### Github TODO:
- Write a basic guide and troubleshooting
- Make a couple basic schematic diagrams
- Write the joystick calibration sketches

### Project TODO:
- Fix the 3d printable tabs
- Figure out a better way to mount the tp4056 that doesnt involve hot glue
- Implement hand bone finger tracking for games that use it
- Implement battery % remaining
- Write support for different controller types (htc)
- Fix the trackpad Y axis not working at all
- NEW: Upgrade Serial to HID
- ~~Write a way to reset controller/headset yaw without having to press F8/F7~~
