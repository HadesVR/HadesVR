The headset will not only be in charge of showing you the image and relaying back rotation data from it's internal IMU, but it will also be in charge of receiving the RF Controller data and sending it back to the computer. Another thing that can be integrated into the headset is an USB Sound Card, that way it'll also have an integrated Microphone and a headphone jack for you to plug your headphones in. Though it's not mandatory.

As of right now, the communication between Headset and PC is done using a Serial port but this is subject to change in the future in favour of USB HID.

# Hardware Needed

## Shell
For the shell you can use any "Phone VR" enclosure, they come in very different qualities but all of them should do the trick (even the carboard ones). Keep in mind the enclosure will dictate your max fov and that you can always modify the shell to allow for a wider fov.

![1](img/Headset/1.png)

You could also 3d print the shell, like the guys over at [Relativty](https://github.com/relativty/Relativty#building-the-mechanical-parts) did. Keep in mind you'll also need to source your own lenses with the correct focal length and head strap.

The screen mount depends on what screen you use and you'll have to design your own sice not all mounts fit all headsets/screen combinations.

## Electronics
I plan on eventually making a custom pcb for the headset that includes a 2 port USB hub, an audio interface for headphones and microphone integration, an STM32 as the main microcontroller and maybe a couple other goodies like an attempt at backlight strobing to reduce screen blur.

Electronics needed as of *right now* is anything that can talk over serial, though this is subject to change once USB HID communication is implemented. So while you technically *could* use an Arduino Nano, I'm going to recommend against it, since once the move to USB HID is done you won't be able to update your driver.

I suggest sticking to stuff that supports the [Arduino HID library](https://www.arduino.cc/en/Reference/HID). An ideal candidate is the Arduino Pro Micro since it's based on the Atmega32u4 and it's fairly small.

Also needed are:
* An MPU9250 to gather rotation data (though you could use any IMU that's fast enough and can output rotation data to quaternions if you modify the code a bit to allow for that)

* An NRF24L01 to receive the controller data from both controllers

* A High brightness LED that's a different colour from the ones used on the controllers (I suggest green) and a resistor for that LED.

## Display(s)
# Circuit schematic
# Uploading the firmware
# Setting correct COMPort