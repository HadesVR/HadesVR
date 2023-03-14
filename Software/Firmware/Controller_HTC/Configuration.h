#pragma once
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

//
// Global configuration for both controllers
//

//#define SERIAL_DEBUG
#define IMU_ADDRESS     0x68                // You can find it out by using the IMUIdentifier example
#define IMU_TYPE        MPU9250             // IMU type

//
// Configuration for individual controllers
//

#ifdef RIGHT_CONTROLLER

/* Right Controller */

#define CALPIN              4               //pin to start mag calibration at power on

#define SysPin              4
#define MenuPin             3
#define GripPin             6
#define JoyXPin             A1
#define JoyYPin             A2
#define JoyClickPin         2
#define TriggerPin          A3
#define VbatPin             A0

#define BatLevelMax         968             //you need to find all of these values on your own
#define JoyXMin             237             //check on the utils folder for sketches and instructions
#define JoyXMax             935             //that help on getting these values
#define JoyYMin             190             //YOU NEED TO DO THIS FOR BOTH CONTROLLERS
#define JoyYMax             900             //if you use these values without changing them you MAY
#define JoyXDeadZoneMin     515             //get stick drift
#define JoyXDeadZoneMax     590
#define JoyYDeadZoneMin     440
#define JoyYDeadZoneMax     600

#else

/* Left Controller */

#define CALPIN              4               //pin to start mag calibration at power on

#define SysPin              4
#define MenuPin             3
#define GripPin             6
#define JoyXPin             A1
#define JoyYPin             A2
#define JoyClickPin         2
#define TriggerPin          A3
#define VbatPin             A0

#define BatLevelMax         968             //you need to find all of these values on your own
#define JoyXMin             200             //check on the utils folder for sketches and instructions
#define JoyXMax             900             //that help on getting these values
#define JoyYMin             150             //YOU NEED TO DO THIS FOR BOTH CONTROLLERS
#define JoyYMax             870             //if you use these values without changing them you MAY
#define JoyXDeadZoneMin     515             //get stick drift
#define JoyXDeadZoneMax     590
#define JoyYDeadZoneMin     430
#define JoyYDeadZoneMax     560

#endif

#endif /* CONFIGURATION_H */