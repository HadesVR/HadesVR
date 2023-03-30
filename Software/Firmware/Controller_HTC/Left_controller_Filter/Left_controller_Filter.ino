/*
  Copyright 2023 HadesVR
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,INCLUDING BUT NOT LIMITED TO
  THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "RF24.h"
#include "FastIMU.h"
#include "Madgwick.h"

//==========================================================================================================
//************************************ USER CONFIGURABLE STUFF HERE*****************************************
//==========================================================================================================

//#define SERIAL_DEBUG
#define IMU_ADDRESS     0x68                // You can find it out by using the IMUIdentifier example
MPU9250 IMU;                                // IMU type
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
//==========================================================================================================


calData calib =
{ false,                   //data valid?
  {0, 0, 0},              //Accel bias
  {0, 0, 0},              //Gyro bias
  {0, 0, 0},              //Mag bias
  {1, 1, 1},              //Mag Scale
};

#define HTC_SysClick        0x0001
#define HTC_MenuClick       0x0002
#define HTC_ThumbstickClick 0x0004
#define HTC_GripClick       0x0008
#define HTC_ThumbstickTouch 0x0010
//==========================================================================================================
//************************************* Data packet stuff *************************************************
//==========================================================================================================
struct ctrlData {
  int16_t qW;
  int16_t qX;
  int16_t qY;
  int16_t qZ;
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  uint16_t BTN;
  uint8_t  trigg;
  int8_t  axisX;
  int8_t  axisY;
  int8_t  trackY;
  uint8_t  vBAT;
  uint8_t  fingerThumb;
  uint8_t  fingerIndex;
  uint8_t  fingerMiddle;
  uint8_t  fingerRing;
  uint8_t  fingerPinky;
  uint8_t  gripForce;
  uint16_t Data;
};
ctrlData data;
//==========================================================================================================
//**************************************** Analog inputs ***************************************************
//==========================================================================================================
int tracky;
int trackoutput;
int axisX;
int axisY;
bool joyTouch = false;
//==========================================================================================================
//**************************************** RF Data stuff ***************************************************
//==========================================================================================================
RF24 radio(9, 10);
//uint64_t Pipe = 0xF0F0F0F0E1LL; //right
uint64_t Pipe = 0xF0F0F0F0D2LL; //left
//==========================================================================================================
//**************************************** IMU variables ***************************************************
//==========================================================================================================
AccelData IMUAccel;
GyroData IMUGyro;
MagData IMUMag;
//==========================================================================================================
//************************************** Filter variables **************************************************
//==========================================================================================================
Madgwick filter;
static const float MadgwickBeta = 0.16f;
float rot = 0.f;
//==========================================================================================================

void setup() {
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  pinMode(SysPin, INPUT_PULLUP);
  pinMode(MenuPin, INPUT_PULLUP);
  pinMode(GripPin, INPUT_PULLUP);
  pinMode(JoyClickPin, INPUT_PULLUP);
  pinMode(TriggerPin, INPUT_PULLUP);

#ifdef SERIAL_DEBUG
  Serial.begin(38400);
#endif

  radio.begin();
  radio.setPayloadSize(32);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.openWritingPipe(Pipe);
  radio.startListening();
  radio.setAutoAck(false);

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0)
  {
    Serial.print("IMU ERROR: ");
    Serial.println(err);
    while (true);
  }
  if (!radio.isChipConnected())
  {
    Serial.println("NRF24L01 Module not detected!");
    while (true);
  }
  else
  {
    Serial.println("NRF24L01 Module up and running!");
  }

  EEPROM.get(210, calib);

  bool calDone = !calib.valid;                             //check if calibration values are on flash
  while (calDone)
  {
    delay(1000);
    Serial.print("Calibration not done!");
    if (!digitalRead(CALPIN))
    {
      calDone = false;
    }
  }
  if (!digitalRead(CALPIN)) {
    if (!digitalRead(MenuPin)) {
      Serial.println("Accelerometer and gyroscope calibration mode.");
      Serial.println("Keep IMU completely still on flat and level surface.");
      delay(8000);
      IMU.calibrateAccelGyro(&calib);
      Serial.println("Accel & Gyro calibration complete!");
      calib.valid = true;
    }
    else {
      if (IMU.hasMagnetometer()) {
        Serial.println("Magnetic calibration mode.");
        Serial.println("Move IMU in figure 8 until done.");
        delay(3000);
        IMU.calibrateMag(&calib);
        Serial.println("Magnetic calibration complete!");
        delay(1000);
      }
    }
    printCalibration();
    Serial.println("Writing values to EEPROM!");
    EEPROM.put(210, calib);
    delay(3000);
  }

  //initialize controller data.
  data.qW = 1;
  data.qX = 0;
  data.qY = 0;
  data.qZ = 0;
  data.BTN = 0;
  data.trigg = 0;
  data.axisX = 0;
  data.axisY = 0;
  data.trackY = 0;
  data.vBAT = 0;
  data.fingerThumb = 0;
  data.fingerIndex = 0;
  data.fingerMiddle = 0;
  data.fingerRing = 0;
  data.fingerPinky = 0;
  data.Data = 0xFF;

  filter.begin(MadgwickBeta);

  err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0)
  {
    Serial.print("IMU ERROR: ");
    Serial.println(err);
    while (true);
  }
}

void loop() {

  IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);

  if (IMU.hasMagnetometer()) {
    IMU.getMag(&IMUMag);
    filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUMag.magX, IMUMag.magY, IMUMag.magZ);
  }
  else {
    filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
  }

  rot += (abs(IMUGyro.gyroX) + abs(IMUGyro.gyroY) + abs(IMUGyro.gyroZ));
  if (rot > 64000.f) rot = 64000.f;
  rot *= 0.97f;
  filter.changeBeta(rot * (1.5 - 0.1) / 64000 + 0.1);

  int btn = 0;

  axisX = analogRead(JoyXPin);
  axisY = analogRead(JoyYPin);

  if (axisX > JoyXDeadZoneMax || axisX < JoyXDeadZoneMin) {
    data.axisX = -map(axisX, JoyXMin, JoyXMax, -127, 127);
  } else {
    data.axisX = 0;
  }

  if (axisY > JoyYDeadZoneMax || axisY < JoyYDeadZoneMin) {
    data.axisY = map(axisY, JoyYMin, JoyYMax, -127, 127);
    btn |= HTC_ThumbstickTouch;
  } else {
    data.axisY = 0;
  }

  data.trigg = (map(analogRead(TriggerPin), 1024, 0, 0, 255));

  if (!digitalRead(SysPin)) {
    btn |= HTC_SysClick;
  }
  if (!digitalRead(MenuPin)) {
    btn |= HTC_MenuClick;
  }
  if (!digitalRead(JoyClickPin)) {
    btn |= HTC_ThumbstickClick;
  }
  if (digitalRead(GripPin)) {
    btn |= HTC_GripClick;
  }


  data.BTN = btn;
  data.trackY = (trackoutput * 127);
  data.vBAT = (map(analogRead(VbatPin), 787, BatLevelMax, 0, 255));
  data.qW = (int16_t)(filter.getQuatW() * 32767.f);
  data.qX = (int16_t)(filter.getQuatY() * 32767.f);
  data.qY = (int16_t)(filter.getQuatZ() * 32767.f);
  data.qZ = (int16_t)(filter.getQuatX() * 32767.f);
  data.accX = (short)(IMUAccel.accelX * 2048);
  data.accY = (short)(IMUAccel.accelY * 2048);
  data.accZ = (short)(IMUAccel.accelZ * 2048);

  radio.stopListening();
  radio.write(&data, sizeof(ctrlData));
  radio.startListening();
}
void printCalibration()
{
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(5000);
}
