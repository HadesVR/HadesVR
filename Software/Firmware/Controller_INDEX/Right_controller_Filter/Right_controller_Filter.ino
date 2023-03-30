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
#define CALPIN              5               //pin to start mag calibration at power on

#define APin                4
#define BPin                3
#define SysPin              5
#define FingerIndexPin      1
#define FingerMiddlePin     6
#define FingerRingPin       7
#define FingerPinkyPin      8
#define TrackpadPin         A0
#define JoyXPin             A1
#define JoyYPin             A2
#define JoyClickPin         2
#define TriggerPin          A3
#define VbatPin             A6

#define BatLevelMax         968             //you need to find all of these values on your own
#define JoyXMin             237             //check on the utils folder for sketches and instructions
#define JoyXMax             935             //that help on getting these values
#define JoyYMin             190             //YOU NEED TO DO THIS FOR BOTH CONTROLLERS
#define JoyYMax             900             //if you use these values without changing them you MAY
#define JoyXDeadZoneMin     490             //get stick drift
#define JoyXDeadZoneMax     620
#define JoyYDeadZoneMin     420
#define JoyYDeadZoneMax     620
//==========================================================================================================


calData calib =
{ false,                   //data valid?
  {0, 0, 0},              //Accel bias
  {0, 0, 0},              //Gyro bias
  {0, 0, 0},              //Mag bias
  {1, 1, 1},              //Mag Scale
};

#define IB_AClick           0x0001
#define IB_ATouch           0x0002
#define IB_BClick           0x0004
#define IB_BTouch           0x0008
#define IB_SYSClick         0x0010
#define IB_ThumbStickClick  0x0020
#define IB_TrackpadTouch    0x0040
#define IB_ThumbStickTouch  0x0080
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
uint64_t Pipe = 0xF0F0F0F0E1LL; //right
//uint64_t Pipe = 0xF0F0F0F0D2LL; //left
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

  pinMode(APin, INPUT_PULLUP);
  pinMode(BPin, INPUT_PULLUP);
  pinMode(SysPin, INPUT_PULLUP);
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
    if (!digitalRead(BPin)) {
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
  data.Data = 0x4B3;

  filter.begin(MadgwickBeta);
  /*
    data.Data |= 0x03;  //non diy index controller identifier
    data.Data |= 0x10;  //controller reports accelerometer values
    data.Data |= 0x20;  //controller does support hand tracking
    //data.Data |= 0x40;  //handtracking type is analog
    data.Data |= 0x80;  //controller color is blue (80 for blue 100 for green 200 for red)
    data.Data |= 0x400; //controller reports battery %
  */
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

  joyTouch = false;
  int btn = 0;
  tracky = analogRead(TrackpadPin);
  if (tracky > 560) {
    trackoutput = 1;
    btn |= IB_TrackpadTouch;
  }
  if (tracky < 300 && tracky > 100) {
    trackoutput = 0;
    btn |= IB_TrackpadTouch;
  }
  if (tracky == 0) {
    trackoutput = -1;
    btn |= IB_TrackpadTouch;
  }
  if (tracky < 550 && tracky > 400) {
    trackoutput = 0;
  }

  axisX = analogRead(JoyXPin);
  axisY = analogRead(JoyYPin);

  if (axisX > JoyXDeadZoneMax || axisX < JoyXDeadZoneMin) {
    if (axisX > JoyXMax) {
      axisX = JoyXMax;
    }
    if (axisX < JoyXMin) {
      axisX = JoyXMin;
    }
    data.axisX = -map(axisX, JoyXMin, JoyXMax, -127, 127);
    btn |= IB_ThumbStickTouch;
    joyTouch = true;
  } else {
    data.axisX = 0;
  }

  if (axisY > JoyYDeadZoneMax || axisY < JoyYDeadZoneMin) {
    if (axisY > JoyYMax) {
      axisY = JoyYMax;
    }
    if (axisY < JoyYMin) {
      axisY = JoyYMin;
    }
    data.axisY = map(axisY, JoyYMin, JoyYMax, -127, 127);
    btn |= IB_ThumbStickTouch;
    joyTouch = true;
  } else {
    data.axisY = 0;
  }


  if (analogRead(TriggerPin) < 1000) {
    data.trigg = map(analogRead(TriggerPin), 1024, 0, 0, 255);
    data.fingerIndex = map(analogRead(TriggerPin), 1024, 0, 0, 255);
  }
  else {
    data.trigg = 0;
    data.fingerIndex = 0;
  }


  if (!digitalRead(APin)) {
    btn |= IB_AClick;
    btn |= IB_ATouch;
  }
  if (!digitalRead(BPin)) {
    btn |= IB_BClick;
    btn |= IB_BTouch;
  }
  if (!digitalRead(SysPin)) {
    btn |= IB_SYSClick;
  }
  if (!digitalRead(JoyClickPin)) {
    btn |= IB_ThumbStickClick;
  }
  //  if (digitalRead(FingerIndexPin)) {
  //    data.fingerIndex = 255;
  //  }
  //  else {
  //    data.fingerIndex = 0;
  //  }
  data.gripForce = 0;
  if (digitalRead(FingerMiddlePin)) {
    data.fingerMiddle = 255;
    data.gripForce += 128;
  }
  else {
    data.fingerMiddle = 0;
  }
  if (digitalRead(FingerRingPin)) {
    data.fingerRing = 255;
     data.gripForce += 64;
  }
  else {
    data.fingerRing = 0;
  }
  if (digitalRead(FingerPinkyPin)) {
    data.fingerPinky = 255;
     data.gripForce += 63;
  }
  else {
    data.fingerPinky = 0;
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

  Serial.print("AX: ");
  Serial.print(IMUAccel.accelX);
  Serial.print(",AY: ");
  Serial.print(IMUAccel.accelY);
  Serial.print(",AZ: ");
  Serial.print(IMUAccel.accelZ);
  Serial.print(",GX: ");
  Serial.print(IMUGyro.gyroX);
  Serial.print(",GY: ");
  Serial.print(IMUGyro.gyroY);
  Serial.print(",GZ: ");
  Serial.println(IMUGyro.gyroZ);
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
