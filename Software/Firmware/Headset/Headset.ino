/*
  Copyright 2022 HadesVR
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
#include "RF24.h"
#include "FastIMU.h"

//==========================================================================================================
//************************************ USER CONFIGURABLE STUFF HERE*****************************************
//==========================================================================================================

#define IMU_ADDRESS     0x69                // You can find it out by using the IMUIdentifier example
BMI055 IMU;                                 // IMU type
#define CALPIN              4               // Pin to start mag calibration at power on.
#define EEPROM_CAL                          // Comment this if your MCU doesn't support EEPROM.
//#define USE_RF                            // Comment this to disalbe RF functionality.
//#define SERIAL_DEBUG                        // Uncomment this to make the serial port spicy.

#define TRANSPORT_TYPE             1        //1 = HID (default), 2 = UART
#define TRANSPORT_SERIAL_PORT      Serial   // Must_not be same as SERIAL_DEBUG (if that is enabled)
#define TRANSPORT_SERIAL_BAUDRATE  230400

//==========================================================================================================
//************************************* Calibration stuff *************************************************
//==========================================================================================================
//eeprom-less mcu stuff, you don't need to touch these if you do the eeprom calibration
calData calib =
{ true,                   //data valid?
  {0, 0, 0},              //Accel bias
  {0, 0, 0},              //Gyro bias
  {0, 0, 0},              //Mag bias
  {1, 1, 1},              //Mag Scale
};
#ifdef EEPROM_CAL
#include <EEPROM.h>
#endif
//==========================================================================================================
//************************************ DATA TRANSPORT LAYER ************************************************
//==========================================================================================================

class DataTransport {
  public:
    virtual ~DataTransport() {}
    virtual void setup() = 0;
    virtual void sendPacket(const void* packet, int len);
};

#if TRANSPORT_TYPE == 1

#include "HID.h"

static const uint8_t USB_HID_Descriptor[] PROGMEM = {

  0x06, 0x03, 0x00,   // USAGE_PAGE (vendor defined)
  0x09, 0x00,         // USAGE (Undefined)
  0xa1, 0x01,         // COLLECTION (Application)
  0x15, 0x00,         //   LOGICAL_MINIMUM (0)
  0x26, 0xff, 0x00,   //   LOGICAL_MAXIMUM (255)
  0x85, 0x01,         //   REPORT_ID (1)
  0x75, 0x08,         //   REPORT_SIZE (16)

  0x95, 0x3f,         //   REPORT_COUNT (1)

  0x09, 0x00,         //   USAGE (Undefined)
  0x81, 0x02,         //   INPUT (Data,Var,Abs) - to the host
  0xc0
};

class HIDTransport : public DataTransport {
  public:
    void setup()
    {
      static HIDSubDescriptor node(USB_HID_Descriptor, sizeof(USB_HID_Descriptor));
      HID().AppendDescriptor(&node);
    }

    void sendPacket(const void* packet, int len)
    {
      HID().SendReport(1, packet, len);
    }
};

#define TransportClass HIDTransport

#elif TRANSPORT_TYPE == 2

#define UART_MAGIC ((uint8_t)0xAA)

class UARTTransport : public DataTransport {
  public:
    UARTTransport() : port(TRANSPORT_SERIAL_PORT) {}

    void setup()
    {
      port.begin(TRANSPORT_SERIAL_BAUDRATE);
      cnt = 0;
    }

    void sendPacket(const void* packet, int len)
    {
      // TODO: Check len < 256
      const uint8_t dataLen = (uint8_t)len;

      // We simply send <magic:1><length:1><data:length> where magic=0xAA, length=len and data=packet
      // Adding a CRC might be a good idea later
      port.write((uint8_t)UART_MAGIC);
      port.write(cnt++);
      port.write(dataLen + 1);
      port.write((uint8_t)0x01); // Dummy report number (not really used)
      port.write((const uint8_t*)packet, dataLen);
    }

  private:
    uint8_t cnt;
    HardwareSerial& port;
};

#define TransportClass UARTTransport

#else
#error "unsupported transport type: "
#endif

static TransportClass transport;

//==========================================================================================================
//************************************* Data packet stuff *************************************************
//==========================================================================================================
struct HMDRAWPacket
{
  uint8_t  PacketID;

  int16_t AccX;
  int16_t AccY;
  int16_t AccZ;

  int16_t GyroX;
  int16_t GyroY;
  int16_t GyroZ;

  int16_t MagX;
  int16_t MagY;
  int16_t MagZ;

  uint16_t HMDData;

  uint8_t Padding[30];

};
struct ControllerPacket
{
  uint8_t PacketID;
  int16_t Ctrl1_QuatW;
  int16_t Ctrl1_QuatX;
  int16_t Ctrl1_QuatY;
  int16_t Ctrl1_QuatZ;
  int16_t Ctrl1_AccelX;
  int16_t Ctrl1_AccelY;
  int16_t Ctrl1_AccelZ;
  uint16_t Ctrl1_Buttons;
  uint8_t Ctrl1_Trigger;
  int8_t Ctrl1_axisX;
  int8_t Ctrl1_axisY;
  int8_t Ctrl1_trackY;
  uint8_t Ctrl1_vBat;
  uint8_t Ctrl1_THUMB;
  uint8_t Ctrl1_INDEX;
  uint8_t Ctrl1_MIDDLE;
  uint8_t Ctrl1_RING;
  uint8_t Ctrl1_PINKY;
  uint8_t Ctrl1_AnalogGrip;
  uint16_t Ctrl1_Data;

  int16_t Ctrl2_QuatW;
  int16_t Ctrl2_QuatX;
  int16_t Ctrl2_QuatY;
  int16_t Ctrl2_QuatZ;
  int16_t Ctrl2_AccelX;
  int16_t Ctrl2_AccelY;
  int16_t Ctrl2_AccelZ;
  uint16_t Ctrl2_Buttons;
  uint8_t Ctrl2_Trigger;
  int8_t Ctrl2_axisX;
  int8_t Ctrl2_axisY;
  int8_t Ctrl2_trackY;
  uint8_t Ctrl2_vBat;
  uint8_t Ctrl2_THUMB;
  uint8_t Ctrl2_INDEX;
  uint8_t Ctrl2_MIDDLE;
  uint8_t Ctrl2_RING;
  uint8_t Ctrl2_PINKY;
  uint8_t Ctrl2_AnalogGrip;
  uint16_t Ctrl2_Data;
};

static HMDRAWPacket HMDRawData;
static ControllerPacket ContData;
//==========================================================================================================
//**************************************** RF Data stuff ***************************************************
//==========================================================================================================
#ifdef USE_RF
const uint64_t rightCtrlPipe = 0xF0F0F0F0E1LL;
const uint64_t leftCtrlPipe = 0xF0F0F0F0D2LL;
const uint64_t trackerPipe = 0xF0F0F0F0C3LL;

RF24 radio(9, 10); // CE, CSN on Blue Pill

bool newCtrlData = false;
#endif
//==========================================================================================================
//**************************************** IMU variables ***************************************************
//==========================================================================================================
AccelData IMUAccel;
GyroData IMUGyro;
MagData IMUMag;
//==========================================================================================================

void setup() {
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  pinMode(CALPIN, INPUT_PULLUP);
  //setup transport
  transport.setup();

#ifdef SERIAL_DEBUG
  Serial.begin(38400);
  while (!Serial) ;
#endif

  //setup IMU
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0)
  {
    debugPrint("IMU ERROR: ");
    debugPrintln(err);
    while (true) ;
  }

#ifdef EEPROM_CAL
  EEPROM.get(210, calib);
  debugPrintln("Loaded Calibration from EEPROM!");
  printCalibration();
#endif
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
  if (!digitalRead(CALPIN)) {                                        //enter calibration mode
    calibrateIMU();
#ifdef EEPROM_CAL
    debugPrintln("Writing values to EEPROM!");
    EEPROM.put(210, calib);
#endif
    delay(3000);
  }

#ifndef EEPROM_CAL
  debugPrintln("Loading calibration values from program memory");
  printCalibration();
#endif

#ifdef USE_RF
  //setup rf
  radio.begin();
  radio.setPayloadSize(40);
  radio.openReadingPipe(3, trackerPipe);
  radio.openReadingPipe(2, leftCtrlPipe);
  radio.openReadingPipe(1, rightCtrlPipe);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  if (!radio.isChipConnected())
  {
    debugPrintln("NRF24L01 Module not detected!");
    while (true) ;
  }
  else
  {
    debugPrintln("NRF24L01 Module up and running!");
  }
#endif
  IMU.init(calib, IMU_ADDRESS);     //Reinitialize with correct calibration values
  HMDRawData.PacketID = 3;
  ContData.PacketID = 2;
  debugPrintln("IMU Initialized.");
}

void loop()
{
#ifdef USE_RF
  uint8_t pipenum;
  if (radio.available(&pipenum)) {                  //thanks SimLeek for this idea!
    if (pipenum == 1) {
      radio.read(&ContData.Ctrl1_QuatW, 29);        //receive right controller data
      newCtrlData = true;
    }
    if (pipenum == 2) {
      radio.read(&ContData.Ctrl2_QuatW, 29);        //receive left controller data
      newCtrlData = true;
    }
    if (pipenum == 3) {
      radio.read(&HMDRawData.tracker1_QuatW, 27);   //recive all 3 trackers' data
    }
  }
  if (newCtrlData) {
    transport.sendPacket(&ContData, 63);
    newCtrlData = false;
  }
#endif

  IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);

  if (IMU.hasMagnetometer()) {
    IMU.getMag(&IMUMag);
    HMDRawData.MagX = (short)(IMUMag.magX * 5);
    HMDRawData.MagY = (short)(IMUMag.magY * 5);
    HMDRawData.MagZ = (short)(IMUMag.magZ * 5);
  }
  else {
    HMDRawData.MagX = (short)(0);
    HMDRawData.MagY = (short)(0);
    HMDRawData.MagZ = (short)(0);
  }

  HMDRawData.AccX = (short)(IMUAccel.accelX * 2048);
  HMDRawData.AccY = (short)(IMUAccel.accelY * 2048);
  HMDRawData.AccZ = (short)(IMUAccel.accelZ * 2048);

  HMDRawData.GyroX = (short)(IMUGyro.gyroX * 16);
  HMDRawData.GyroY = (short)(IMUGyro.gyroY * 16);
  HMDRawData.GyroZ = (short)(IMUGyro.gyroZ * 16);

  transport.sendPacket(&HMDRawData, 63);
}

void calibrateIMU()
{
  if (IMU.hasMagnetometer()) {
    Serial.println("Magnetic calibration mode.");
    Serial.println("Move IMU in figure 8 until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration complete!");
    delay(1000);
  }
  Serial.println("Accelerometer and gyroscope calibration mode.");
  Serial.println("Keep IMU completely still on flat and level surface.");
  delay(8000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Accel & Gyro calibration complete!");
  calib.valid = true;
  printCalibration();
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

void debugPrint(String arg) {
#ifdef SERIAL_DEBUG
  Serial.print(arg);
#endif
}
void debugPrint(int arg) {
#ifdef SERIAL_DEBUG
  Serial.print(arg);
#endif
}
void debugPrintln(String arg) {
#ifdef SERIAL_DEBUG
  Serial.println(arg);
#endif
}
void debugPrintln(int arg) {
#ifdef SERIAL_DEBUG
  Serial.println(arg);
#endif
}
