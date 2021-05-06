/*Copyright 2021 LiquidCGS
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,INCLUDING BUT NOT LIMITED TO
  THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  =============================================================================
  ===========================MPU-9250 ARDUINO AHRS=============================
  =============================================================================
  Part of the code is taken straight out of Kriswiner's MPU9250 AHRS code.
  I've tested this on a 16mhz UNO and it runs fine, I have yet to test it on 12mhz and 8mhz.

  you'll need to at least calibrate the magnetometer to use this, you can do that by uploading the calibration sketch and following the instructions in the serial monitor.
  Calibration settings are stored in the 328p's integrated EEPROM, if your MCU doesn't support EEPROM you'll have to comment the line USE_EEPROM_CALIBRATION and input
  the calibration values manually.

  Anyways, check out Kriswiner's original MPU9250 code over here: https://github.com/kriswiner/MPU9250
  You can get the Madgwick filter from here: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
  ~LiquidCGS
*/

#include <SPI.h>
#include "RF24.h"
#include "HID.h"
#include <Wire.h>
#include "RegisterMap.h"
#include "Madgwick.h"

//==========================================================================================================
//*************** CALIBRATION VALUES HERE (ONLY USE IF NOT USING EEPROM CALIBRATION)************************
//==========================================================================================================

float magBias[3] {0, 0, 0}; // bias corrections for mag
float magScale[3] {1, 1, 1}; // estimated hard iron corrections for mag
float gyroBias[3] {0, 0, 0}; // bias corrections for gyro
float accelBias[3] {0, 0, 0}; // bias corrections for accel

//==========================================================================================================
//************************************ USER CONFIGURABLE STUFF HERE*****************************************
//==========================================================================================================

#define betaDef         0.02f       // how hard do you want to correct for drift with the magnetometer, leave as is for headset.
#define USE_EEPROM_CALIBRATION      //comment this to input calibration values manually.
#define MPU9250_ADDRESS 0x68 //ADO 0

//==========================================================================================================

#ifdef USE_EEPROM_CALIBRATION
#include <EEPROM.h>
#endif

Madgwick filter;

float magCalibration[3]; // factory mag calibration

struct Calibration {
  int calDone;
  float magBias[3];
  float magScale[3];
  float gyroBias[3];
  float accelBias[3]; // bias corrections
};

Calibration cal;

float x, y, z;
static float ax, ay, az, gx, gy, gz, mx, my, mz;

enum class AFS { A2G, A4G, A8G, A16G };
enum class GFS { G250DPS, G500DPS, G1000DPS, G2000DPS };
enum class MFS { M14BITS, M16BITS }; // 0.6mG, 0.15mG per LSB

static float aRes;
static float gRes;
static float mRes;

AFS AFSSEL = AFS::A16G;
GFS GFSSEL = GFS::G2000DPS;
MFS MFSSEL = MFS::M16BITS;
#define Mmode 0x06                    // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

static const uint8_t _hidReportDescriptor[] PROGMEM = {

  0x06, 0x03, 0x00,         // USAGE_PAGE (vendor defined)
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

RF24 radio(9, 10);

const uint64_t rightCtrlPipe = 0xF0F0F0F0E1LL;
const uint64_t leftCtrlPipe = 0xF0F0F0F0D2LL;

struct ctrlData {
  float qW;
  float qX;
  float qY;
  float qZ;
  uint32_t BTN;
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
};

ctrlData Ctrl1Data, Ctrl2Data;

struct HMDPacket
{
  uint8_t  PacketID;
  float HMDQuatW;
  float HMDQuatX;
  float HMDQuatY;
  float HMDQuatZ;

  uint16_t tracker1_QuatW;
  uint16_t tracker1_QuatX;
  uint16_t tracker1_QuatY;
  uint16_t tracker1_QuatZ;

  uint16_t tracker2_QuatW;
  uint16_t tracker2_QuatX;
  uint16_t tracker2_QuatY;
  uint16_t tracker2_QuatZ;

  uint16_t tracker3_QuatW;
  uint16_t tracker3_QuatX;
  uint16_t tracker3_QuatY;
  uint16_t tracker3_QuatZ;

  uint8_t tracker1_vBat;
  uint8_t tracker2_vBat;
  uint8_t tracker3_vBat;
};

struct ControllerPacket
{
  uint8_t PacketID;
  float Ctrl1_QuatW;
  float Ctrl1_QuatX;
  float Ctrl1_QuatY;
  float Ctrl1_QuatZ;
  uint32_t Ctrl1_Buttons;
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

  float Ctrl2_QuatW;
  float Ctrl2_QuatX;
  float Ctrl2_QuatY;
  float Ctrl2_QuatZ;
  uint32_t Ctrl2_Buttons;
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
};

static HMDPacket HMDData;
static ControllerPacket ContData;
bool newContData = false;

void setup() {

  static HIDSubDescriptor node (_hidReportDescriptor, sizeof(_hidReportDescriptor));
  HID().AppendDescriptor(&node);

  HMDData.PacketID = 1;
  ContData.PacketID = 2;

  Wire.begin();

  aRes = getAres();
  gRes = getGres();
  mRes = getMres();

  if (readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250) == MPU9250_WHOAMI_DEFAULT_VALUE)
  {

    Serial.println("MPU9250 is online");
    initMPU();

    if (readByte(AK8963_ADDRESS, AK8963_WHO_AM_I) == AK8963_WHOAMI_DEFAULT_VALUE)
    {
      Serial.println("AK8963 is online");
      initAK8963(magCalibration);
    }
    else
    {
      Serial.print("Could not connect to AK8963: 0x");
      Serial.println(readByte(AK8963_ADDRESS, AK8963_WHO_AM_I), HEX);
    }
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250), HEX);
  }

#ifdef USE_EEPROM_CALIBRATION

  EEPROM.get(50, cal);

  if (cal.calDone != 99) Serial.println("IMU not calibrated. Please run calibration first!");
  while (cal.calDone != 99) {
    ;
  }

#else
  cal.magBias[0] = magBias[0];
  cal.magBias[1] = magBias[1];
  cal.magBias[2] = magBias[2];
  cal.magScale[0] = magScale[0];
  cal.magScale[1] = magScale[1];
  cal.magScale[2] = magScale[2];
  cal.gyroBias[0] = gyroBias[0];
  cal.gyroBias[1] = gyroBias[1];
  cal.gyroBias[2] = gyroBias[2];
  cal.accelBias[0] = accelBias[0];
  cal.accelBias[1] = accelBias[1];
  cal.accelBias[2] = accelBias[2];
#endif

  filter.begin();

  radio.begin();
  radio.setPayloadSize(40);
  radio.openReadingPipe(2, leftCtrlPipe);
  radio.openReadingPipe(1, rightCtrlPipe);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

}


void loop() {

  uint8_t pipenum;

  if (dataAvailable()) {
    updateAccelGyro();
    updateMag();
  }

  filter.update(gx, gy, gz, ax, ay, az, my, mx, mz);


  HMDData.HMDQuatW = filter.getQuatW();
  HMDData.HMDQuatX = filter.getQuatY();
  HMDData.HMDQuatY = filter.getQuatZ();
  HMDData.HMDQuatZ = filter.getQuatX();

  HID().SendReport(1, &HMDData, 63);

  if (radio.available(&pipenum)) {                  //thanks SimLeek for this idea!
    if (pipenum == 1) {
      radio.read(&Ctrl1Data, sizeof(ctrlData));
      newContData = true;
    }
    if (pipenum == 2) {
      radio.read(&Ctrl2Data, sizeof(ctrlData));
      newContData = true;
    }
  }

  if (newContData) {

    ContData.Ctrl1_QuatW    = Ctrl1Data.qW;         //there's more efficient ways of doing this...
    ContData.Ctrl1_QuatX    = Ctrl1Data.qY;
    ContData.Ctrl1_QuatY    = Ctrl1Data.qZ;
    ContData.Ctrl1_QuatZ    = Ctrl1Data.qX;
    ContData.Ctrl1_Buttons  = Ctrl1Data.BTN;
    ContData.Ctrl1_Trigger  = Ctrl1Data.trigg;
    ContData.Ctrl1_axisX    = Ctrl1Data.axisX;
    ContData.Ctrl1_axisY    = Ctrl1Data.axisY;
    ContData.Ctrl1_trackY   = Ctrl1Data.trackY;
    ContData.Ctrl1_vBat     = Ctrl1Data.vBAT;

    ContData.Ctrl1_THUMB    = Ctrl1Data.fingerThumb;
    ContData.Ctrl1_INDEX    = Ctrl1Data.fingerIndex;
    ContData.Ctrl1_MIDDLE   = Ctrl1Data.fingerMiddle;
    ContData.Ctrl1_RING     = Ctrl1Data.fingerRing;
    ContData.Ctrl1_PINKY    = Ctrl1Data.fingerPinky;

    ContData.Ctrl2_QuatW    = Ctrl2Data.qW;
    ContData.Ctrl2_QuatX    = Ctrl2Data.qY;
    ContData.Ctrl2_QuatY    = Ctrl2Data.qZ;
    ContData.Ctrl2_QuatZ    = Ctrl2Data.qX;
    ContData.Ctrl2_Buttons  = Ctrl2Data.BTN;
    ContData.Ctrl2_Trigger  = Ctrl2Data.trigg;
    ContData.Ctrl2_axisX    = Ctrl2Data.axisX;
    ContData.Ctrl2_axisY    = Ctrl2Data.axisY;
    ContData.Ctrl2_trackY   = Ctrl2Data.trackY;
    ContData.Ctrl2_vBat     = Ctrl2Data.vBAT;

    ContData.Ctrl2_THUMB    = Ctrl2Data.fingerThumb;
    ContData.Ctrl2_INDEX    = Ctrl2Data.fingerIndex;
    ContData.Ctrl2_MIDDLE   = Ctrl2Data.fingerMiddle;
    ContData.Ctrl2_RING     = Ctrl2Data.fingerRing;
    ContData.Ctrl2_PINKY    = Ctrl2Data.fingerPinky;

    HID().SendReport(1, &ContData, 63);

    newContData = false;

  }

}

void initMPU()
{
  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, MPU_CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | (uint8_t)GFSSEL << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | (uint8_t)AFSSEL << 3; // Set full scale range for the accelerometer
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}

void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, (uint8_t)MFSSEL << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);

  //  Serial.println("Calibration values: ");
  //  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(destination[0], 2);
  //  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(destination[1], 2);
  //  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(destination[2], 2);
  //  Serial.print("X-Axis sensitivity offset value "); Serial.println(cal.magBias[0], 2);
  //  Serial.print("Y-Axis sensitivity offset value "); Serial.println(cal.magBias[1], 2);
  //  Serial.print("Z-Axis sensitivity offset value "); Serial.println(cal.magBias[2], 2);
}

void updateAccelGyro()
{
  int16_t MPU9250Data[7];                                       // used to read all 14 bytes at once from the MPU9250 accel/gyro
  uint8_t rawData[14];                                          // x/y/z accel register data stored here

  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);    // Read the 14 raw data registers into data array

  MPU9250Data[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;    // Turn the MSB and LSB into a signed 16-bit value
  MPU9250Data[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  MPU9250Data[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  MPU9250Data[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  MPU9250Data[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  MPU9250Data[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  MPU9250Data[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;

  // Now we'll calculate the accleration value into actual g's
  ax = (float)MPU9250Data[0] * aRes - cal.accelBias[0];              // get actual g value, this depends on scale being set
  ay = (float)MPU9250Data[1] * aRes - cal.accelBias[1];
  az = (float)MPU9250Data[2] * aRes - cal.accelBias[2];

  // Calculate the gyro value into actual degrees per second
  gx = (float)MPU9250Data[4] * gRes - cal.gyroBias[0];               // get actual gyro value, this depends on scale being set
  gy = (float)MPU9250Data[5] * gRes - cal.gyroBias[1];
  gz = (float)MPU9250Data[6] * gRes - cal.gyroBias[2];
}

void updateMag()
{
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) {             // wait for magnetometer data ready bit to be set
    int16_t magCount[3] = {0, 0, 0};                             // Stores the 16-bit signed magnetometer sensor output
    uint8_t rawData[7];                                          // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);    // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6];                                      // End data read by reading ST2 register
    if (!(c & 0x08)) {                                           // Check if magnetic sensor overflow set, if not then report data
      magCount[0] = ((int16_t)rawData[1] << 8) | rawData[0];     // Turn the MSB and LSB into a signed 16-bit value
      magCount[1] = ((int16_t)rawData[3] << 8) | rawData[2];     // Data stored as little Endian
      magCount[2] = ((int16_t)rawData[5] << 8) | rawData[4];
    }

    // getMres();
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections

    mx = (float)(magCount[0] * mRes * magCalibration[0] - cal.magBias[0]) * cal.magScale[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)(magCount[1] * mRes * magCalibration[1] - cal.magBias[1]) * cal.magScale[1];
    mz = (float)(magCount[2] * mRes * magCalibration[2] - cal.magBias[2]) * cal.magScale[2];
    //
    //    // Apply mag soft iron error compensation
    //    mx = x * cal.mag_softiron_matrix[0][0] + y * cal.mag_softiron_matrix[0][1] + z * cal.mag_softiron_matrix[0][2];
    //    my = x * cal.mag_softiron_matrix[1][0] + y * cal.mag_softiron_matrix[1][1] + z * cal.mag_softiron_matrix[1][2];
    //    mz = x * cal.mag_softiron_matrix[2][0] + y * cal.mag_softiron_matrix[2][1] + z * cal.mag_softiron_matrix[2][2];
  }
}

float getAres()
{
  switch (AFSSEL)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS::A2G:  return 2.0 / 32768.0;
    case AFS::A4G:  return 4.0 / 32768.0;
    case AFS::A8G:  return 8.0 / 32768.0;
    case AFS::A16G: return 16.0 / 32768.0;
  }
}

float getGres()
{
  switch (GFSSEL)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS::G250DPS:  return 250.0 / 32768.0;
    case GFS::G500DPS:  return 500.0 / 32768.0;
    case GFS::G1000DPS: return 1000.0 / 32768.0;
    case GFS::G2000DPS: return 2000.0 / 32768.0;
  }
}

float getMres()
{
  switch (MFSSEL)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    // Proper scale to return milliGauss
    case MFS::M14BITS: return 10. * 4912. / 8190.0;
    case MFS::M16BITS: return 10. * 4912. / 32760.0;
  }
}

bool dataAvailable()
{
  return (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01);
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}
