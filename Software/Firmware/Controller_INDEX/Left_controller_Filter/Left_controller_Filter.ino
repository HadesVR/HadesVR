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
#include <EEPROM.h>
#include "RF24.h"
#include "RegisterMap.h"
#include "Madgwick.h"

//==========================================================================================================
//************************************ USER CONFIGURABLE STUFF HERE*****************************************
//==========================================================================================================

//#define SERIAL_DEBUG
#define USE_MAG                           //use magnetometer (will slow down the filter!)
#define MPU9250_ADDRESS     0x68            //ADO 0
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
#define JoyXMin             200             //check on the utils folder for sketches and instructions
#define JoyXMax             900             //that help on getting these values
#define JoyYMin             150             //YOU NEED TO DO THIS FOR BOTH CONTROLLERS
#define JoyYMax             870             //if you use these values without changing them you MAY
#define JoyXDeadZoneMin     515             //get stick drift
#define JoyXDeadZoneMax     590
#define JoyYDeadZoneMin     430
#define JoyYDeadZoneMax     560

//uint64_t Pipe = 0xF0F0F0F0E1LL; //right
uint64_t Pipe = 0xF0F0F0F0D2LL; //left
//==========================================================================================================


Madgwick filter;
static const float MadgwickBeta = 0.16f;
float magCalibration[3]; // factory mag calibration

struct Calibration {
  int calDone;
  float magBias[3];
  float magScale[3];
  float gyroBias[3]; // bias corrections
  float accelBias[3]; // bias corrections
};
bool calDone;
Calibration cal;

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

#define IB_AClick           0x0001
#define IB_ATouch           0x0002
#define IB_BClick           0x0004
#define IB_BTouch           0x0008
#define IB_SYSClick         0x0010
#define IB_ThumbStickClick  0x0020
#define IB_TrackpadTouch    0x0040
#define IB_ThumbStickTouch  0x0080

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
  uint16_t Data;
};
ctrlData data;

int tracky;
int trackoutput;
int axisX;
int axisY;
bool joyTouch = false;

RF24 radio(9, 10); // CE, CSN on Blue Pill

void setup() {

  pinMode(APin, INPUT_PULLUP);
  pinMode(BPin, INPUT_PULLUP);
  pinMode(SysPin, INPUT_PULLUP);
  pinMode(JoyClickPin, INPUT_PULLUP);
  pinMode(TriggerPin, INPUT_PULLUP);

  aRes = getAres();
  gRes = getGres();
  mRes = getMres();

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

  Wire.begin();

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
    while (true) {
    }
  }
  if (!radio.isChipConnected())
  {
    Serial.println("NRF24L01 Module not detected!");
    while (true)
    {
    }
  }
  else
  {
    Serial.println("NRF24L01 Module up and running!");
  }

  EEPROM.get(0, cal);

  calDone = (cal.calDone != 99);                                  //check if calibration values are on flash
  while (calDone) {
    delay(200);
    if (!digitalRead(CALPIN)) {
      calDone = false;
    }
  }

  if (!digitalRead(CALPIN)) {                                        //enter calibration mode
    if (!digitalRead(BPin)) {
      Serial.println("Accelerometer calibration mode.");
      delay(5000);
      calibrateMPU9250(cal.gyroBias, cal.accelBias);
      Serial.println("Done!");
      Serial.print("gyroBias: "); Serial.print(cal.gyroBias[0], 7); Serial.print(","); Serial.print(cal.gyroBias[1], 7); Serial.print(","); Serial.println(cal.gyroBias[2], 7);
      Serial.print("accelBias: "); Serial.print(cal.accelBias[0], 7); Serial.print(","); Serial.print(cal.accelBias[1], 7); Serial.print(","); Serial.println(cal.accelBias[2], 7);
      delay(1000);
    }
    else {
      Serial.println("Magnetic calibration mode.");
      delay(1000);
      magcalMPU9250(cal.magBias, cal.magScale);
      Serial.print("magBias: "); Serial.print(cal.magBias[0], 7); Serial.print(","); Serial.print(cal.magBias[1], 7); Serial.print(","); Serial.println(cal.magBias[2], 7);
      Serial.print("magScale: "); Serial.print(cal.magScale[0], 7); Serial.print(","); Serial.print(cal.magScale[1], 7); Serial.print(","); Serial.println(cal.magScale[2], 7);
    }
    cal.calDone = 99;
    Serial.println("Writting calibration values to EEPROM!");
    EEPROM.put(0, cal);
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

}

void loop() {

  updateMag();
  if (dataAvailable()) {
    updateAccelGyro();
  }
#ifdef USE_MAG
  filter.update(gx, gy, gz, ax, ay, az, my, mx, -mz);
#else
  filter.updateIMU(gx, gy, gz, ax, ay, az);
#endif

  float acc = abs(ax + ay + az);
  if (acc < 1.25f) {
    filter.changeBeta(0.05);
  }
  else if (acc < 2.0) {
    filter.changeBeta(0.1);
  }
  else if (acc < 2.5) {
    filter.changeBeta(0.15);
  }
  else if (acc < 5) {
    filter.changeBeta(0.3);
  }
  else if (acc < 10) {
    filter.changeBeta(0.6);
  }

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
  if (digitalRead(FingerMiddlePin)) {
    data.fingerMiddle = 255;
  }
  else {
    data.fingerMiddle = 0;
  }
  if (digitalRead(FingerRingPin)) {
    data.fingerRing = 255;
  }
  else {
    data.fingerRing = 0;
  }
  if (digitalRead(FingerPinkyPin)) {
    data.fingerPinky = 255;
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
  data.accX = (short)(ax * 2048);
  data.accY = (short)(ay * 2048);
  data.accZ = (short)(az * 2048);

#ifdef SERIAL_DEBUG
  Serial.print("qW: ");
  Serial.print(filter.getQuatW());

  Serial.print(" qX: ");
  Serial.print(filter.getQuatX());

  Serial.print(" qY: ");
  Serial.print(filter.getQuatY());

  Serial.print(" qZ: ");
  Serial.println(filter.getQuatZ());
#endif

  radio.stopListening();
  radio.write(&data, sizeof(ctrlData));
  radio.startListening();
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
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Use a 200 Hz rate; a rate consistent with the filter update rate
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

    // Apply mag soft iron error compensation
    mx = (float)(magCount[0] * mRes * magCalibration[0] - cal.magBias[0]) * cal.magScale[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)(magCount[1] * mRes * magCalibration[1] - cal.magBias[1]) * cal.magScale[1];
    mz = (float)(magCount[2] * mRes * magCalibration[2] - cal.magBias[2]) * cal.magScale[2];
  }
  else {
    mx = 0;
    my = 0;
    mz = 0;
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

void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, MPU_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  // int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  // readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  // accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  // readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  // accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  // readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  // accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  // uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  // uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  // for(ii = 0; ii < 3; ii++) {
  //     if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  // }

  // // Construct total accelerometer bias, including calculated average accelerometer bias from above
  // accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  // accel_bias_reg[1] -= (accel_bias[1] / 8);
  // accel_bias_reg[2] -= (accel_bias[2] / 8);

  // data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  // data[1] = (accel_bias_reg[0])      & 0xFF;
  // data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  // data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  // data[3] = (accel_bias_reg[1])      & 0xFF;
  // data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  // data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  // data[5] = (accel_bias_reg[2])      & 0xFF;
  // data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  // writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  // writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  // writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  // writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  // writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  // writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;

  Serial.println("MPU9250 bias");
  Serial.println(" x   y   z  ");
  Serial.print((int)(1000 * cal.accelBias[0])); Serial.print(" ");
  Serial.print((int)(1000 * cal.accelBias[1])); Serial.print(" ");
  Serial.print((int)(1000 * cal.accelBias[2])); Serial.print(" ");
  Serial.println("mg");
  Serial.print(cal.gyroBias[0], 1); Serial.print(" ");
  Serial.print(cal.gyroBias[1], 1); Serial.print(" ");
  Serial.print(cal.gyroBias[2], 1); Serial.print(" ");
  Serial.println("o/s");

  delay(100);

  initMPU();

  delay(1000);
}

void magcalMPU9250(float * dest1, float * dest2)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = { -32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);

  // shoot for ~fifteen seconds of mag data
  if      (Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
  else if (Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms

  for (ii = 0; ii < sample_count; ii++)
  {
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
      readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
      uint8_t c = rawData[6]; // End data read by reading ST2 register
      if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
        mag_temp[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
        mag_temp[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
        mag_temp[2] = ((int16_t)rawData[5] << 8) | rawData[4];
      }
    }
    for (int jj = 0; jj < 3; jj++)
    {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if (Mmode == 0x02) delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
    if (Mmode == 0x06) delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
  }

  Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * mRes * magCalibration[0]; // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * mRes * magCalibration[1];
  dest1[2] = (float) mag_bias[2] * mRes * magCalibration[2];

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  dest2[0] = avg_rad / ((float)mag_scale[0]);
  dest2[1] = avg_rad / ((float)mag_scale[1]);
  dest2[2] = avg_rad / ((float)mag_scale[2]);

  Serial.println("Mag Calibration done!");

  Serial.println("AK8963 mag biases (mG)");
  Serial.print(cal.magBias[0]); Serial.print(", ");
  Serial.print(cal.magBias[1]); Serial.print(", ");
  Serial.print(cal.magBias[2]); Serial.println();
  Serial.println("AK8963 mag scale (mG)");
  Serial.print(cal.magScale[0]); Serial.print(", ");
  Serial.print(cal.magScale[1]); Serial.print(", ");
  Serial.print(cal.magScale[2]); Serial.println();
}


bool dataAvailable()
{
  return (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01);
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

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

void writeBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* data) {

  Wire.beginTransmission(address);
  Wire.write((uint8_t) subAddress); // send address
  for (uint8_t i = 0; i < count; i++) {
    Wire.write((uint8_t) data[i]);
  }
  Wire.endTransmission();
}
//fast invsqrt
float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}
