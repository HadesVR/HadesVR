#include <SPI.h>
#include "RF24.h"
#include "mpu.h"
#include "I2Cdev.h"

#define DISPLAY_INTERVAL  5
#define SerialDebug false

#define HMDQW          0
#define HMDQX          1
#define HMDQY          2
#define HMDQZ          3
#define CTRL1QW        4
#define CTRL1QX        5
#define CTRL1QY        6
#define CTRL1QZ        7
#define CTRL1BTN       8
#define CTRL1TRIGG     9
#define CTRL1AXISX     10
#define CTRL1AXISY     11
#define CTRL1TRACKY    12
#define CTRL1VBAT      13
#define CTRL2QW        14
#define CTRL2QX        15
#define CTRL2QY        16
#define CTRL2QZ        17
#define CTRL2BTN       18
#define CTRL2TRIGG     19
#define CTRL2AXISX     20
#define CTRL2AXISY     21
#define CTRL2TRACKY    22
#define CTRL2VBAT      23
#define CHECKSUM       24

unsigned long lastDisplay = 0;
float data[25]; float ctrl1Data[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; float ctrl2Data[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

RF24 radio(9, 10);

const uint64_t rightCtrlPipe = 0xF0F0F0F0E1LL;
const uint64_t leftCtrlPipe = 0xF0F0F0F0D2LL;

void setup() {
  Fastwire::setup(400, 0);
  Serial.begin(115200);
  mympu_open(200);
  while (!Serial) {
    mympu_update();
  }
  radio.begin();
  radio.openReadingPipe(2, leftCtrlPipe);
  radio.openReadingPipe(1, rightCtrlPipe);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
}

void loop() {
  uint8_t pipenum;
  unsigned long now = millis();
  mympu_update();

  if (radio.available(&pipenum)) {                  //thanks SimLeek for this idea!
    if (pipenum == 1) {
      radio.read(&ctrl1Data, sizeof(ctrl1Data));
    }
    if (pipenum == 2) {
      radio.read(&ctrl2Data, sizeof(ctrl2Data));
    }
  }

  if ((now - lastDisplay) >= DISPLAY_INTERVAL)
  {
    if (SerialDebug) {
      Serial.print("W,X,Y,Z: ");
      Serial.print(mympu.qW, 2);
      Serial.print(", ");
      Serial.print(mympu.qX, 2);
      Serial.print(", ");
      Serial.print(mympu.qY, 2);
      Serial.print(", ");
      Serial.println(mympu.qZ, 2);

      Serial.print("pipenum: ");
      Serial.println(pipenum);

      Serial.print("Controller1Data: ");
      Serial.print(ctrl1Data[0], 2);
      Serial.print(", ");
      Serial.print(ctrl1Data[1], 2);
      Serial.print(", ");
      Serial.print(ctrl1Data[2], 2);
      Serial.print(", ");
      Serial.print(ctrl1Data[3], 2);
      Serial.print(", ");
      Serial.print(ctrl1Data[4], 2);
      Serial.print(", ");
      Serial.print(ctrl1Data[5], 2);
      Serial.print(", ");
      Serial.print(ctrl1Data[6], 2);
      Serial.print(", ");
      Serial.print(ctrl1Data[7], 2);
      Serial.print(", ");
      Serial.print(ctrl1Data[8], 2);
      Serial.print(", ");
      Serial.println(ctrl1Data[9], 2);

      delay(100);
    }
    else {
      data[HMDQW] = mympu.qW;
      data[HMDQX] = mympu.qY;
      data[HMDQY] = mympu.qZ;
      data[HMDQZ] = mympu.qX;

      data[CTRL1QW]     = ctrl1Data[0];
      data[CTRL1QX]     = ctrl1Data[1];
      data[CTRL1QY]     = ctrl1Data[2];
      data[CTRL1QZ]     = ctrl1Data[3];
      data[CTRL1BTN]    = ctrl1Data[4];
      data[CTRL1TRIGG]  = ctrl1Data[5];
      data[CTRL1AXISX]  = ctrl1Data[6];
      data[CTRL1AXISY]  = ctrl1Data[7];
      data[CTRL1TRACKY] = ctrl1Data[8];
      data[CTRL1VBAT]   = ctrl1Data[9];

      data[CTRL2QW]     = ctrl2Data[0];
      data[CTRL2QX]     = ctrl2Data[1];
      data[CTRL2QY]     = ctrl2Data[2];
      data[CTRL2QZ]     = ctrl2Data[3];
      data[CTRL2BTN]    = ctrl2Data[4];
      data[CTRL2TRIGG]  = ctrl2Data[5];
      data[CTRL2AXISX]  = ctrl2Data[6];
      data[CTRL2AXISY]  = ctrl2Data[7];
      data[CTRL2TRACKY] = ctrl2Data[8];
      data[CTRL2VBAT]   = ctrl2Data[9];

      data[CHECKSUM] = 29578643;                //TODO: make proper checksum
      Serial.write((byte*)&data, sizeof(data)); 
    }
    lastDisplay = now;
  }
}
