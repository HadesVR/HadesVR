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
float data[25]; float ctrl2Data[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

RF24 radio(9, 10);

const uint64_t rightCtrlPipe = 0xF0F0F0F0E1LL;
const uint64_t leftCtrlPipe = 0xF0F0F0F0D2LL;

struct ctrlData{
  float qW;
  float qX;
  float qY;
  float qZ;
  float BTN;
  short  trigg;
  short  axisX;
  short  axisY;
  short  trackY;
  short  vBAT;
};

ctrlData Ctrl1Data,Ctrl2Data;

void setup() {
  
  Fastwire::setup(400, 0);
  Serial.begin(115200);
  mympu_open(200);
  while (!Serial) {
    mympu_update();
  }
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
  unsigned long now = millis();
  mympu_update();

  if (radio.available(&pipenum)) {                  //thanks SimLeek for this idea!
    if (pipenum == 1) {
      radio.read(&Ctrl1Data, sizeof(ctrlData));
    }
    if (pipenum == 2) {
      radio.read(&Ctrl2Data, sizeof(ctrlData));
    }
  }

  if ((now - lastDisplay) >= DISPLAY_INTERVAL)
  {
    if (SerialDebug) {
//      Serial.print("W,X,Y,Z: ");
//      Serial.print(mympu.qW, 2);
//      Serial.print(", ");
//      Serial.print(mympu.qX, 2);
//      Serial.print(", ");
//      Serial.print(mympu.qY, 2);
//      Serial.print(", ");
//      Serial.println(mympu.qZ, 2);
//
//      Serial.print("pipenum: ");
//      Serial.println(pipenum);

      Serial.print("Controller1Data: ");
      Serial.print(Ctrl1Data.qW, 2);
      Serial.print(", ");
      Serial.print(Ctrl1Data.qX, 2);
      Serial.print(", ");
      Serial.print(Ctrl1Data.qY, 2);
      Serial.print(", ");
      Serial.print(Ctrl1Data.qZ, 2);
      Serial.print(", ");
      Serial.print(Ctrl1Data.BTN, 2);
      Serial.print(", ");
      Serial.print(((float)Ctrl1Data.trigg/100), 2);
      Serial.print(", ");
      Serial.print(((float)Ctrl1Data.axisX /100), 2);
      Serial.print(", ");
      Serial.print(((float)Ctrl1Data.axisY /100), 2);
      Serial.print(", ");
      Serial.print(((float)Ctrl1Data.trackY /100), 2);
      Serial.print(", ");
      Serial.println(((float)Ctrl1Data.vBAT /100) , 2);

      delay(100);
    }
    else {
      data[HMDQW] = mympu.qW;
      data[HMDQX] = mympu.qY;
      data[HMDQY] = mympu.qZ;
      data[HMDQZ] = mympu.qX;

      data[CTRL1QW]     = Ctrl1Data.qW;
      data[CTRL1QX]     = Ctrl1Data.qX;
      data[CTRL1QY]     = Ctrl1Data.qY;
      data[CTRL1QZ]     = Ctrl1Data.qZ;
      data[CTRL1BTN]    = Ctrl1Data.BTN;
      data[CTRL1TRIGG]  = ((float)Ctrl1Data.trigg/100);
      data[CTRL1AXISX]  = ((float)Ctrl1Data.axisX/100);
      data[CTRL1AXISY]  = ((float)Ctrl1Data.axisY/100);
      data[CTRL1TRACKY] = ((float)Ctrl1Data.trackY/100);
      data[CTRL1VBAT]   = ((float)Ctrl1Data.vBAT/100);

      data[CTRL2QW]     = Ctrl2Data.qW;
      data[CTRL2QX]     = Ctrl2Data.qX;
      data[CTRL2QY]     = Ctrl2Data.qY;
      data[CTRL2QZ]     = Ctrl2Data.qZ;
      data[CTRL2BTN]    = Ctrl2Data.BTN;
      data[CTRL2TRIGG]  = ((float)Ctrl2Data.trigg/100);
      data[CTRL2AXISX]  = ((float)Ctrl2Data.axisX/100);
      data[CTRL2AXISY]  = ((float)Ctrl2Data.axisY/100);
      data[CTRL2TRACKY] = ((float)Ctrl2Data.trackY/100);
      data[CTRL2VBAT]   = ((float)Ctrl2Data.vBAT/100);

      data[CHECKSUM] = 29578643;                //TODO: make proper checksum
      Serial.write((byte*)&data, sizeof(data)); 
    }
    lastDisplay = now;
  }
}
