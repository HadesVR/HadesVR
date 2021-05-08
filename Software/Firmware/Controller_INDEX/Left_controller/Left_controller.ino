#include <SPI.h>
#include "RF24.h"
#include "mpu.h"
#include "I2Cdev.h"

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

#define IB_AClick           0x0001
#define IB_ATouch           0x0002
#define IB_BClick           0x0004
#define IB_BTouch           0x0008
#define IB_SYSClick         0x0010
#define IB_ThumbStickClick  0x0020
#define IB_TrackpadTouch    0x0040
#define IB_ThumbStickTouch  0x0080

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

int tracky;
int trackoutput;
int axisX;
int axisY;

RF24 radio(9, 10);


void setup() {

  pinMode(APin, INPUT_PULLUP);
  pinMode(BPin, INPUT_PULLUP);
  pinMode(SysPin, INPUT_PULLUP);
  pinMode(JoyClickPin, INPUT_PULLUP);
  pinMode(TriggerPin, INPUT_PULLUP);

  Fastwire::setup(400, 0);
  mympu_open(200);
  radio.begin();
  radio.setPayloadSize(40);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.openWritingPipe(Pipe);
  radio.startListening();
  radio.setAutoAck(false);

}

void loop() {
  mympu_update();
  ctrlData data;
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
    data.axisX = -map(axisX, JoyXMin, JoyXMax, -127, 127);
  } else {
    data.axisX = 0;
  }

  if (axisY > JoyYDeadZoneMax || axisY < JoyYDeadZoneMin) {
    data.axisY = map(axisY, JoyYMin, JoyYMax, -127, 127);
    btn |= IB_ThumbStickTouch;
  } else {
    data.axisY = 0;
  }


  if (analogRead(TriggerPin) < 1000) {
    data.trigg = map(analogRead(TriggerPin), 1024, 0, 0, 255);
    data.fingerIndex = map(analogRead(TriggerPin), 1024, 0, 0, 255);
  }
  else{
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

  data.qW = mympu.qW;
  data.qX = mympu.qY;
  data.qY = mympu.qZ;
  data.qZ = mympu.qX;
  data.BTN = btn;
  data.trackY = (trackoutput * 127);
  data.vBAT = (map(analogRead(VbatPin), 787, BatLevelMax, 0, 255));
  radio.stopListening();
  radio.write(&data, sizeof(ctrlData));
  radio.startListening();
}
