#include <SPI.h>
#include "RF24.h"
#include "mpu.h"
#include "I2Cdev.h"

#define SysPin              4
#define MenuPin             3
#define GripPin             6
#define JoyXPin             A1
#define JoyYPin             A2
#define JoyClickPin         2
#define TriggerPin          A3
#define VbatPin             A0

#define HTC_SysClick    0x0001
#define HTC_MenuClick   0x0004
#define HTC_ThumbstickClick 0x0020
#define HTC_GripClick   0x0080
#define HTC_ThumbstickTouch 0x0800

#define BatLevelMax         968             //you need to find all of these values on your own
#define JoyXMin             237             //check on the utils folder for sketches and instructions
#define JoyXMax             935             //that help on getting these values
#define JoyYMin             190             //YOU NEED TO DO THIS FOR BOTH CONTROLLERS
#define JoyYMax             900             //if you use these values without changing them you MAY
#define JoyXDeadZoneMin     525             //get stick drift
#define JoyXDeadZoneMax     580
#define JoyYDeadZoneMin     460
#define JoyYDeadZoneMax     530

#define CTRLQW        0
#define CTRLQX        1
#define CTRLQY        2
#define CTRLQZ        3
#define CTRLBTN       4
#define CTRLTRIGG     5
#define CTRLAXISX     6
#define CTRLAXISY     7
#define CTRLTRACKY    8
#define CTRLVBAT      9


uint64_t Pipe = 0xF0F0F0F0E1LL; //right
//uint64_t Pipe = 0xF0F0F0F0D2LL; //left

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

float trigger;
float axisXout;
float axisYout;
int tracky;
int trackoutput;
int axisX;
int axisY;

RF24 radio(9, 10);


void setup() {
  Serial.begin(9600);
  
  pinMode(SysPin, INPUT_PULLUP);
  pinMode(MenuPin, INPUT_PULLUP);
  pinMode(GripPin, INPUT_PULLUP);
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

  axisX = analogRead(JoyXPin);
  axisY = analogRead(JoyYPin);

  if (axisX > JoyXDeadZoneMax || axisX < JoyXDeadZoneMin) {
    axisXout = -mapFloat(axisX, JoyXMin, JoyXMax, -1, 1);
  } else {
    axisXout = 0;
  }

  if (axisY > JoyYDeadZoneMax || axisY < JoyYDeadZoneMin) {
    axisYout = mapFloat(axisY, JoyYMin, JoyYMax, -1, 1);
    btn |= HTC_ThumbstickTouch;
  } else {
    axisYout = 0;
  }

  trigger = (mapFloat(analogRead(TriggerPin), 1024, 0, 0, 1));

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

  data.qW = mympu.qW;
  data.qY = mympu.qY;
  data.qZ = mympu.qZ;
  data.qX = mympu.qX;
  data.BTN = btn;
  data.trigg = (trigger * 100);
  data.axisX = (axisXout * 100);
  data.axisY = (axisYout * 100);
  data.vBAT = (mapFloat(analogRead(VbatPin), 787, BatLevelMax, 0, 1) * 100);
  radio.stopListening();
  radio.write(&data, sizeof(ctrlData));
  radio.startListening();
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
