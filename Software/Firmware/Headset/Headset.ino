#include <SPI.h>
#include "RF24.h"
#include "mpu.h"
#include "I2Cdev.h"
#include "HID.h"

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
  Fastwire::setup(400, 0);
  mympu_open(200);
  radio.begin();
  radio.setPayloadSize(40);
  radio.openReadingPipe(2, leftCtrlPipe);
  radio.openReadingPipe(1, rightCtrlPipe);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  HMDData.PacketID = 1;
  ContData.PacketID = 2;
}



void loop() {
  uint8_t pipenum;

  mympu_update();

  HMDData.HMDQuatW = mympu.qW;
  HMDData.HMDQuatX = mympu.qY;
  HMDData.HMDQuatY = mympu.qZ;
  HMDData.HMDQuatZ = mympu.qX;
  
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

  if(newContData){

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
  }
}
