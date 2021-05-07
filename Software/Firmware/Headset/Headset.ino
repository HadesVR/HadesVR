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
bool newCtrlData = false;

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
      radio.read(&ContData.Ctrl1_QuatW, 30);
      newCtrlData = true;
    }
    if (pipenum == 2) {
      radio.read(&ContData.Ctrl2_QuatW, 30);
      newCtrlData = true;
    }
  }
  if(newCtrlData){
    HID().SendReport(1, &ContData, 63);  
    newCtrlData = false;
  }
}
