#include <SPI.h>
#include "RF24.h"
#include "HID.h"

//#define SERIAL_DEBUG
#define TRANSPORT_TYPE             1        //1 = HID (default), 2 = UART
#define TRANSPORT_SERIAL_PORT      Serial   // Must_not be same as SERIAL_DEBUG (if that is enabled)
#define TRANSPORT_SERIAL_BAUDRATE  230400

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

RF24 radio(9, 10);

const uint64_t rightCtrlPipe = 0xF0F0F0F0E1LL;
const uint64_t leftCtrlPipe = 0xF0F0F0F0D2LL;

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

static ControllerPacket ContData;

void setup() {
  //setup transport
  transport.setup();
  //setup rf
  radio.begin();
  radio.setPayloadSize(40);
  radio.openReadingPipe(2, leftCtrlPipe);
  radio.openReadingPipe(1, rightCtrlPipe);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  if (!radio.isChipConnected())
  {
#ifdef SERIAL_DEBUG
    Serial.println("NRF24L01 Module not detected!");
#endif
    while (true) ;
  }
  else
  {
#ifdef SERIAL_DEBUG
    Serial.println("NRF24L01 Module up and running!");
#endif
  }
  ContData.PacketID = 2;
}

void loop() {
  uint8_t pipenum;
  bool newCtrlData = false;

  if (radio.available(&pipenum)) {                  //thanks SimLeek for this idea!
    if (pipenum == 1) {
      radio.read(&ContData.Ctrl1_QuatW, 29);        //receive right controller data
      newCtrlData = true;
    }
    if (pipenum == 2) {
      radio.read(&ContData.Ctrl2_QuatW, 29);        //receive left controller data
      newCtrlData = true;
    }
  }
  if (newCtrlData) {
    transport.sendPacket(&ContData, 63);
  }
}
