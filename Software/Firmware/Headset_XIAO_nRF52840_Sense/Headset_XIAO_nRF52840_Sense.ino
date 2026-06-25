/*
  HadesVR inside-out MVP headset firmware for Seeed Studio XIAO nRF52840 Sense.

  Transport:
    USB CDC serial, HadesVR UART frame format:
    <magic 0xAA><frame seq:1><payload size:1><report id 0x01><63 byte packet>

  Packet:
    PacketID 4 is a timestamped raw IMU packet. The SteamVR driver converts it
    through the existing HadesVR Madgwick orientation path.
*/

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include "LSM6DS3.h"

#define IMU_I2C_ADDRESS 0x6A
#define USB_SERIAL_BAUDRATE 230400
#define TARGET_SAMPLE_RATE_HZ 250u
#define SAMPLE_PERIOD_US (1000000u / TARGET_SAMPLE_RATE_HZ)
#define HADESVR_UART_MAGIC ((uint8_t)0xAA)
#define HADESVR_REPORT_ID ((uint8_t)0x01)
#define HADESVR_PACKET_ID_TIMESTAMPED_IMU ((uint8_t)0x04)

LSM6DS3 imu(I2C_MODE, IMU_I2C_ADDRESS);

class DataTransport {
public:
  virtual ~DataTransport() {}
  virtual void begin() = 0;
  virtual void sendPacket(const void* packet, uint8_t len) = 0;
};

class UsbSerialTransport : public DataTransport {
public:
  void begin() override {
    Serial.begin(USB_SERIAL_BAUDRATE);
    const uint32_t startMs = millis();
    while (!Serial && (millis() - startMs) < 3000) {
      delay(10);
    }
  }

  void sendPacket(const void* packet, uint8_t len) override {
    const uint8_t payloadSize = len + 1;
    Serial.write(HADESVR_UART_MAGIC);
    Serial.write(frameSequence++);
    Serial.write(payloadSize);
    Serial.write(HADESVR_REPORT_ID);
    Serial.write((const uint8_t*)packet, len);
  }

private:
  uint8_t frameSequence = 0;
};

static UsbSerialTransport transport;

#pragma pack(push, 1)
struct HMDIMUPacket {
  uint8_t PacketID;
  uint8_t Version;
  uint16_t Sequence;
  uint32_t TimestampUs;

  int16_t AccX;
  int16_t AccY;
  int16_t AccZ;

  int16_t GyroX;
  int16_t GyroY;
  int16_t GyroZ;

  int16_t MagX;
  int16_t MagY;
  int16_t MagZ;

  int16_t Temperature;
  uint16_t HMDData;
  uint16_t SamplePeriodUs;
  uint8_t ImuStatus;

  uint8_t Reserved[30];
};
#pragma pack(pop)

static_assert(sizeof(HMDIMUPacket) == 63, "HMDIMUPacket must stay 63 bytes");

static HMDIMUPacket packet;
static uint16_t imuSequence = 0;
static uint32_t nextSampleUs = 0;

int16_t scaleToInt16(float value, float scale) {
  const float scaled = value * scale;
  if (scaled > 32767.0f) return 32767;
  if (scaled < -32768.0f) return -32768;
  return (int16_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

void blinkError() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(400);
  }
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  transport.begin();

  imu.settings.gyroRange = 2000;
  imu.settings.gyroSampleRate = 416;
  imu.settings.gyroBandWidth = 400;
  imu.settings.accelRange = 16;
  imu.settings.accelSampleRate = 416;
  imu.settings.accelBandWidth = 100;
  imu.settings.tempEnabled = 1;

  if (imu.begin() != IMU_SUCCESS) {
    blinkError();
  }

  memset(&packet, 0, sizeof(packet));
  packet.PacketID = HADESVR_PACKET_ID_TIMESTAMPED_IMU;
  packet.Version = 1;
  packet.SamplePeriodUs = SAMPLE_PERIOD_US;
  packet.ImuStatus = 0x01;

  nextSampleUs = micros();
}

void loop() {
  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextSampleUs) < 0) {
    return;
  }
  nextSampleUs += SAMPLE_PERIOD_US;

  const float ax = imu.readFloatAccelX();
  const float ay = imu.readFloatAccelY();
  const float az = imu.readFloatAccelZ();
  const float gx = imu.readFloatGyroX();
  const float gy = imu.readFloatGyroY();
  const float gz = imu.readFloatGyroZ();

  packet.PacketID = HADESVR_PACKET_ID_TIMESTAMPED_IMU;
  packet.Version = 1;
  packet.Sequence = imuSequence++;
  packet.TimestampUs = nowUs;
  packet.AccX = scaleToInt16(ax, 2048.0f);
  packet.AccY = scaleToInt16(ay, 2048.0f);
  packet.AccZ = scaleToInt16(az, 2048.0f);
  packet.GyroX = scaleToInt16(gx, 16.0f);
  packet.GyroY = scaleToInt16(gy, 16.0f);
  packet.GyroZ = scaleToInt16(gz, 16.0f);
  packet.MagX = 0;
  packet.MagY = 0;
  packet.MagZ = 0;
  packet.Temperature = scaleToInt16(imu.readTempC(), 100.0f);
  packet.HMDData = 0;
  packet.SamplePeriodUs = SAMPLE_PERIOD_US;
  packet.ImuStatus = 0x01;

  transport.sendPacket(&packet, sizeof(packet));
}
