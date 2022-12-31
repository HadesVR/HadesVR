#pragma once

#ifndef _dataHandler_
#define _dataHandler_

#include <windows.h>
#include <thread>
#include <chrono>
#include <atlstr.h> 
#include <math.h>

#include "filters/V3Kalman.h"
#include "filters/MadgwickOrientation.h"
#include "Quaternion.hpp"
#include "Vector3.hpp"
#include "PSMoveService/PSMoveClient_CAPI.h"
#include "driverlog.h"
#include "settingsAPIKeys.h"
#include "DataTransport.hpp"
#include "HIDTransport.hpp"

using namespace ATL;
using namespace std::chrono;
using namespace vr;

typedef struct _TrackingData {
	Vector3 AngularVelocity = Vector3::Zero();
	Vector3 AngularAccel = Vector3::Zero();

	Vector3 Accel = Vector3::Zero();
	Vector3 oldAccel = Vector3::Zero();

	Vector3 Velocity = Vector3::Zero();

	Vector3 Position = Vector3::Zero();
	Vector3 oldPosition = Vector3::Zero();

	Vector3 LastCameraPos = Vector3::Zero();						//last camera position, unfiltered
	Vector3 TempIMUPos = Vector3::Zero();							
	Vector3 CameraVelocity = Vector3::Zero();					//camera velocity, unfiltered, no IMU involvement.
	Vector3 LastCameraVelocity = Vector3::Zero();					//last camera velocity, unfiltered, no IMU involvement.

	bool isTracked = false;
	bool wasTracked = false;

	Quaternion RawRotation = Quaternion::Identity();
	Quaternion VectorRotation = Quaternion::Identity();
	Quaternion OutputRotation = Quaternion::Identity();

	Quaternion RotationConfigOffset = Quaternion::Identity();			//offset in the config filer
	Quaternion RotationUserOffset = Quaternion::Identity();				//offset when pressing f8
	Quaternion RotationDriftOffset = Quaternion::Identity();			//drift offset (internal)

	Vector3 PositionOffset = Vector3::Zero();

	std::chrono::steady_clock::time_point lastIMUUpdate;
	std::chrono::steady_clock::time_point lastCamUpdate;
};

typedef struct _HMDData
{
	_TrackingData TrackingData;
	uint16_t Data;
} THMD, * PHMD;

typedef struct _ControllerData
{
	_TrackingData TrackingData;

	uint16_t Buttons;
	float	Trigger;
	float	JoyAxisX;
	float	JoyAxisY;
	float   TrackpY;
	float	vBat;
	float	FingThumb;
	float	FingIndex;
	float	FingMiddl;
	float	FingRing;
	float	FingPinky;
	uint16_t Data;
} TController, * PController;

typedef struct _TrackerData
{
	Vector3 Position;
	Quaternion Rotation;
	float	vBat;
} TTracker, * PTracker;

#pragma pack(push, 1)
struct HMDQuaternionPacket
{
	uint8_t HIDID;			//this is fucking stupid
	uint8_t  PacketID;
	float HMDQuatW;
	float HMDQuatX;
	float HMDQuatY;
	float HMDQuatZ;

	int16_t accX;
	int16_t accY;
	int16_t accZ;

	uint16_t HMDData;

	int16_t tracker1_QuatW;
	int16_t tracker1_QuatX;
	int16_t tracker1_QuatY;
	int16_t tracker1_QuatZ;
	uint8_t tracker1_vBat;
	uint8_t tracker1_data;

	int16_t tracker2_QuatW;
	int16_t tracker2_QuatX;
	int16_t tracker2_QuatY;
	int16_t tracker2_QuatZ;
	uint8_t tracker2_vBat;
	uint8_t tracker2_data;

	int16_t tracker3_QuatW;
	int16_t tracker3_QuatX;
	int16_t tracker3_QuatY;
	int16_t tracker3_QuatZ;
	uint8_t tracker3_vBat;
	uint8_t tracker3_data;

	uint8_t Padding[8];
};

struct HMDRAWPacket
{
	uint8_t HIDID;		
	uint8_t  PacketID;

	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;

	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;

	int16_t MagX;
	int16_t MagY;
	int16_t MagZ;

	uint16_t HMDData;

	int16_t tracker1_QuatW;
	int16_t tracker1_QuatX;
	int16_t tracker1_QuatY;
	int16_t tracker1_QuatZ;
	uint8_t tracker1_vBat;
	uint8_t tracker1_data;

	int16_t tracker2_QuatW;
	int16_t tracker2_QuatX;
	int16_t tracker2_QuatY;
	int16_t tracker2_QuatZ;
	uint8_t tracker2_vBat;
	uint8_t tracker2_data;

	int16_t tracker3_QuatW;
	int16_t tracker3_QuatX;
	int16_t tracker3_QuatY;
	int16_t tracker3_QuatZ;
	uint8_t tracker3_vBat;
	uint8_t tracker3_data;

	uint8_t Padding[12];
};

struct ControllerPacket
{
	uint8_t HIDID;			
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
	uint16_t Ctrl2_Data;
	uint8_t Padding[6];
};
#pragma pack(pop)

class CdataHandler {
public:
	CdataHandler(DataTransport& transport);
	virtual ~CdataHandler() {}

	DriverPose_t GetHMDPose();
	DriverPose_t GetControllersPose(int ControllerIndex);
	void GetControllerData(TController* RightController, TController* LeftController);
	void GetTrackersData(TTracker* waistTracker, TTracker* leftTracker, TTracker* rightTracker);
	
	void StartData();
	void StopData();

	void SetCentering(bool reset);

	bool HeadsetConnected() { return dataTransport.IsConnected(); }

	bool PSMConnected = false;
	std::thread* pPSMUpdatethread = NULL;
	std::thread* pTransportthread = NULL;
	int psmsUpdateRate = 60;
	int psmsMillisecondPeriod;

private:
	//for dev stuff
	void TestThread();
	std::thread* pTestThread = NULL;
	static void TestThreadEnter(CdataHandler* ptr) {
		ptr->TestThread();
	}

    void ResetPos(bool controllers, bool hmd);
	void ReadTransportData();
	bool connectToPSMOVE();
	void PSMUpdate();

	void UpdateIMUPosition(_TrackingData& _data, V3Kalman& k);
	void UpdateVelocity(_TrackingData& _data, bool _wasTracked, Vector3 newCameraPos);
	void UpdateDriftCorrection(_TrackingData& _data, Vector3 newCameraPos, float percent, float lowerTreshold, float upperTreshold, bool debug);

	void SetOffsetQuat(_TrackingData& _data);
	void SaveUserOffset(float DataW, float DataY, Quaternion& Offset, const char* settingsKey);

	DataTransport& dataTransport;

	_HMDData		HMDData;
	_ControllerData RightCtrlData;
	_ControllerData LeftCtrlData;
	_TrackerData	TrackerWaistData;
	_TrackerData	TrackerLeftData;
	_TrackerData	TrackerRightData;
	
	uint8_t packet_buffer[64];

	bool orientationFilterInit = false;
	bool ctrl1Allocated = false, ctrl2Allocated = false, HMDAllocated = false;
	bool CtrlAccelEnable = false;
	bool HMDAccelEnable = false;

	double k_fScalePSMoveAPIToMeters = 0.01f; // psmove driver in cm

	PSMControllerList controllerList;
	PSMHmdList hmdList;
	PSMVector3f psmHmdPos, psmCtrlRightPos, psmCtrlLeftPos;

	Madgwick HMDfilter;

	int readsFromInit = 0;
	bool receivedControllerData = false;
	float minFilterBeta = 0.02f;
	float maxFilterBeta = 0.30f;
	double deltatime = 0;

	bool enableDriftCorrection = false;
	float corrVelocityLowerTreshold = 0.8f;
	float corrVelocityUpperTreshold = 1.5f;
	float hmdDriftCorr = 0.02f;
	float contDriftCorr = 0.05f;
	
	V3Kalman HMDKalman;
	V3Kalman CtrlLeftKalman;
	V3Kalman CtrlRightKalman;

	static void PSMUpdateEnter(CdataHandler* ptr) {
		ptr->PSMUpdate();
	}

	static void ReadTransportEnter(CdataHandler* ptr) {
		ptr->ReadTransportData();
	}
};


#endif