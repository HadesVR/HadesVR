#pragma once

#ifndef _dataHandler_
#define _dataHandler_

#include <windows.h>
#include <thread>
#include <atlstr.h> 
#include <math.h>

#include "Quaternion.hpp"
#include "PSMoveService/PSMoveClient_CAPI.h"
#include "hidapi/hidapi.h"
#include "driverlog.h"
#include "settingsAPIKeys.h"

using namespace ATL;

typedef struct _HMDData
{
	double	X;
	double	Y;
	double	Z;
	double	qW;
	double	qX;
	double	qY;
	double  qZ;
	float   accelX;
	float   accelY;
	float   accelZ;
	uint16_t Data;
} THMD, * PHMD;

typedef struct _Controller
{
	double	X;
	double	Y;
	double	Z;
	double	qW;
	double	qX;
	double	qY;
	double  qZ;
	float   accelX;
	float   accelY;
	float   accelZ;
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
	double	X;
	double	Y;
	double	Z;
	double	qW;
	double	qX;
	double	qY;
	double  qZ;
	float	vBat;
} TTracker, * PTracker;

#pragma pack(push, 1)

struct HMDPacket
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

struct ControllerPacket
{
	uint8_t HIDID;			//this is also fucking stupid
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
	void SetCentering();
	void ReadHIDData();
	void SerialStreamStart();
	void PSMUpdate();
	void GetHMDData(THMD* HMD);
	void GetControllersData(TController* RightController, TController* LeftController);
	void GetTrackersData(TTracker* waistTracker, TTracker* leftTracker, TTracker* rightTracker);
	bool connectToPSMOVE();
	void StartData(int32_t PID, int32_t VID);
	void CdataHandler::stopData();

	hid_device* hHID;
	bool HIDConnected = false;
	bool PSMConnected = false;
	std::thread* pHIDthread = NULL;
	std::thread* pPSMUpdatethread = NULL;

private:

	_HMDData	HMDData;
	_Controller RightCtrlData;
	_Controller LeftCtrlData;
	_TrackerData TrackerWaistData;
	_TrackerData TrackerLeftData;
	_TrackerData TrackerRightData;
	

	uint8_t packet_buffer[64];

	float lerp(const float a, const float b, const float f);

	Quaternion HMDOffset = Quaternion::Identity();
	Quaternion RightCtrlOffset = Quaternion::Identity();
	Quaternion LeftCtrlOffset = Quaternion::Identity();
	Quaternion WaistTrackerOffset = Quaternion::Identity();
	Quaternion LeftTrackerOffset = Quaternion::Identity();
	Quaternion RightTrackerOffset = Quaternion::Identity();

	Quaternion CTRL1ConfigOffset = Quaternion::Identity();
	Quaternion CTRL2ConfigOffset = Quaternion::Identity();

	Quaternion HMDConfigOffset = Quaternion::Identity();

	bool HIDInit = false;
	bool ctrl1Allocated = false, ctrl2Allocated = false, HMDAllocated = false;

	float k_fScalePSMoveAPIToMeters = 0.01f; // psmove driver in cm

	PSMControllerList controllerList;
	PSMHmdList hmdList;
	PSMVector3f hmdPos, ctrlRightPos, ctrlLeftPos;

	static void PSMUpdateEnter(CdataHandler* ptr) {
		ptr->PSMUpdate();
	}

	static void ReadHIDEnter(CdataHandler* ptr) {
		ptr->ReadHIDData();
	}
};


#endif