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

typedef struct _HMDData
{
	double	X;
	double	Y;
	double	Z;
	double	qW;
	double	qX;
	double	qY;
	double  qZ;
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
	uint32_t Buttons;
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
} TController, * PController;

typedef struct HMDPacket
{
	uint8_t ID;
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

	uint8_t Padding[20];
};

typedef struct ControllerPacket
{
	uint8_t ID;
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
	uint8_t Padding[3];
};

class CdataHandler {
public:
	void SetCentering();
	void ReadHIDData();
	void SerialStreamStart();
	void PSMUpdate();
	void GetHMDData(THMD* HMD);
	void GetControllersData(TController* FirstController, TController* SecondController);
	bool connectToPSMOVE();
	void StartData(int32_t PID, int32_t VID);
	void CdataHandler::stopData();

	hid_device* hHID;
	bool HIDConnected = false;
	bool PSMConnected = false;
	std::thread* pHIDthread = NULL;
	std::thread* pPSMUpdatethread = NULL;

private:

	_Controller Ctrl1Data;
	_Controller Ctrl2Data;
	_HMDData	HMDData;

	uint8_t packet_buffer[64];

	float lerp(const float a, const float b, const float f);

	Quaternion Ctrl1Offset = Quaternion::Identity();
	Quaternion Ctrl2Offset = Quaternion::Identity();
	Quaternion HMDOffset = Quaternion::Identity();

	bool InitCentring = false;
	bool HIDInit = false;
	bool ctrl1Allocated = false, ctrl2Allocated = false, HMDAllocated = false;

	float k_fScalePSMoveAPIToMeters = 0.01f; // psmove driver in cm

	PSMControllerList controllerList;
	PSMHmdList hmdList;
	PSMVector3f hmdPos, ctrl1Pos, ctrl2Pos;

	static void PSMUpdateEnter(CdataHandler* ptr) {
		ptr->PSMUpdate();
	}

	static void ReadHIDEnter(CdataHandler* ptr) {
		ptr->ReadHIDData();
	}
};


#endif