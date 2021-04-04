#pragma once

#ifndef _dataHandler_
#define _dataHandler_

#include <windows.h>
#include <thread>
#include <atlstr.h> 
#include <math.h>

#include "Quaternion.hpp"
#include "PSMoveService/PSMoveClient_CAPI.h"
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
	unsigned short	Buttons;
	float	Trigger;
	float	JoyAxisX;
	float	JoyAxisY;
	float   TrackpY;
	float	vBat;
} TController, * PController;

class CdataHandler {
public:
	void SetCentering();
	void ReadSerialData();
	void SerialStreamStart();
	void PSMUpdate();
	void GetHMDData(THMD* HMD);
	void GetControllersData(TController* FirstController, TController* SecondController);
	bool connectToPSMOVE();
	void StartData(int comPort);
	

	HANDLE hSerial;
	bool SerialConnected = false;
	bool PSMConnected = false;
	std::thread* pCtrlthread = NULL;
	std::thread* pPSMUpdatethread = NULL;

private:

	float lerp(const float a, const float b, const float f);

	float ArduinoData[25] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	float LastArduinoArduinoData[24] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

	Quaternion Ctrl1Offset = Quaternion::Identity();
	Quaternion Ctrl2Offset = Quaternion::Identity();
	Quaternion HMDOffset = Quaternion::Identity();

	bool CtrlInitCentring = false;
	bool SerialInit = false;
	bool ctrl1Allocated = false, ctrl2Allocated = false, HMDAllocated = false;

	float k_fScalePSMoveAPIToMeters = 0.01f; // psmove driver in cm

	PSMControllerList controllerList;
	PSMHmdList hmdList;
	PSMVector3f hmdPos, ctrl1Pos, ctrl2Pos;

	int dataCOMPort;

	static void PSMUpdateEnter(CdataHandler* ptr) {
		ptr->PSMUpdate();
	}

	static void ReadSerialDataEnter(CdataHandler* ptr) {
		ptr->ReadSerialData();
	}
	
};


#endif