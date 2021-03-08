#ifndef _dataHandler_
#define _dataHandler_

#pragma once

#include <windows.h>
#include <thread>
#include <atlstr.h> 
#include <math.h>

#include "Quaternion.hpp"
#include "PSMoveService/PSMoveClient_CAPI.h"
#include "driverlog.h"

using namespace ATL;

#define HMDQW          0
#define HMDQX          1
#define HMDQY          2
#define HMDQZ          3
#define CTRL1QW        4
#define CTRL1QX        5
#define CTRL1QY        6
#define CTRL1QZ        7
#define CTRL1BTN       8
#define CTRL1TRIGG     9
#define CTRL1AXISX     10
#define CTRL1AXISY     11
#define CTRL1TRACKY	   12
#define CTRL1VBAT      13
#define CTRL2QW        14
#define CTRL2QX        15
#define CTRL2QY        16
#define CTRL2QZ        17
#define CTRL2BTN       18
#define CTRL2TRIGG     19
#define CTRL2AXISX     20
#define CTRL2AXISY     21
#define CTRL2TRACKY	   22
#define CTRL2VBAT      23
#define CHECKSUM       24

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

#define SUCCESS 0
#define FAILURE 1

HANDLE hSerial;

std::thread* pCtrlthread = NULL;
std::thread* pPSMUpdatethread = NULL;

float ArduinoData[25] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
float LastArduinoArduinoData[24] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

Quaternion Ctrl1Offset = Quaternion::Identity();
Quaternion Ctrl2Offset = Quaternion::Identity();
Quaternion HMDOffset = Quaternion::Identity();

bool SerialConnected = false;
bool PSMConnected = false;
bool CtrlInitCentring = false;
bool SerialInit = false;
bool ctrl1Allocated = false, ctrl2Allocated = false, HMDAllocated = false;

static const float k_fScalePSMoveAPIToMeters = 0.01f; // psmove driver in cm

PSMControllerList controllerList;
PSMHmdList hmdList;
PSMVector3f hmdPos, ctrl1Pos, ctrl2Pos;

int dataCOMPort;


void SetCentering()
{
	Ctrl1Offset.W = ArduinoData[CTRL1QW];
	Ctrl1Offset.Y = -ArduinoData[CTRL1QY];

	Ctrl2Offset.W = ArduinoData[CTRL2QW];
	Ctrl2Offset.Y = -ArduinoData[CTRL2QY];

	HMDOffset.W = ArduinoData[HMDQW];
	HMDOffset.Y = -ArduinoData[HMDQY];
}

inline Quaternion SetOffsetQuat(double qW, double qX, double qY, double qZ, Quaternion offsetQuat)
{
	Quaternion Input;
	Input.W = qW;
	Input.X = qX;
	Input.Y = qY;
	Input.Z = qZ;
	Quaternion Output = Quaternion::Normalized(offsetQuat * Input);
	return Output;
}

void ReadSerialData()
{
	bool firstread = false;
	DWORD bytesRead;
	while (SerialConnected) {
		ReadFile(hSerial, &ArduinoData, sizeof(ArduinoData), &bytesRead, 0);
		//Filter incorrect values
		if (!(ArduinoData[CHECKSUM] == 29578643))
		{
			//Last correct values
			ArduinoData[0] = LastArduinoArduinoData[0];
			ArduinoData[1] = LastArduinoArduinoData[1];
			ArduinoData[2] = LastArduinoArduinoData[2];
			ArduinoData[3] = LastArduinoArduinoData[3];
			ArduinoData[4] = LastArduinoArduinoData[4];
			ArduinoData[5] = LastArduinoArduinoData[5];
			ArduinoData[6] = LastArduinoArduinoData[6];
			ArduinoData[7] = LastArduinoArduinoData[7];
			ArduinoData[8] = LastArduinoArduinoData[8];
			ArduinoData[9] = LastArduinoArduinoData[9];
			ArduinoData[10] = LastArduinoArduinoData[10];
			ArduinoData[11] = LastArduinoArduinoData[11];
			ArduinoData[12] = LastArduinoArduinoData[12];
			ArduinoData[13] = LastArduinoArduinoData[13];
			ArduinoData[14] = LastArduinoArduinoData[14];
			ArduinoData[15] = LastArduinoArduinoData[15];
			ArduinoData[16] = LastArduinoArduinoData[16];
			ArduinoData[17] = LastArduinoArduinoData[17];
			ArduinoData[18] = LastArduinoArduinoData[18];
			ArduinoData[19] = LastArduinoArduinoData[19];
			ArduinoData[20] = LastArduinoArduinoData[20];
			ArduinoData[21] = LastArduinoArduinoData[21];
			ArduinoData[22] = LastArduinoArduinoData[22];
			ArduinoData[23] = LastArduinoArduinoData[23];

			PurgeComm(hSerial, PURGE_TXCLEAR | PURGE_RXCLEAR);
		}

		//Save last correct values
		if (ArduinoData[CHECKSUM] == 29578643)
		{
			LastArduinoArduinoData[0] = ArduinoData[0];
			LastArduinoArduinoData[1] = ArduinoData[1];
			LastArduinoArduinoData[2] = ArduinoData[2];
			LastArduinoArduinoData[3] = ArduinoData[3];
			LastArduinoArduinoData[4] = ArduinoData[4];
			LastArduinoArduinoData[5] = ArduinoData[5];
			LastArduinoArduinoData[6] = ArduinoData[6];
			LastArduinoArduinoData[7] = ArduinoData[7];
			LastArduinoArduinoData[8] = ArduinoData[8];
			LastArduinoArduinoData[9] = ArduinoData[9];
			LastArduinoArduinoData[10] = ArduinoData[10];
			LastArduinoArduinoData[11] = ArduinoData[11];
			LastArduinoArduinoData[12] = ArduinoData[12];
			LastArduinoArduinoData[13] = ArduinoData[13];
			LastArduinoArduinoData[14] = ArduinoData[14];
			LastArduinoArduinoData[15] = ArduinoData[15];
			LastArduinoArduinoData[16] = ArduinoData[16];
			LastArduinoArduinoData[17] = ArduinoData[17];
			LastArduinoArduinoData[18] = ArduinoData[18];
			LastArduinoArduinoData[19] = ArduinoData[19];
			LastArduinoArduinoData[20] = ArduinoData[20];
			LastArduinoArduinoData[21] = ArduinoData[21];
			LastArduinoArduinoData[22] = ArduinoData[22];
			LastArduinoArduinoData[23] = ArduinoData[23];

		}

		if (CtrlInitCentring == false)
			if (ArduinoData[0] != 0 || ArduinoData[1] != 0 || ArduinoData[2] != 0) {
				SetCentering();
				CtrlInitCentring = true;
			}
	}
}

void SerialStreamStart() {
	CString sPortName;
	sPortName.Format(_T("COM%d"), (int)dataCOMPort);

	hSerial = ::CreateFile(sPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

	if (hSerial != INVALID_HANDLE_VALUE && GetLastError() != ERROR_FILE_NOT_FOUND) {

		DCB dcbSerialParams = { 0 };
		dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

		if (GetCommState(hSerial, &dcbSerialParams))
		{
			dcbSerialParams.BaudRate = CBR_115200;
			dcbSerialParams.ByteSize = 8;
			dcbSerialParams.StopBits = ONESTOPBIT;
			dcbSerialParams.Parity = NOPARITY;

			if (SetCommState(hSerial, &dcbSerialParams))
			{
				SerialConnected = true;
				PurgeComm(hSerial, PURGE_TXCLEAR | PURGE_RXCLEAR);
				pCtrlthread = new std::thread(ReadSerialData);
				DriverLog("[DataStream]: created ReadSerialData thread");
			}
		}
	}
}

void PSMUpdate()
{
	while (PSMConnected && HMDAllocated && ctrl1Allocated && ctrl2Allocated) {
		
		PSM_Update();

		PSM_GetHmdPosition(hmdList.hmd_id[0], &hmdPos);
		PSM_GetControllerPosition(controllerList.controller_id[0], &ctrl1Pos);
		PSM_GetControllerPosition(controllerList.controller_id[1], &ctrl2Pos);

	}
}

void GetHMDData(THMD* HMD)
{
	if (SerialConnected) {

		Quaternion HMDQuat = SetOffsetQuat(ArduinoData[HMDQW], ArduinoData[HMDQX], ArduinoData[HMDQY], ArduinoData[HMDQZ], HMDOffset);

		HMD->X = hmdPos.x * k_fScalePSMoveAPIToMeters;
		HMD->Y = hmdPos.z * k_fScalePSMoveAPIToMeters;
		HMD->Z = hmdPos.y * k_fScalePSMoveAPIToMeters;

		HMD->qW = HMDQuat.W;
		HMD->qX = HMDQuat.X;
		HMD->qY = HMDQuat.Y;
		HMD->qZ = HMDQuat.Z;
	}
	if ((GetAsyncKeyState(VK_F8) & 0x8000) != 0)
		SetCentering();
}

void GetControllersData(TController* FirstController, TController* SecondController)
{
	if (SerialConnected) {

		Quaternion Ctrl1Quat = SetOffsetQuat(ArduinoData[CTRL1QW], ArduinoData[CTRL1QX], ArduinoData[CTRL1QY], ArduinoData[CTRL1QZ], Ctrl1Offset);
		Quaternion Ctrl2Quat = SetOffsetQuat(ArduinoData[CTRL2QW], ArduinoData[CTRL2QX], ArduinoData[CTRL2QY], ArduinoData[CTRL2QZ], Ctrl2Offset);

		FirstController->X = ctrl1Pos.x * k_fScalePSMoveAPIToMeters;
		FirstController->Y = ctrl1Pos.z * k_fScalePSMoveAPIToMeters;
		FirstController->Z = ctrl1Pos.y * k_fScalePSMoveAPIToMeters;

		FirstController->qW = Ctrl1Quat.W;
		FirstController->qX = Ctrl1Quat.X;
		FirstController->qY = Ctrl1Quat.Y;
		FirstController->qZ = Ctrl1Quat.Z;

		FirstController->Buttons = ArduinoData[CTRL1BTN];
		FirstController->Trigger = ArduinoData[CTRL1TRIGG];
		FirstController->JoyAxisX = ArduinoData[CTRL1AXISX];
		FirstController->JoyAxisY = ArduinoData[CTRL1AXISY];
		FirstController->TrackpY = ArduinoData[CTRL1TRACKY];
		FirstController->vBat = ArduinoData[CTRL1VBAT];

		SecondController->X = ctrl2Pos.x * k_fScalePSMoveAPIToMeters;
		SecondController->Y = ctrl2Pos.z * k_fScalePSMoveAPIToMeters;
		SecondController->Z = ctrl2Pos.y * k_fScalePSMoveAPIToMeters;

		SecondController->qW = Ctrl2Quat.W;
		SecondController->qX = Ctrl2Quat.X;
		SecondController->qY = Ctrl2Quat.Y;
		SecondController->qZ = Ctrl2Quat.Z;

		SecondController->Buttons = ArduinoData[CTRL2BTN];
		SecondController->Trigger = ArduinoData[CTRL2TRIGG];
		SecondController->JoyAxisX = ArduinoData[CTRL2AXISX];
		SecondController->JoyAxisY = ArduinoData[CTRL2AXISY];
		SecondController->TrackpY = ArduinoData[CTRL2TRACKY];
		SecondController->vBat = ArduinoData[CTRL2VBAT];

	}
	else
	{
		FirstController->X = 0.1;
		FirstController->Y = -0.3;
		FirstController->Z = -0.2;

		SecondController->X = -0.1;
		SecondController->Y = -0.3;
		SecondController->Z = -0.2;

		FirstController->qW = 0;
		FirstController->qX = 0;
		FirstController->qY = 0;
		FirstController->qZ = 0;

		SecondController->qW = 0;
		SecondController->qX = 0;
		SecondController->qY = 0;
		SecondController->qZ = 0;

		FirstController->Buttons = 0;
		FirstController->Trigger = 0;
		FirstController->JoyAxisX = 0;
		FirstController->JoyAxisY = 0;
		FirstController->TrackpY = 0;
		FirstController->vBat = 0;

		SecondController->Buttons = 0;
		SecondController->Trigger = 0;
		SecondController->JoyAxisX = 0;
		SecondController->JoyAxisY = 0;
		SecondController->TrackpY = 0;
		SecondController->vBat = 0;
	}
}

bool connectToPSMOVE()
{
	DriverLog("[PsMoveData] connecting to PSM, on ADDRESS %s and PORT %s", PSMOVESERVICE_DEFAULT_ADDRESS, PSMOVESERVICE_DEFAULT_PORT);
	int PSMstatus = PSM_InitializeAsync(PSMOVESERVICE_DEFAULT_ADDRESS, PSMOVESERVICE_DEFAULT_PORT);
	bool bSuccess = (PSMstatus != PSMResult_Error);
	DriverLog("[PsMoveData] PSM status: %d", PSMstatus);
	PSMConnected = bSuccess;
	
	unsigned int data_stream_flags =
		PSMControllerDataStreamFlags::PSMStreamFlags_includePositionData |
		PSMControllerDataStreamFlags::PSMStreamFlags_includePhysicsData |
		PSMControllerDataStreamFlags::PSMStreamFlags_includeCalibratedSensorData |
		PSMControllerDataStreamFlags::PSMStreamFlags_includeRawTrackerData;

	if (PSMConnected) {
		memset(&hmdList, 0, sizeof(PSMHmdList));
		PSM_GetHmdList(&hmdList, PSM_DEFAULT_TIMEOUT);

		memset(&controllerList, 0, sizeof(PSMControllerList));
		PSM_GetControllerList(&controllerList, PSM_DEFAULT_TIMEOUT);

		DriverLog("[PsMoveData] PSM hmdCount: %d", hmdList.count);
		DriverLog("[PsMoveData] PSM ControllerCount: %d", controllerList.count);
	}

	//hmd
	if (hmdList.count > 0) 
	{
		if (PSM_AllocateHmdListener(hmdList.hmd_id[0]) == PSMResult_Success && PSM_StartHmdDataStream(hmdList.hmd_id[0], data_stream_flags, PSM_DEFAULT_TIMEOUT) == PSMResult_Success) 
		{
			DriverLog("HMD Allocated successfully!");
			HMDAllocated = true;
		}
	}



	//controllers

	if (PSM_AllocateControllerListener(controllerList.controller_id[0]) == PSMResult_Success && PSM_StartControllerDataStream(controllerList.controller_id[0], data_stream_flags, PSM_DEFAULT_TIMEOUT) == PSMResult_Success)
	{
		DriverLog("Controller %d allocated successfully!", controllerList.controller_id[0]);
		ctrl1Allocated = true;
	}
	

	if (PSM_AllocateControllerListener(controllerList.controller_id[1]) == PSMResult_Success && PSM_StartControllerDataStream(controllerList.controller_id[1], data_stream_flags, PSM_DEFAULT_TIMEOUT) == PSMResult_Success)
	{
		DriverLog("Controller %d allocated successfully!", controllerList.controller_id[0]);
		ctrl2Allocated = true;
	}
	
		
	if (bSuccess)
		pPSMUpdatethread = new std::thread(PSMUpdate);
	
	return bSuccess;

}

void StartData(int comPort)
{
	if (SerialInit == false) {
		dataCOMPort = comPort;
		SerialInit = true;
		SerialStreamStart();
		connectToPSMOVE();
	}

}
#endif