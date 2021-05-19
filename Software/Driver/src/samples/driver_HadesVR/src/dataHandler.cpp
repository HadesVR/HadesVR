#include "dataHandler.h"

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

#define SUCCESS 0
#define FAILURE 1

void CdataHandler::SetCentering()
{
	HMDOffset.W = HMDData.qW;
	HMDOffset.Y = -HMDData.qY;

	Ctrl1Offset.W = Ctrl1Data.qW;
	Ctrl1Offset.Y = -Ctrl1Data.qY;

	Ctrl2Offset.W = Ctrl2Data.qW;
	Ctrl2Offset.Y = -Ctrl2Data.qY;

	if (Ctrl1Offset.W == 0 && Ctrl1Offset.Y == 0) {
		Ctrl1Offset.W = 1;
		Ctrl1Offset.Y = 0;
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1W_Float, Ctrl1Offset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1Y_Float, Ctrl1Offset.Y);
	}
	else {
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1W_Float, Ctrl1Offset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1Y_Float, Ctrl1Offset.Y);
	}


	if (Ctrl2Offset.W == 0 && Ctrl2Offset.Y == 0) {
		Ctrl2Offset.W = 1;
		Ctrl2Offset.Y = 0;
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2W_Float, Ctrl2Offset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2Y_Float, Ctrl2Offset.Y);
	}
	else {
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2W_Float, Ctrl2Offset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2Y_Float, Ctrl2Offset.Y);
	}


	if (HMDOffset.W == 0 && HMDOffset.Y == 0) {
		HMDOffset.W = 1;
		HMDOffset.Y = 0;
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDW_Float, HMDOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDY_Float, HMDOffset.Y);
	}
	else {
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDW_Float, HMDOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDY_Float, HMDOffset.Y);
	}
}

inline Quaternion SetOffsetQuat(double qW, double qX, double qY, double qZ, Quaternion offsetQuat, Quaternion configOffset)
{
	Quaternion Input;
	Input.W = qW;
	Input.X = qX;
	Input.Y = qY;
	Input.Z = qZ;

	Quaternion inputCal = Quaternion::Normalized(offsetQuat * Input);
	
	Quaternion Output = Quaternion::Normalized(inputCal * configOffset);

	return Output;
}

void CdataHandler::ReadHIDData()
{
	HMDPacket* DataHMD = (HMDPacket*)packet_buffer;
	ControllerPacket* DataCtrl = (ControllerPacket*)packet_buffer;
	int r;
	DriverLog("[HID] ReadHIDData Thread created, HIDConnected Status: %d", HIDConnected);
	while (HIDConnected) {
		r = hid_read(hHID, packet_buffer, 64); //Result should be greater than 0.
		if (r > 0) {
			switch (packet_buffer[1])
			{
			case 1:
				HMDData.qW = DataHMD->HMDQuatW;
				HMDData.qX = DataHMD->HMDQuatX;
				HMDData.qY = DataHMD->HMDQuatY;
				HMDData.qZ = DataHMD->HMDQuatZ;

				Tracker1.qW = (float)(DataHMD->tracker1_QuatW) / 32767;
				Tracker1.qX = (float)(DataHMD->tracker1_QuatX) / 32767;
				Tracker1.qY = (float)(DataHMD->tracker1_QuatY) / 32767;
				Tracker1.qZ = (float)(DataHMD->tracker1_QuatZ) / 32767;
				Tracker1.vBat = (float)(DataHMD->tracker1_vBat) / 255;

				Tracker2.qW = (float)(DataHMD->tracker2_QuatW) / 32767;
				Tracker2.qX = (float)(DataHMD->tracker2_QuatX) / 32767;
				Tracker2.qY = (float)(DataHMD->tracker2_QuatY) / 32767;
				Tracker2.qZ = (float)(DataHMD->tracker2_QuatZ) / 32767;
				Tracker2.vBat = (float)(DataHMD->tracker2_vBat) / 255;

				Tracker3.qW = (float)(DataHMD->tracker3_QuatW) / 32767;
				Tracker3.qX = (float)(DataHMD->tracker3_QuatX) / 32767;
				Tracker3.qY = (float)(DataHMD->tracker3_QuatY) / 32767;
				Tracker3.qZ = (float)(DataHMD->tracker3_QuatZ) / 32767;
				Tracker3.vBat = (float)(DataHMD->tracker3_vBat) / 255;
				break;
			case 2:
				Ctrl1Data.qW = DataCtrl->Ctrl1_QuatW;
				Ctrl1Data.qX = DataCtrl->Ctrl1_QuatX;
				Ctrl1Data.qY = DataCtrl->Ctrl1_QuatY;
				Ctrl1Data.qZ = DataCtrl->Ctrl1_QuatZ;
				Ctrl1Data.Buttons = DataCtrl->Ctrl1_Buttons;
				Ctrl1Data.Trigger = (float)(DataCtrl->Ctrl1_Trigger) / 255;
				Ctrl1Data.JoyAxisX = (float)(DataCtrl->Ctrl1_axisX) / 127;
				Ctrl1Data.JoyAxisY = (float)(DataCtrl->Ctrl1_axisY) / 127;
				Ctrl1Data.TrackpY = (float)(DataCtrl->Ctrl1_trackY) / 127;
				Ctrl1Data.vBat = (float)(DataCtrl->Ctrl1_vBat) / 255;
				Ctrl1Data.FingThumb = (float)(DataCtrl->Ctrl1_THUMB) / 255;
				Ctrl1Data.FingIndex = (float)(DataCtrl->Ctrl1_INDEX) / 255;
				Ctrl1Data.FingMiddl = (float)(DataCtrl->Ctrl1_MIDDLE) / 255;
				Ctrl1Data.FingRing = (float)(DataCtrl->Ctrl1_RING) / 255;
				Ctrl1Data.FingPinky = (float)(DataCtrl->Ctrl1_PINKY) / 255;

				Ctrl2Data.qW = DataCtrl->Ctrl2_QuatW;
				Ctrl2Data.qX = DataCtrl->Ctrl2_QuatX;
				Ctrl2Data.qY = DataCtrl->Ctrl2_QuatY;
				Ctrl2Data.qZ = DataCtrl->Ctrl2_QuatZ;
				Ctrl2Data.Buttons = DataCtrl->Ctrl2_Buttons;
				Ctrl2Data.Trigger = (float)(DataCtrl->Ctrl2_Trigger) / 255;
				Ctrl2Data.JoyAxisX = (float)(DataCtrl->Ctrl2_axisX) / 127;
				Ctrl2Data.JoyAxisY = (float)(DataCtrl->Ctrl2_axisY) / 127;
				Ctrl2Data.TrackpY = (float)(DataCtrl->Ctrl2_trackY) / 127;
				Ctrl2Data.vBat = (float)(DataCtrl->Ctrl2_vBat) / 255;
				Ctrl2Data.FingThumb = (float)(DataCtrl->Ctrl2_THUMB) / 255;
				Ctrl2Data.FingIndex = (float)(DataCtrl->Ctrl2_INDEX) / 255;
				Ctrl2Data.FingMiddl = (float)(DataCtrl->Ctrl2_MIDDLE) / 255;
				Ctrl2Data.FingRing = (float)(DataCtrl->Ctrl2_RING) / 255;
				Ctrl2Data.FingPinky = (float)(DataCtrl->Ctrl2_PINKY) / 255;
				break;
			}
		}
	}
}

void CdataHandler::PSMUpdate()
{
	while (PSMConnected && HMDAllocated && ctrl1Allocated && ctrl2Allocated) {
		
		PSM_Update();

		PSM_GetHmdPosition(hmdList.hmd_id[0], &hmdPos);
		PSM_GetControllerPosition(controllerList.controller_id[0], &ctrl1Pos);
		PSM_GetControllerPosition(controllerList.controller_id[1], &ctrl2Pos);

	}
}

void CdataHandler::GetHMDData(THMD* HMD)
{
	if (HIDConnected) {

		Quaternion HMDQuat = SetOffsetQuat(HMDData.qW, HMDData.qX, HMDData.qY, HMDData.qZ, HMDOffset, HMDConfigOffset);

		if (PSMConnected) {			//PSM POSITION

			HMD->X = hmdPos.x * k_fScalePSMoveAPIToMeters;
			HMD->Y = hmdPos.z * k_fScalePSMoveAPIToMeters;
			HMD->Z = hmdPos.y * k_fScalePSMoveAPIToMeters;
		}
		else {
			HMD->X = 0;
			HMD->Y = 0;
			HMD->Z = 0;
		}

		HMD->qW = HMDQuat.W;
		HMD->qX = HMDQuat.X;
		HMD->qY = HMDQuat.Y;
		HMD->qZ = HMDQuat.Z;

	}
	if ((GetAsyncKeyState(VK_F8) & 0x8000) != 0)
		SetCentering();
}

void CdataHandler::GetControllersData(TController* FirstController, TController* SecondController)
{
	if (HIDConnected) {

		Quaternion Ctrl1Quat = SetOffsetQuat(Ctrl1Data.qW, Ctrl1Data.qX, Ctrl1Data.qY, Ctrl1Data.qZ, Ctrl1Offset, CTRL1ConfigOffset);
		Quaternion Ctrl2Quat = SetOffsetQuat(Ctrl2Data.qW, Ctrl2Data.qX, Ctrl2Data.qY, Ctrl2Data.qZ, Ctrl2Offset, CTRL2ConfigOffset);
		
		FirstController->qW = Ctrl1Quat.W;
		FirstController->qX = Ctrl1Quat.X;
		FirstController->qY = Ctrl1Quat.Y;
		FirstController->qZ = Ctrl1Quat.Z;

		FirstController->Buttons  = Ctrl1Data.Buttons;
		FirstController->Trigger  = Ctrl1Data.Trigger;
		FirstController->JoyAxisX = Ctrl1Data.JoyAxisX;
		FirstController->JoyAxisY = Ctrl1Data.JoyAxisY;
		FirstController->TrackpY  = Ctrl1Data.TrackpY;
		FirstController->vBat     = Ctrl1Data.vBat;

		FirstController->FingThumb = Ctrl1Data.FingThumb;
		FirstController->FingIndex = Ctrl1Data.FingIndex;
		FirstController->FingMiddl = Ctrl1Data.FingMiddl;
		FirstController->FingRing  = Ctrl1Data.FingRing;
		FirstController->FingPinky = Ctrl1Data.FingPinky;


		SecondController->qW = Ctrl2Quat.W;
		SecondController->qX = Ctrl2Quat.X;
		SecondController->qY = Ctrl2Quat.Y;
		SecondController->qZ = Ctrl2Quat.Z;

		SecondController->Buttons  = Ctrl2Data.Buttons;
		SecondController->Trigger  = Ctrl2Data.Trigger;
		SecondController->JoyAxisX = Ctrl2Data.JoyAxisX;
		SecondController->JoyAxisY = Ctrl2Data.JoyAxisY;
		SecondController->TrackpY  = Ctrl2Data.TrackpY;
		SecondController->vBat     = Ctrl2Data.vBat;

		SecondController->FingThumb = Ctrl2Data.FingThumb;
		SecondController->FingIndex = Ctrl2Data.FingIndex;
		SecondController->FingMiddl = Ctrl2Data.FingMiddl;
		SecondController->FingRing  = Ctrl2Data.FingRing;
		SecondController->FingPinky = Ctrl2Data.FingPinky;

		if (PSMConnected) {		//PSM POSITION

			FirstController->X = ctrl1Pos.x * k_fScalePSMoveAPIToMeters;
			FirstController->Y = ctrl1Pos.z * k_fScalePSMoveAPIToMeters;
			FirstController->Z = ctrl1Pos.y * k_fScalePSMoveAPIToMeters;

			SecondController->X = ctrl2Pos.x * k_fScalePSMoveAPIToMeters;
			SecondController->Y = ctrl2Pos.z * k_fScalePSMoveAPIToMeters;
			SecondController->Z = ctrl2Pos.y * k_fScalePSMoveAPIToMeters;

		}
		else {
			FirstController->X = 0.1;
			FirstController->Y = -0.3;
			FirstController->Z = -0.2;

			SecondController->X = -0.1;
			SecondController->Y = -0.3;
			SecondController->Z = -0.2;
		}
	}
}

float CdataHandler::lerp(const float a, const float b, const float f) {
	return a + f * (b - a);
}

bool CdataHandler::connectToPSMOVE()
{
	DriverLog("[PsMoveData] Trying to connect to PSMS on ADDRESS %s and PORT %s", PSMOVESERVICE_DEFAULT_ADDRESS, PSMOVESERVICE_DEFAULT_PORT);
	int PSMstatus = PSM_Initialize(PSMOVESERVICE_DEFAULT_ADDRESS, PSMOVESERVICE_DEFAULT_PORT, 3000);
	bool bSuccess = (PSMstatus == PSMResult_Success);
	DriverLog("[PsMoveData] PSM status: %d", PSMstatus);
	PSMConnected = bSuccess;
	
	unsigned int data_stream_flags =
		PSMControllerDataStreamFlags::PSMStreamFlags_includePositionData |
		PSMControllerDataStreamFlags::PSMStreamFlags_includePhysicsData |
		PSMControllerDataStreamFlags::PSMStreamFlags_includeCalibratedSensorData |
		PSMControllerDataStreamFlags::PSMStreamFlags_includeRawTrackerData;

	if (PSMConnected) {

		DriverLog("[PsMoveData] PSMS connected!");

		memset(&hmdList, 0, sizeof(PSMHmdList));
		PSM_GetHmdList(&hmdList, PSM_DEFAULT_TIMEOUT);

		memset(&controllerList, 0, sizeof(PSMControllerList));
		PSM_GetControllerList(&controllerList, PSM_DEFAULT_TIMEOUT);

		DriverLog("[PsMoveData] PSM hmdCount: %d", hmdList.count);
		DriverLog("[PsMoveData] PSM ControllerCount: %d", controllerList.count);

	//hmd
	if (hmdList.count > 0) 
	{
		if (PSM_AllocateHmdListener(hmdList.hmd_id[0]) == PSMResult_Success && PSM_StartHmdDataStream(hmdList.hmd_id[0], data_stream_flags, PSM_DEFAULT_TIMEOUT) == PSMResult_Success) 
		{
			DriverLog("[PsMoveData] HMD Allocated successfully!");
			HMDAllocated = true;
		}
	}

	//controllers
	if (PSM_AllocateControllerListener(controllerList.controller_id[0]) == PSMResult_Success && PSM_StartControllerDataStream(controllerList.controller_id[0], data_stream_flags, PSM_DEFAULT_TIMEOUT) == PSMResult_Success)
	{
		DriverLog("[PsMoveData] Controller %d allocated successfully!", controllerList.controller_id[0]);
		ctrl1Allocated = true;
	}
	

	if (PSM_AllocateControllerListener(controllerList.controller_id[1]) == PSMResult_Success && PSM_StartControllerDataStream(controllerList.controller_id[1], data_stream_flags, PSM_DEFAULT_TIMEOUT) == PSMResult_Success)
	{
		DriverLog("[PsMoveData] Controller %d allocated successfully!", controllerList.controller_id[1]);
		ctrl2Allocated = true;
	}
	
		
	if (bSuccess)
		pPSMUpdatethread = new std::thread(this->PSMUpdateEnter,this);

	}
	else {
		DriverLog("[PsMoveData] PSMS not connected!");
	}
	return bSuccess;

}

void CdataHandler::StartData(int32_t PID, int32_t VID)
{
	if (HIDInit == false) {
		
		connectToPSMOVE();
		int result;
		result = hid_init(); //Result should be 0.
		if (result) {
			DriverLog("[DataStream] HID init failed.");
		}

		hHID = hid_open((unsigned short)VID, (unsigned short)PID, NULL);
		if (!hHID) {
			DriverLog("[DataStream] Unable to start data stream of device with pid=%d and vid=%d.\n", PID, VID);
			HIDConnected = false;
			return;
		}
		HIDInit = true;
		HIDConnected = true;
		pHIDthread = new std::thread(this->ReadHIDEnter, this);

		CTRL1ConfigOffset = Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller1_PitchOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_YawOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller1_RollOffset_Float) * 3.14159265358979323846 / 180);
		CTRL2ConfigOffset = Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_PitchOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_YawOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_RollOffset_Float) * 3.14159265358979323846 / 180);

		HMDConfigOffset =  Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_PitchOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_YawOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_RollOffset_Float) * 3.14159265358979323846 / 180);

		HMDOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDW_Float);
		HMDOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDY_Float);

		Ctrl1Offset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1W_Float);
		Ctrl1Offset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1Y_Float);

		Ctrl2Offset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2W_Float);
		Ctrl2Offset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2Y_Float);

		if (Ctrl1Offset.W == 0 && Ctrl1Offset.Y == 0) {
			Ctrl1Offset.W = 1;
			Ctrl1Offset.Y = 0;
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1W_Float, Ctrl1Offset.W);
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1Y_Float, Ctrl1Offset.Y);
		}
		if (Ctrl2Offset.W == 0 && Ctrl2Offset.Y == 0) {
			Ctrl2Offset.W = 1;
			Ctrl2Offset.Y = 0;
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2W_Float, Ctrl2Offset.W);
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2Y_Float, Ctrl2Offset.Y);
		}
		if (HMDOffset.W == 0 && HMDOffset.Y == 0) {
			HMDOffset.W = 1;
			HMDOffset.Y = 0;
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDW_Float, HMDOffset.W);
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDY_Float, HMDOffset.Y);
		}

		DriverLog("[Settings] Loaded Calibration settings");
	}
}

void CdataHandler::stopData() {
	hid_close(hHID);
	hid_exit();
	HIDConnected = false;
	HIDInit = false;
}