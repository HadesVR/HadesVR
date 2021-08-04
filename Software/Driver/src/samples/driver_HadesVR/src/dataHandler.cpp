#include "dataHandler.h"

void CdataHandler::SetCentering()
{
	/// <summary>
	/// HMD
	/// </summary>

	HMDOffset.W = HMDData.qW;
	HMDOffset.Y = -HMDData.qY;

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

	/// <summary>
	///				Controllers
	/// </summary>

	RightCtrlOffset.W = RightCtrlData.qW;
	RightCtrlOffset.Y = -RightCtrlData.qY;

	LeftCtrlOffset.W = LeftCtrlData.qW;
	LeftCtrlOffset.Y = -LeftCtrlData.qY;

	if (RightCtrlOffset.W == 0 && RightCtrlOffset.Y == 0) {
		RightCtrlOffset.W = 1;
		RightCtrlOffset.Y = 0;
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1W_Float, RightCtrlOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1Y_Float, RightCtrlOffset.Y);
	}
	else {
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1W_Float, RightCtrlOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1Y_Float, RightCtrlOffset.Y);
	}

	if (LeftCtrlOffset.W == 0 && LeftCtrlOffset.Y == 0) {
		LeftCtrlOffset.W = 1;
		LeftCtrlOffset.Y = 0;
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2W_Float, LeftCtrlOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2Y_Float, LeftCtrlOffset.Y);
	}
	else {
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2W_Float, LeftCtrlOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2Y_Float, LeftCtrlOffset.Y);
	}

	/// <summary>
	///					Trackers
	/// </summary>

	WaistTrackerOffset.W = TrackerWaistData.qW;
	WaistTrackerOffset.Y = -TrackerWaistData.qY;

	LeftTrackerOffset.W = TrackerLeftData.qW;
	LeftTrackerOffset.Y = -TrackerLeftData.qY;

	RightTrackerOffset.W = TrackerRightData.qW;
	RightTrackerOffset.Y = -TrackerRightData.qY;

	if (WaistTrackerOffset.W == 0 && WaistTrackerOffset.Y == 0) {
		WaistTrackerOffset.W = 1;
		WaistTrackerOffset.Y = 0;
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKWaistW_Float, WaistTrackerOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKWaistY_Float, WaistTrackerOffset.Y);
	}
	else {
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKWaistW_Float, WaistTrackerOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKWaistY_Float, WaistTrackerOffset.Y);
	}

	if (LeftTrackerOffset.W == 0 && LeftTrackerOffset.Y == 0) {
		LeftTrackerOffset.W = 1;
		LeftTrackerOffset.Y = 0;
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKLeftW_Float, LeftTrackerOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKLeftY_Float, LeftTrackerOffset.Y);
	}
	else {
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKLeftW_Float, LeftTrackerOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKLeftY_Float, LeftTrackerOffset.Y);
	}

	if (RightTrackerOffset.W == 0 && RightTrackerOffset.Y == 0) {
		RightTrackerOffset.W = 1;
		RightTrackerOffset.Y = 0;
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKRightW_Float, RightTrackerOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKRightY_Float, RightTrackerOffset.Y);
	}
	else {
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKRightW_Float, RightTrackerOffset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKRightY_Float, RightTrackerOffset.Y);
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

				TrackerWaistData.qW = (float)(DataHMD->tracker1_QuatW) / 32767;
				TrackerWaistData.qX = (float)(DataHMD->tracker1_QuatX) / 32767;
				TrackerWaistData.qY = (float)(DataHMD->tracker1_QuatY) / 32767;
				TrackerWaistData.qZ = (float)(DataHMD->tracker1_QuatZ) / 32767;
				TrackerWaistData.vBat = (float)(DataHMD->tracker1_vBat) / 255;

				TrackerLeftData.qW = (float)(DataHMD->tracker2_QuatW) / 32767;
				TrackerLeftData.qX = (float)(DataHMD->tracker2_QuatX) / 32767;
				TrackerLeftData.qY = (float)(DataHMD->tracker2_QuatY) / 32767;
				TrackerLeftData.qZ = (float)(DataHMD->tracker2_QuatZ) / 32767;
				TrackerLeftData.vBat = (float)(DataHMD->tracker2_vBat) / 255;

				TrackerRightData.qW = (float)(DataHMD->tracker3_QuatW) / 32767;
				TrackerRightData.qX = (float)(DataHMD->tracker3_QuatX) / 32767;
				TrackerRightData.qY = (float)(DataHMD->tracker3_QuatY) / 32767;
				TrackerRightData.qZ = (float)(DataHMD->tracker3_QuatZ) / 32767;
				TrackerRightData.vBat = (float)(DataHMD->tracker3_vBat) / 255;
				break;
			case 2:
				RightCtrlData.qW = (float)(DataCtrl->Ctrl1_QuatW) / 32767;
				RightCtrlData.qX = (float)(DataCtrl->Ctrl1_QuatX) / 32767;
				RightCtrlData.qY = (float)(DataCtrl->Ctrl1_QuatY) / 32767;
				RightCtrlData.qZ = (float)(DataCtrl->Ctrl1_QuatZ) / 32767;
				RightCtrlData.Buttons = DataCtrl->Ctrl1_Buttons;
				RightCtrlData.Trigger = (float)(DataCtrl->Ctrl1_Trigger) / 255;
				RightCtrlData.JoyAxisX = (float)(DataCtrl->Ctrl1_axisX) / 127;
				RightCtrlData.JoyAxisY = (float)(DataCtrl->Ctrl1_axisY) / 127;
				RightCtrlData.TrackpY = (float)(DataCtrl->Ctrl1_trackY) / 127;
				RightCtrlData.vBat = (float)(DataCtrl->Ctrl1_vBat) / 255;
				RightCtrlData.FingThumb = (float)(DataCtrl->Ctrl1_THUMB) / 255;
				RightCtrlData.FingIndex = (float)(DataCtrl->Ctrl1_INDEX) / 255;
				RightCtrlData.FingMiddl = (float)(DataCtrl->Ctrl1_MIDDLE) / 255;
				RightCtrlData.FingRing = (float)(DataCtrl->Ctrl1_RING) / 255;
				RightCtrlData.FingPinky = (float)(DataCtrl->Ctrl1_PINKY) / 255;

				LeftCtrlData.qW = (float)(DataCtrl->Ctrl2_QuatW) / 32767;
				LeftCtrlData.qX = (float)(DataCtrl->Ctrl2_QuatX) / 32767;
				LeftCtrlData.qY = (float)(DataCtrl->Ctrl2_QuatY) / 32767;
				LeftCtrlData.qZ = (float)(DataCtrl->Ctrl2_QuatZ) / 32767;
				LeftCtrlData.Buttons = DataCtrl->Ctrl2_Buttons;
				LeftCtrlData.Trigger = (float)(DataCtrl->Ctrl2_Trigger) / 255;
				LeftCtrlData.JoyAxisX = (float)(DataCtrl->Ctrl2_axisX) / 127;
				LeftCtrlData.JoyAxisY = (float)(DataCtrl->Ctrl2_axisY) / 127;
				LeftCtrlData.TrackpY = (float)(DataCtrl->Ctrl2_trackY) / 127;
				LeftCtrlData.vBat = (float)(DataCtrl->Ctrl2_vBat) / 255;
				LeftCtrlData.FingThumb = (float)(DataCtrl->Ctrl2_THUMB) / 255;
				LeftCtrlData.FingIndex = (float)(DataCtrl->Ctrl2_INDEX) / 255;
				LeftCtrlData.FingMiddl = (float)(DataCtrl->Ctrl2_MIDDLE) / 255;
				LeftCtrlData.FingRing = (float)(DataCtrl->Ctrl2_RING) / 255;
				LeftCtrlData.FingPinky = (float)(DataCtrl->Ctrl2_PINKY) / 255;
				break;
			}
		}
	}
}

void CdataHandler::PSMUpdate()
{
	while (PSMConnected) {
		
		PSM_Update();

		if (HMDAllocated) {
			PSM_GetHmdPosition(hmdList.hmd_id[0], &hmdPos);
		}
		if (ctrl1Allocated) {
			PSM_GetControllerPosition(controllerList.controller_id[0], &ctrlRightPos);
		}
		if (ctrl2Allocated) {
			PSM_GetControllerPosition(controllerList.controller_id[1], &ctrlLeftPos);
		}

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

void CdataHandler::GetControllersData(TController* RightController, TController* LeftController)
{
	if (HIDConnected) {

		Quaternion CtrlRightQuat = SetOffsetQuat(RightCtrlData.qW, RightCtrlData.qX, RightCtrlData.qY, RightCtrlData.qZ, RightCtrlOffset, CTRL1ConfigOffset);
		Quaternion CtrlLeftQuat = SetOffsetQuat(LeftCtrlData.qW, LeftCtrlData.qX, LeftCtrlData.qY, LeftCtrlData.qZ, LeftCtrlOffset, CTRL2ConfigOffset);
		
		RightController->qW = CtrlRightQuat.W;
		RightController->qX = CtrlRightQuat.X;
		RightController->qY = CtrlRightQuat.Y;
		RightController->qZ = CtrlRightQuat.Z;

		RightController->Buttons  = RightCtrlData.Buttons;
		RightController->Trigger  = RightCtrlData.Trigger;
		RightController->JoyAxisX = RightCtrlData.JoyAxisX;
		RightController->JoyAxisY = RightCtrlData.JoyAxisY;
		RightController->TrackpY  = RightCtrlData.TrackpY;
		RightController->vBat     = RightCtrlData.vBat;

		RightController->FingThumb = RightCtrlData.FingThumb;
		RightController->FingIndex = RightCtrlData.FingIndex;
		RightController->FingMiddl = RightCtrlData.FingMiddl;
		RightController->FingRing  = RightCtrlData.FingRing;
		RightController->FingPinky = RightCtrlData.FingPinky;


		LeftController->qW = CtrlLeftQuat.W;
		LeftController->qX = CtrlLeftQuat.X;
		LeftController->qY = CtrlLeftQuat.Y;
		LeftController->qZ = CtrlLeftQuat.Z;

		LeftController->Buttons  = LeftCtrlData.Buttons;
		LeftController->Trigger  = LeftCtrlData.Trigger;
		LeftController->JoyAxisX = LeftCtrlData.JoyAxisX;
		LeftController->JoyAxisY = LeftCtrlData.JoyAxisY;
		LeftController->TrackpY  = LeftCtrlData.TrackpY;
		LeftController->vBat     = LeftCtrlData.vBat;

		LeftController->FingThumb = LeftCtrlData.FingThumb;
		LeftController->FingIndex = LeftCtrlData.FingIndex;
		LeftController->FingMiddl = LeftCtrlData.FingMiddl;
		LeftController->FingRing  = LeftCtrlData.FingRing;
		LeftController->FingPinky = LeftCtrlData.FingPinky;

		if (PSMConnected) {		//PSM POSITION

			RightController->X = ctrlRightPos.x * k_fScalePSMoveAPIToMeters;
			RightController->Y = ctrlRightPos.z * k_fScalePSMoveAPIToMeters;
			RightController->Z = ctrlRightPos.y * k_fScalePSMoveAPIToMeters;

			LeftController->X = ctrlLeftPos.x * k_fScalePSMoveAPIToMeters;
			LeftController->Y = ctrlLeftPos.z * k_fScalePSMoveAPIToMeters;
			LeftController->Z = ctrlLeftPos.y * k_fScalePSMoveAPIToMeters;

		}
		else {
			RightController->X = 0.1;
			RightController->Y = -0.3;
			RightController->Z = -0.2;

			LeftController->X = -0.1;
			LeftController->Y = -0.3;
			LeftController->Z = -0.2;
		}
	}
}

void CdataHandler::GetTrackersData(TTracker* waistTracker, TTracker* leftTracker, TTracker* rightTracker) 
{
	if (HIDConnected) {
		Quaternion TRKWaistQuat = SetOffsetQuat(TrackerWaistData.qW, TrackerWaistData.qX, TrackerWaistData.qY, TrackerWaistData.qZ, WaistTrackerOffset, Quaternion::Identity());
		Quaternion TRKLeftQuat = SetOffsetQuat(TrackerLeftData.qW, TrackerLeftData.qX, TrackerLeftData.qY, TrackerLeftData.qZ, LeftTrackerOffset, Quaternion::Identity());
		Quaternion TRKRightQuat = SetOffsetQuat(TrackerRightData.qW, TrackerRightData.qX, TrackerRightData.qY, TrackerRightData.qZ, RightTrackerOffset, Quaternion::Identity());

		waistTracker->qW = TRKWaistQuat.W;
		waistTracker->qX = TRKWaistQuat.X;
		waistTracker->qY = TRKWaistQuat.Y;
		waistTracker->qZ = TRKWaistQuat.Z;
		waistTracker->vBat = TrackerWaistData.vBat;

		leftTracker->qW = TRKLeftQuat.W;
		leftTracker->qX = TRKLeftQuat.X;
		leftTracker->qY = TRKLeftQuat.Y;
		leftTracker->qZ = TRKLeftQuat.Z;
		leftTracker->vBat = TrackerLeftData.vBat;

		rightTracker->qW = TRKRightQuat.W;
		rightTracker->qX = TRKRightQuat.X;
		rightTracker->qY = TRKRightQuat.Y;
		rightTracker->qZ = TRKRightQuat.Z;
		rightTracker->vBat = TrackerRightData.vBat;
	}

	if (PSMConnected) {
		waistTracker->X = 0;
		waistTracker->Y = 0;
		waistTracker->Z = 0;
											//todo: psm stuff
		leftTracker->X = 0;
		leftTracker->Y = 0;
		leftTracker->Z = 0;

		rightTracker->X = 0;
		rightTracker->Y = 0;
		rightTracker->Z = 0;
	}
	else {
		waistTracker->X = 0;
		waistTracker->Y = 0;
		waistTracker->Z = 0;

		leftTracker->X = 0;
		leftTracker->Y = 0;
		leftTracker->Z = 0;

		rightTracker->X = 0;
		rightTracker->Y = 0;
		rightTracker->Z = 0;
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


		HMDConfigOffset = Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_PitchOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_YawOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_RollOffset_Float) * 3.14159265358979323846 / 180);

		CTRL1ConfigOffset = Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller1_PitchOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_YawOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller1_RollOffset_Float) * 3.14159265358979323846 / 180);
		CTRL2ConfigOffset = Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_PitchOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_YawOffset_Float) * 3.14159265358979323846 / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_RollOffset_Float) * 3.14159265358979323846 / 180);
		

		HMDOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDW_Float);
		HMDOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDY_Float);

		RightCtrlOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1W_Float);
		RightCtrlOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1Y_Float);

		LeftCtrlOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2W_Float);
		LeftCtrlOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2Y_Float);

		WaistTrackerOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKWaistW_Float);
		WaistTrackerOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKWaistY_Float);

		LeftTrackerOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKLeftW_Float);
		LeftTrackerOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKLeftY_Float);

		RightTrackerOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKRightW_Float);
		RightTrackerOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKRightY_Float);


		if (HMDOffset.W == 0 && HMDOffset.Y == 0) {
			HMDOffset.W = 1;
			HMDOffset.Y = 0;
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDW_Float, HMDOffset.W);
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDY_Float, HMDOffset.Y);
		}
		if (RightCtrlOffset.W == 0 && RightCtrlOffset.Y == 0) {
			RightCtrlOffset.W = 1;
			RightCtrlOffset.Y = 0;
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1W_Float, RightCtrlOffset.W);
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT1Y_Float, RightCtrlOffset.Y);
		}
		if (LeftCtrlOffset.W == 0 && LeftCtrlOffset.Y == 0) {
			LeftCtrlOffset.W = 1;
			LeftCtrlOffset.Y = 0;
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2W_Float, LeftCtrlOffset.W);
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONT2Y_Float, LeftCtrlOffset.Y);
		}
		if (WaistTrackerOffset.W == 0 && WaistTrackerOffset.Y == 0) {
			WaistTrackerOffset.W = 1;
			WaistTrackerOffset.Y = 0;
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKWaistW_Float, WaistTrackerOffset.W);
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKWaistY_Float, WaistTrackerOffset.Y);
		}
		if (LeftTrackerOffset.W == 0 && LeftTrackerOffset.Y == 0) {
			LeftTrackerOffset.W = 1;
			LeftTrackerOffset.Y = 0;
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKLeftW_Float, LeftTrackerOffset.W);
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKLeftY_Float, LeftTrackerOffset.Y);
		}
		if (RightTrackerOffset.W == 0 && RightTrackerOffset.Y == 0) {
			RightTrackerOffset.W = 1;
			RightTrackerOffset.Y = 0;
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKRightW_Float, RightTrackerOffset.W);
			vr::VRSettings()->SetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKRightY_Float, RightTrackerOffset.Y);
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