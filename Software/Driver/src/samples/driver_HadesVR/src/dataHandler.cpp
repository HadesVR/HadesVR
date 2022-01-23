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
	HMDQuaternionPacket* DataHMDQuat = (HMDQuaternionPacket*)packet_buffer;
	HMDRAWPacket* DataHMDRAW = (HMDRAWPacket*)packet_buffer;
	ControllerPacket* DataCtrl = (ControllerPacket*)packet_buffer;
	int r;
	DriverLog("[HID] ReadHIDData Thread created, HIDConnected Status: %d", HIDConnected);
	while (HIDConnected) {
		r = hid_read(hHID, packet_buffer, 64); //Result should be greater than 0.
		if (r > 0) {
			switch (packet_buffer[1])
			{
			case 1:		//HMD quaternion packet
				HMDData.qW = DataHMDQuat->HMDQuatW;
				HMDData.qX = DataHMDQuat->HMDQuatX;
				HMDData.qY = DataHMDQuat->HMDQuatY;
				HMDData.qZ = DataHMDQuat->HMDQuatZ;

				HMDData.Data = DataHMDQuat->HMDData;

				TrackerWaistData.qW = (float)(DataHMDQuat->tracker1_QuatW) / 32767.f;
				TrackerWaistData.qX = (float)(DataHMDQuat->tracker1_QuatX) / 32767.f;
				TrackerWaistData.qY = (float)(DataHMDQuat->tracker1_QuatY) / 32767.f;
				TrackerWaistData.qZ = (float)(DataHMDQuat->tracker1_QuatZ) / 32767.f;
				TrackerWaistData.vBat = (float)(DataHMDQuat->tracker1_vBat) / 255.f;

				TrackerLeftData.qW = (float)(DataHMDQuat->tracker2_QuatW) / 32767.f;
				TrackerLeftData.qX = (float)(DataHMDQuat->tracker2_QuatX) / 32767.f;
				TrackerLeftData.qY = (float)(DataHMDQuat->tracker2_QuatY) / 32767.f;
				TrackerLeftData.qZ = (float)(DataHMDQuat->tracker2_QuatZ) / 32767.f;
				TrackerLeftData.vBat = (float)(DataHMDQuat->tracker2_vBat) / 255.f;

				TrackerRightData.qW = (float)(DataHMDQuat->tracker3_QuatW) / 32767.f;
				TrackerRightData.qX = (float)(DataHMDQuat->tracker3_QuatX) / 32767.f;
				TrackerRightData.qY = (float)(DataHMDQuat->tracker3_QuatY) / 32767.f;
				TrackerRightData.qZ = (float)(DataHMDQuat->tracker3_QuatZ) / 32767.f;
				TrackerRightData.vBat = (float)(DataHMDQuat->tracker3_vBat) / 255.f;
				break;

			case 2:		//Controller quaternion packet

				RightCtrlData.qW = (float)(DataCtrl->Ctrl1_QuatW) / 32767.f;
				RightCtrlData.qX = (float)(DataCtrl->Ctrl1_QuatX) / 32767.f;
				RightCtrlData.qY = (float)(DataCtrl->Ctrl1_QuatY) / 32767.f;
				RightCtrlData.qZ = (float)(DataCtrl->Ctrl1_QuatZ) / 32767.f;

				RightCtrlData.accelX = (float)(DataCtrl->Ctrl1_AccelX) / 2048.f;
				RightCtrlData.accelY = (float)(DataCtrl->Ctrl1_AccelY) / 2048.f;
				RightCtrlData.accelZ = (float)(DataCtrl->Ctrl1_AccelZ) / 2048.f;

				if (ctrlAccelEnable) {
					CalcAccelPosition(RightCtrlData.qW, RightCtrlData.qX, RightCtrlData.qZ, RightCtrlData.qY, RightCtrlData.accelX, RightCtrlData.accelY, RightCtrlData.accelZ, ctrlRightPosData);
				}
				
				RightCtrlData.Data = DataCtrl->Ctrl1_Data;
				RightCtrlData.Buttons = DataCtrl->Ctrl1_Buttons;

				RightCtrlData.Trigger = (float)(DataCtrl->Ctrl1_Trigger) / 255.f;
				RightCtrlData.JoyAxisX = (float)(DataCtrl->Ctrl1_axisX) / 127.f;
				RightCtrlData.JoyAxisY = (float)(DataCtrl->Ctrl1_axisY) / 127.f;
				RightCtrlData.TrackpY = (float)(DataCtrl->Ctrl1_trackY) / 127.f;
				RightCtrlData.vBat = (float)(DataCtrl->Ctrl1_vBat) / 255.f;
				RightCtrlData.FingThumb = (float)(DataCtrl->Ctrl1_THUMB) / 255.f;
				RightCtrlData.FingIndex = (float)(DataCtrl->Ctrl1_INDEX) / 255.f;
				RightCtrlData.FingMiddl = (float)(DataCtrl->Ctrl1_MIDDLE) / 255.f;
				RightCtrlData.FingRing = (float)(DataCtrl->Ctrl1_RING) / 255.f;
				RightCtrlData.FingPinky = (float)(DataCtrl->Ctrl1_PINKY) / 255.f;
				

				LeftCtrlData.qW = (float)(DataCtrl->Ctrl2_QuatW) / 32767.f;
				LeftCtrlData.qX = (float)(DataCtrl->Ctrl2_QuatX) / 32767.f;
				LeftCtrlData.qY = (float)(DataCtrl->Ctrl2_QuatY) / 32767.f;
				LeftCtrlData.qZ = (float)(DataCtrl->Ctrl2_QuatZ) / 32767.f;

				
				LeftCtrlData.accelX = (float)(DataCtrl->Ctrl2_AccelX) / 2048.f;
				LeftCtrlData.accelY = (float)(DataCtrl->Ctrl2_AccelY) / 2048.f;
				LeftCtrlData.accelZ = (float)(DataCtrl->Ctrl2_AccelZ) / 2048.f;
				
				if (ctrlAccelEnable) {
					CalcAccelPosition(LeftCtrlData.qW, LeftCtrlData.qX, LeftCtrlData.qY, LeftCtrlData.qZ, LeftCtrlData.accelX, LeftCtrlData.accelY, LeftCtrlData.accelZ, ctrlLeftPosData);
				}

				LeftCtrlData.Data = DataCtrl->Ctrl2_Data;
				LeftCtrlData.Buttons = DataCtrl->Ctrl2_Buttons;

				LeftCtrlData.Trigger = (float)(DataCtrl->Ctrl2_Trigger) / 255.f;
				LeftCtrlData.JoyAxisX = (float)(DataCtrl->Ctrl2_axisX) / 127.f;
				LeftCtrlData.JoyAxisY = (float)(DataCtrl->Ctrl2_axisY) / 127.f;
				LeftCtrlData.TrackpY = (float)(DataCtrl->Ctrl2_trackY) / 127.f;
				LeftCtrlData.vBat = (float)(DataCtrl->Ctrl2_vBat) / 255.f;
				LeftCtrlData.FingThumb = (float)(DataCtrl->Ctrl2_THUMB) / 255.f;
				LeftCtrlData.FingIndex = (float)(DataCtrl->Ctrl2_INDEX) / 255.f;
				LeftCtrlData.FingMiddl = (float)(DataCtrl->Ctrl2_MIDDLE) / 255.f;
				LeftCtrlData.FingRing = (float)(DataCtrl->Ctrl2_RING) / 255.f;
				LeftCtrlData.FingPinky = (float)(DataCtrl->Ctrl2_PINKY) / 255.f;
				
				break;

			case 3:		//hmd IMU packet

				if (!orientationFilterInit) {		//init filter
					HMDfilter.begin();
					HMDfilter.setBeta(2.f);
					DriverLog("[Madgwick] Revving up the filter and redlining it to a beta of 2");
					orientationFilterInit = true;
				}

				if (readsFromInit < 2000) {
					readsFromInit++;
					if (readsFromInit == 2000) {
						HMDfilter.setBeta(filterBeta);
						DriverLog("[Madgwick] first 2000 readings done! switching to more accurate beta value. of %f", filterBeta);
					}
					if (PSMConnected) {
						ResetPos(true);
					}
				}

				float accX = (float)(DataHMDRAW->AccX) / 2048;
				float accY = (float)(DataHMDRAW->AccY) / 2048;
				float accZ = (float)(DataHMDRAW->AccZ) / 2048;

				//get data and scale it properly. Then update the filter.
				HMDfilter.update((float)(DataHMDRAW->GyroX / 16), (float)(DataHMDRAW->GyroY / 16), (float)(DataHMDRAW->GyroZ / 16), 
					accX, accY, accZ,
					(float)(DataHMDRAW->MagX / 5), (float)(DataHMDRAW->MagY / 5), (float)(DataHMDRAW->MagZ / 5));

				//Apply rotation to the HMD
				float quatW = HMDfilter.getQuatW();
				float quatX = HMDfilter.getQuatX();
				float quatY = HMDfilter.getQuatY();
				float quatZ = HMDfilter.getQuatZ();

				HMDData.qW = quatW;
				HMDData.qX = quatY;
				HMDData.qY = quatZ;
				HMDData.qZ = quatX;

				CalcAccelPosition(quatW, quatX, quatY, quatZ, accX, accY, accZ, hmdPosData);

				HMDData.Data = DataHMDRAW->HMDData;

				TrackerWaistData.qW = (float)(DataHMDRAW->tracker1_QuatW) / 32767;
				TrackerWaistData.qX = (float)(DataHMDRAW->tracker1_QuatX) / 32767;
				TrackerWaistData.qY = (float)(DataHMDRAW->tracker1_QuatY) / 32767;
				TrackerWaistData.qZ = (float)(DataHMDRAW->tracker1_QuatZ) / 32767;
				TrackerWaistData.vBat = (float)(DataHMDRAW->tracker1_vBat) / 255;

				TrackerLeftData.qW = (float)(DataHMDRAW->tracker2_QuatW) / 32767;
				TrackerLeftData.qX = (float)(DataHMDRAW->tracker2_QuatX) / 32767;
				TrackerLeftData.qY = (float)(DataHMDRAW->tracker2_QuatY) / 32767;
				TrackerLeftData.qZ = (float)(DataHMDRAW->tracker2_QuatZ) / 32767;
				TrackerLeftData.vBat = (float)(DataHMDRAW->tracker2_vBat) / 255;

				TrackerRightData.qW = (float)(DataHMDRAW->tracker3_QuatW) / 32767;
				TrackerRightData.qX = (float)(DataHMDRAW->tracker3_QuatX) / 32767;
				TrackerRightData.qY = (float)(DataHMDRAW->tracker3_QuatY) / 32767;
				TrackerRightData.qZ = (float)(DataHMDRAW->tracker3_QuatZ) / 32767;
				TrackerRightData.vBat = (float)(DataHMDRAW->tracker3_vBat) / 255;
				break;
			}
		}
	}
}

void CdataHandler::PSMUpdate()
{
	std::chrono::steady_clock::time_point lastPSMSUpdate;

	while (PSMConnected) {
		
		PSM_Update();

		auto now = std::chrono::high_resolution_clock::now();
		deltatime = std::chrono::duration_cast<std::chrono::microseconds>(now - lastPSMSUpdate).count() / 1000000.0f;
		lastPSMSUpdate = now;

		if (HMDAllocated) {
			PSM_GetHmdPosition(hmdList.hmd_id[0], &psmHmdPos);

			float HMDposX = psmHmdPos.x * k_fScalePSMoveAPIToMeters;
			float HMDposY = psmHmdPos.z * k_fScalePSMoveAPIToMeters;
			float HMDposZ = psmHmdPos.y * k_fScalePSMoveAPIToMeters;

			CalcTrackedPos(hmdPosData, HMDposX, HMDposY, HMDposZ, HMDSmoothK);

		}
		if (ctrl1Allocated) {
			PSM_GetControllerPosition(controllerList.controller_id[0], &psmCtrlRightPos);

			float CtrlRightPosX = psmCtrlRightPos.x * k_fScalePSMoveAPIToMeters;
			float CtrlRightPosY = psmCtrlRightPos.z * k_fScalePSMoveAPIToMeters;
			float CtrlRightPosZ = psmCtrlRightPos.y * k_fScalePSMoveAPIToMeters;

			CalcTrackedPos(ctrlRightPosData, CtrlRightPosX, CtrlRightPosY, CtrlRightPosZ, ContSmoothK);

		}
		if (ctrl2Allocated) {
			PSM_GetControllerPosition(controllerList.controller_id[1], &psmCtrlLeftPos);

			float CtrlLeftPosX = psmCtrlLeftPos.x * k_fScalePSMoveAPIToMeters;
			float CtrlLeftPosY = psmCtrlLeftPos.z * k_fScalePSMoveAPIToMeters;
			float CtrlLeftPosZ = psmCtrlLeftPos.y * k_fScalePSMoveAPIToMeters;

			CalcTrackedPos(ctrlLeftPosData, CtrlLeftPosX, CtrlLeftPosY, CtrlLeftPosZ, ContSmoothK);
			
		}
		//no need to update this as often as we do.
		std::this_thread::sleep_for(std::chrono::milliseconds(psmsMillisecondPeriod));
	}
}

void CdataHandler::GetHMDData(THMD* HMD)
{
	if (HIDConnected) {

		Quaternion HMDQuat = SetOffsetQuat(HMDData.qW, HMDData.qX, HMDData.qY, HMDData.qZ, HMDOffset, HMDConfigOffset);

		if (PSMConnected) {			//PSM POSITION

			HMD->X = hmdPosData.posX;
			HMD->Y = hmdPosData.posY;
			HMD->Z = hmdPosData.posZ;
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
	if ((GetAsyncKeyState(VK_F8) & 0x8000) != 0) {
		SetCentering();
	}
		
	if ((GetAsyncKeyState(VK_F9) & 0x8000) != 0) {
		ResetPos(false);
	}
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

			RightController->X = ctrlRightPosData.posX;
			RightController->Y = ctrlRightPosData.posY;
			RightController->Z = ctrlRightPosData.posZ;

			LeftController->X = ctrlLeftPosData.posX;
			LeftController->Y = ctrlLeftPosData.posY;
			LeftController->Z = ctrlLeftPosData.posZ;
			
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

void CdataHandler::CalcAccelPosition(float quatW, float quatX, float quatY, float quatZ, float accelX, float accelY, float accelZ, PosData &pos) {
	
	//get time delta
	auto now = std::chrono::high_resolution_clock::now();
	deltatime = std::chrono::duration_cast<std::chrono::microseconds>(now - pos.lastUpdate).count() / 1000000.0f;
	pos.lastUpdate = now;

	//Rotate gravity vector https://web.archive.org/web/20121004000626/http://www.varesano.net/blog/fabio/simple-gravity-compensation-9-dom-imus
	float gx = (2.0f * (quatX * quatZ - quatW * quatY));
	float gy = (2.0f * (quatW * quatX + quatY * quatZ));
	float gz = (quatW * quatW - quatX * quatX - quatY * quatY + quatZ * quatZ);

	float lin_ax = accelX - gx;
	float lin_ay = accelY - gy;
	float lin_az = accelZ - gz;

	//convert to m/s^2
	lin_ax *= 9.80665f;
	lin_ay *= 9.80665f;
	lin_az *= 9.80665f;

	//integrate to get velocity
	pos.vx += (lin_ay * deltatime);
	pos.vy += (lin_ax * deltatime);
	pos.vz += (lin_az * deltatime);

	//integrate to get position
	pos.posX = ((pos.vx + pos.oldvX) * 0.5f) * deltatime + pos.oldPosX;
	pos.posY = ((pos.vy + pos.oldvY) * 0.5f) * deltatime + pos.oldPosY;
	pos.posZ = ((pos.vz + pos.oldvZ) * 0.5f) * deltatime + pos.oldPosZ;

	//decay
	pos.vx *= 0.8f;
	pos.vy *= 0.8f;
	pos.vz *= 0.8f;

	pos.oldvX = pos.vx;
	pos.oldvY = pos.vy;
	pos.oldvZ = pos.vz;

	pos.oldPosX = pos.posX;
	pos.oldPosY = pos.posY;
	pos.oldPosZ = pos.posZ;
}

void CdataHandler::CalcTrackedPos(PosData &pos, float x, float y, float z, float smooth) {

	float diffX = pos.posX - x;
	float diffY = pos.posY - y;
	float diffZ = pos.posZ - z;

	float fabsfdiffX = fabsf(diffX);
	float fabsfdiffY = fabsf(diffY);
	float fabsfdiffZ = fabsf(diffZ);

	//I have no idea, this is technically a high pass filter but think of it as a low pass one, plug "f\left(x\right)=\left(1-e^{-kx}\right)" into desmos to see how it works, let k be between 1 and 100.
	float outX = diffX * (1 - pow(E_Num, -smooth * fabsfdiffX));
	float outY = diffY * (1 - pow(E_Num, -smooth * fabsfdiffY));
	float outZ = diffZ * (1 - pow(E_Num, -smooth * fabsfdiffZ));
	
	//update position
	pos.posX += -outX;
	pos.posY += -outY;
	pos.posZ += -outZ;

	pos.oldPosX = pos.posX;
	pos.oldPosY = pos.posY;
	pos.oldPosZ = pos.posZ;


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
		filterBeta = vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_FilterBeta_Float);

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

		//get psms update rate
		psmsUpdateRate = vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_PSMS_UPDATE_RATE_Int32);
		psmsMillisecondPeriod = (float)((1.f / psmsUpdateRate) * 1000.f);
		DriverLog("[Settings] PSMS update rate in hz: %i, with a period of %i milliseconds.", psmsUpdateRate, psmsMillisecondPeriod);
		//use ctrl accelerometers?
		ctrlAccelEnable = vr::VRSettings()->GetBool(k_pch_Controllers_Section, k_pch_Controller_AccelEnable_Bool);

		//get tracker update rate and smoothness thing
		ContSmoothK = vr::VRSettings()->GetFloat(k_pch_Driver_Section, k_pch_TRACKER_SMOOTH_CTRL_Float);
		HMDSmoothK = vr::VRSettings()->GetFloat(k_pch_Driver_Section, k_pch_TRACKER_SMOOTH_HMD_Float);

		if (ctrlAccelEnable) {
			ContSmoothK *= 0.1f;
		}
		//set initial states for controllers and hmd.
		ResetPos(false);

		connectToPSMOVE();
	}
}

void CdataHandler::ResetPos(bool hmdOnly) {
	if (!hmdOnly) {
		ctrlRightPosData.posX = 0.1f;
		ctrlRightPosData.posY = -0.3f;
		ctrlRightPosData.posZ = -0.2f;
		ctrlRightPosData.oldPosX = 0.1f;
		ctrlRightPosData.oldPosY = -0.3f;
		ctrlRightPosData.oldPosZ = -0.2f;
		ctrlRightPosData.vx = 0.f;
		ctrlRightPosData.vy = 0.f;
		ctrlRightPosData.vz = 0.f;
		ctrlRightPosData.oldvX = 0.f;
		ctrlRightPosData.oldvY = 0.f;
		ctrlRightPosData.oldvZ = 0.f;

		ctrlLeftPosData.posX = 0.f;
		ctrlLeftPosData.posY = 0.f;
		ctrlLeftPosData.posZ = 0.f;
		ctrlLeftPosData.oldPosX = 0.f;
		ctrlLeftPosData.oldPosY = 0.f;
		ctrlLeftPosData.oldPosZ = 0.f;
		ctrlLeftPosData.vx = 0.f;
		ctrlLeftPosData.vy = 0.f;
		ctrlLeftPosData.vz = 0.f;
		ctrlLeftPosData.oldvX = 0.f;
		ctrlLeftPosData.oldvY = 0.f;
		ctrlLeftPosData.oldvZ = 0.f;
	}
	hmdPosData.posX = 0.f;
	hmdPosData.posY = 0.f;
	hmdPosData.posZ = 0.f;
	hmdPosData.oldPosX = 0.f;
	hmdPosData.oldPosY = 0.f;
	hmdPosData.oldPosZ = 0.f;
	hmdPosData.vx = 0.f;
	hmdPosData.vy = 0.f;
	hmdPosData.vz = 0.f;
	hmdPosData.oldvX = 0.f;
	hmdPosData.oldvY = 0.f;
	hmdPosData.oldvZ = 0.f;
}

void CdataHandler::stopData() {
	hid_close(hHID);
	hid_exit();
	HIDConnected = false;
	HIDInit = false;
}