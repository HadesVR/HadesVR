#include "dataHandler.h"

/**
	 Reads HID data and separates the incoming data packet into HMD, controller or tracker data.
*/
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
				HMDData.Rotation.W = DataHMDQuat->HMDQuatW;
				HMDData.Rotation.X = DataHMDQuat->HMDQuatX;
				HMDData.Rotation.Y = DataHMDQuat->HMDQuatY;
				HMDData.Rotation.Z = DataHMDQuat->HMDQuatZ;

				HMDData.Data = DataHMDQuat->HMDData;

				TrackerWaistData.Rotation.W = (float)(DataHMDQuat->tracker1_QuatW) / 32767.f;
				TrackerWaistData.Rotation.X = (float)(DataHMDQuat->tracker1_QuatX) / 32767.f;
				TrackerWaistData.Rotation.Y = (float)(DataHMDQuat->tracker1_QuatY) / 32767.f;
				TrackerWaistData.Rotation.Z = (float)(DataHMDQuat->tracker1_QuatZ) / 32767.f;
				TrackerWaistData.vBat = (float)(DataHMDQuat->tracker1_vBat) / 255.f;

				TrackerLeftData.Rotation.W = (float)(DataHMDQuat->tracker2_QuatW) / 32767.f;
				TrackerLeftData.Rotation.X = (float)(DataHMDQuat->tracker2_QuatX) / 32767.f;
				TrackerLeftData.Rotation.Y = (float)(DataHMDQuat->tracker2_QuatY) / 32767.f;
				TrackerLeftData.Rotation.Z = (float)(DataHMDQuat->tracker2_QuatZ) / 32767.f;
				TrackerLeftData.vBat = (float)(DataHMDQuat->tracker2_vBat) / 255.f;

				TrackerRightData.Rotation.W = (float)(DataHMDQuat->tracker3_QuatW) / 32767.f;
				TrackerRightData.Rotation.X = (float)(DataHMDQuat->tracker3_QuatX) / 32767.f;
				TrackerRightData.Rotation.Y = (float)(DataHMDQuat->tracker3_QuatY) / 32767.f;
				TrackerRightData.Rotation.Z = (float)(DataHMDQuat->tracker3_QuatZ) / 32767.f;
				TrackerRightData.vBat = (float)(DataHMDQuat->tracker3_vBat) / 255.f;
				break;

			case 2:		//Controller quaternion packet

				RightCtrlData.Rotation.W = (float)(DataCtrl->Ctrl1_QuatW) / 32767.f;
				RightCtrlData.Rotation.X = (float)(DataCtrl->Ctrl1_QuatX) / 32767.f;
				RightCtrlData.Rotation.Y = (float)(DataCtrl->Ctrl1_QuatY) / 32767.f;
				RightCtrlData.Rotation.Z = (float)(DataCtrl->Ctrl1_QuatZ) / 32767.f;

				RightCtrlData.accelX = (float)(DataCtrl->Ctrl1_AccelX) / 2048.f;
				RightCtrlData.accelY = (float)(DataCtrl->Ctrl1_AccelY) / 2048.f;
				RightCtrlData.accelZ = (float)(DataCtrl->Ctrl1_AccelZ) / 2048.f;

				/*if (ctrlAccelEnable) {
					CalcAccelPosition(RightCtrlData.Rotation.W, RightCtrlData.Rotation.X, RightCtrlData.Rotation.Z, RightCtrlData.Rotation.Y, RightCtrlData.accelX, RightCtrlData.accelY, RightCtrlData.accelZ, ctrlRightPosData);
				}*/
				
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
				

				LeftCtrlData.Rotation.W = (float)(DataCtrl->Ctrl2_QuatW) / 32767.f;
				LeftCtrlData.Rotation.X = (float)(DataCtrl->Ctrl2_QuatX) / 32767.f;
				LeftCtrlData.Rotation.Y = (float)(DataCtrl->Ctrl2_QuatY) / 32767.f;
				LeftCtrlData.Rotation.Z = (float)(DataCtrl->Ctrl2_QuatZ) / 32767.f;

				
				LeftCtrlData.accelX = (float)(DataCtrl->Ctrl2_AccelX) / 2048.f;
				LeftCtrlData.accelY = (float)(DataCtrl->Ctrl2_AccelY) / 2048.f;
				LeftCtrlData.accelZ = (float)(DataCtrl->Ctrl2_AccelZ) / 2048.f;
				
				/*if (ctrlAccelEnable) {
					CalcAccelPosition(LeftCtrlData.Rotation.W, LeftCtrlData.Rotation.X, LeftCtrlData.Rotation.Y, LeftCtrlData.Rotation.Z, LeftCtrlData.accelX, LeftCtrlData.accelY, LeftCtrlData.accelZ, ctrlLeftPosData);
				}*/

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
				HMDData.Rotation = HMDfilter.getQuat();

				//CalcAccelPosition(HMDData.Rotation.W, HMDData.Rotation.X, HMDData.Rotation.Y, HMDData.Rotation.Z, accX, accY, accZ, hmdPosData);

				HMDData.Data = DataHMDRAW->HMDData;

				TrackerWaistData.Rotation.W = (float)(DataHMDRAW->tracker1_QuatW) / 32767;
				TrackerWaistData.Rotation.X = (float)(DataHMDRAW->tracker1_QuatX) / 32767;
				TrackerWaistData.Rotation.Y = (float)(DataHMDRAW->tracker1_QuatY) / 32767;
				TrackerWaistData.Rotation.Z = (float)(DataHMDRAW->tracker1_QuatZ) / 32767;
				TrackerWaistData.vBat = (float)(DataHMDRAW->tracker1_vBat) / 255;

				TrackerLeftData.Rotation.W = (float)(DataHMDRAW->tracker2_QuatW) / 32767;
				TrackerLeftData.Rotation.X = (float)(DataHMDRAW->tracker2_QuatX) / 32767;
				TrackerLeftData.Rotation.Y = (float)(DataHMDRAW->tracker2_QuatY) / 32767;
				TrackerLeftData.Rotation.Z = (float)(DataHMDRAW->tracker2_QuatZ) / 32767;
				TrackerLeftData.vBat = (float)(DataHMDRAW->tracker2_vBat) / 255;

				TrackerRightData.Rotation.W = (float)(DataHMDRAW->tracker3_QuatW) / 32767;
				TrackerRightData.Rotation.X = (float)(DataHMDRAW->tracker3_QuatX) / 32767;
				TrackerRightData.Rotation.Y = (float)(DataHMDRAW->tracker3_QuatY) / 32767;
				TrackerRightData.Rotation.Z = (float)(DataHMDRAW->tracker3_QuatZ) / 32767;
				TrackerRightData.vBat = (float)(DataHMDRAW->tracker3_vBat) / 255;
				break;
			}
		}
	}
}
/*
void CdataHandler::CalcAccelPosition(float quatW, float quatX, float quatY, float quatZ, float accelX, float accelY, float accelZ, PosData& pos) {

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

	//integrate to get velocity														TODO: REWORK ALL THIS CRAP
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
*/

/**
	 This basically blends between old pos data and new pos data using an exponential curve that is defined by the "smooth" component to avoid high frequency jitter.
*/
void CdataHandler::CalcTrackedPos(PosData& pos, Vector3 newPos, float smooth)
{
	Vector3 diff = pos.oldPosition - newPos;

	Vector3 absdiff = Vector3(fabsf(diff.X), fabsf(diff.Y), fabsf(diff.Z));

	//I have no idea, this is technically a high pass filter but think of it as a low pass one, plug "f\left(x\right)=\left(1-e^{-kx}\right)" into desmos to see how it works, let k be between 1 and 100.
	float outX = diff.X * (1 - pow(M_E, -smooth * absdiff.X));
	float outY = diff.Y * (1 - pow(M_E, -smooth * absdiff.Y));
	float outZ = diff.Z * (1 - pow(M_E, -smooth * absdiff.Z));

	//update position
	pos.position += Vector3(-outX, -outY, -outZ);

	pos.oldPosition = pos.position;
}

/**
	 Grabs Final HMD data...
*/
void CdataHandler::GetHMDData(THMD* HMD)
{
	if (HIDConnected) {

		Quaternion HMDQuat = SetOffsetQuat(HMDData.Rotation, HMDOffset, HMDConfigOffset);

		if (PSMConnected) {			//PSM POSITION

			HMD->Position = hmdPosData.position;
		}
		else {
			HMD->Position = Vector3(0, 0, 0);
		}

		HMD->Rotation = HMDQuat;
		
	}
	if ((GetAsyncKeyState(VK_F8) & 0x8000) && !once)
	{
		SetCentering();
		once = 1;
	}
	if (once && !(GetAsyncKeyState(VK_F8) & 0x8000))
	{
		once = 0;
	}
	if ((GetAsyncKeyState(VK_F9) & 0x8000) != 0) {
		ResetPos(false);
	}
}

/**
	 Grabs Final Controller data...
*/
void CdataHandler::GetControllersData(TController* RightController, TController* LeftController)
{
	if (HIDConnected) {

		Quaternion CtrlRightQuat = SetOffsetQuat(RightCtrlData.Rotation, RightCtrlOffset, CTRL1ConfigOffset);
		Quaternion CtrlLeftQuat = SetOffsetQuat(LeftCtrlData.Rotation, LeftCtrlOffset, CTRL2ConfigOffset);

		RightController->Rotation = CtrlRightQuat;
		RightController->Buttons = RightCtrlData.Buttons;
		RightController->Trigger = RightCtrlData.Trigger;
		RightController->JoyAxisX = RightCtrlData.JoyAxisX;
		RightController->JoyAxisY = RightCtrlData.JoyAxisY;
		RightController->TrackpY = RightCtrlData.TrackpY;
		RightController->vBat = RightCtrlData.vBat;

		RightController->FingThumb = RightCtrlData.FingThumb;
		RightController->FingIndex = RightCtrlData.FingIndex;
		RightController->FingMiddl = RightCtrlData.FingMiddl;
		RightController->FingRing = RightCtrlData.FingRing;
		RightController->FingPinky = RightCtrlData.FingPinky;



		LeftController->Rotation = CtrlLeftQuat;
		LeftController->Buttons = LeftCtrlData.Buttons;
		LeftController->Trigger = LeftCtrlData.Trigger;
		LeftController->JoyAxisX = LeftCtrlData.JoyAxisX;
		LeftController->JoyAxisY = LeftCtrlData.JoyAxisY;
		LeftController->TrackpY = LeftCtrlData.TrackpY;
		LeftController->vBat = LeftCtrlData.vBat;

		LeftController->FingThumb = LeftCtrlData.FingThumb;
		LeftController->FingIndex = LeftCtrlData.FingIndex;
		LeftController->FingMiddl = LeftCtrlData.FingMiddl;
		LeftController->FingRing = LeftCtrlData.FingRing;
		LeftController->FingPinky = LeftCtrlData.FingPinky;



		if (PSMConnected) {		//PSM POSITION

			RightController->Position = ctrlRightPosData.position;
			LeftController->Position = ctrlLeftPosData.position;
		}
		else {
			RightController->Position = Vector3(0.1, -0.3, 0.2);
			LeftController->Position = Vector3(-0.1, -0.3, 0.2);
		}
	}
}

/**
	 Grabs Final Tracker data...
*/
void CdataHandler::GetTrackersData(TTracker* waistTracker, TTracker* leftTracker, TTracker* rightTracker)
{/*
	if (HIDConnected) {
		Quaternion TRKWaistQuat = SetOffsetQuat(TrackerWaistData.Rotation, WaistTrackerOffset, Quaternion::Identity());
		Quaternion TRKLeftQuat = SetOffsetQuat(TrackerLeftData.Rotation, LeftTrackerOffset, Quaternion::Identity());
		Quaternion TRKRightQuat = SetOffsetQuat(TrackerRightData.Rotation, RightTrackerOffset, Quaternion::Identity());

		waistTracker->Rotation = TRKWaistQuat;
		waistTracker->vBat = TrackerWaistData.vBat;

		leftTracker->Rotation = TRKLeftQuat;
		leftTracker->vBat = TrackerLeftData.vBat;

		rightTracker->Rotation = TRKRightQuat;
		rightTracker->vBat = TrackerRightData.vBat;
	}

	if (PSMConnected) {
		//todo: psm stuff
		waistTracker->Position = Vector3(0, 0, 0);
		leftTracker->Position = Vector3(0, 0, 0);
		rightTracker->Position = Vector3(0, 0, 0);

	}
	else {
		waistTracker->Position = Vector3(0, 0, 0);
		leftTracker->Position = Vector3(0, 0, 0);
		rightTracker->Position = Vector3(0, 0, 0);
	}*/
}

/**
	 Attempts to connect to PSMoveService on the default address and port.
	 if it succeeds it allocates listeners for Controller and HMDs and starts up the PSMoveUpdate thread which is used for position tracking.
*/
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

/**
	 Updates PSMoveService position tracking data. 
*/
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

			Vector3 PSMSHMDPos = Vector3((float)psmHmdPos.x * k_fScalePSMoveAPIToMeters, (float)psmHmdPos.z * k_fScalePSMoveAPIToMeters, (float)psmHmdPos.y * k_fScalePSMoveAPIToMeters);

			CalcTrackedPos(hmdPosData, PSMSHMDPos, HMDSmoothK);

		}
		if (ctrl1Allocated) {
			PSM_GetControllerPosition(controllerList.controller_id[0], &psmCtrlRightPos);

			Vector3 PSMSCtrlRightPos = Vector3((float)psmCtrlRightPos.x * k_fScalePSMoveAPIToMeters, (float)psmCtrlRightPos.z * k_fScalePSMoveAPIToMeters, (float)psmCtrlRightPos.y * k_fScalePSMoveAPIToMeters);

			CalcTrackedPos(ctrlRightPosData, PSMSCtrlRightPos, ContSmoothK);

		}
		if (ctrl2Allocated) {
			PSM_GetControllerPosition(controllerList.controller_id[1], &psmCtrlLeftPos);

			Vector3 PSMSCtrlLeftPos = Vector3((float)psmCtrlLeftPos.x * k_fScalePSMoveAPIToMeters, (float)psmCtrlLeftPos.z * k_fScalePSMoveAPIToMeters, (float)psmCtrlLeftPos.y * k_fScalePSMoveAPIToMeters);

			CalcTrackedPos(ctrlLeftPosData, PSMSCtrlLeftPos, ContSmoothK);

		}
		//no need to update this faster than we can capture the images.
		std::this_thread::sleep_for(std::chrono::milliseconds(psmsMillisecondPeriod));
	}
}

/**
	 Checks for the headset to be connected and if it is it starts an HID connection with it.
	 Once it does it creates the ReadHID thread and grabs offset calibration data, filter beta, jitter rejection coefficient and PSMS update rate.
	 Once done it attempts to connect to PSMoveService by calling connectToPSMOVE().
*/
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

		//get config offsets
		HMDConfigOffset = Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_PitchOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_YawOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_RollOffset_Float) * M_PI / 180);
		filterBeta = vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_FilterBeta_Float);
		
		CTRL1ConfigOffset = Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller1_PitchOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_YawOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller1_RollOffset_Float) * M_PI / 180);
		CTRL2ConfigOffset = Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_PitchOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_YawOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_Controller2_RollOffset_Float) * M_PI / 180);
		

		//get calibration offsets
		HMDOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDW_Float);
		HMDOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_HMDY_Float);

		RightCtrlOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONTRightW_Float);
		RightCtrlOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONTRightY_Float);

		LeftCtrlOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONTLeftW_Float);
		LeftCtrlOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_CONTLeftY_Float);

		WaistTrackerOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKWaistW_Float);
		WaistTrackerOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKWaistY_Float);

		LeftTrackerOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKLeftW_Float);
		LeftTrackerOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKLeftY_Float);

		RightTrackerOffset.W = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKRightW_Float);
		RightTrackerOffset.Y = vr::VRSettings()->GetFloat(k_pch_Calibration_Section, k_pch_Calibration_TRKRightY_Float);

		DriverLog("[Settings] Loaded Calibration settings");

		//get psms update rate
		psmsUpdateRate = vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_PSMS_UPDATE_RATE_Int32);
		psmsMillisecondPeriod = (float)((1.f / psmsUpdateRate) * 1000.f);
		DriverLog("[Settings] PSMS update rate in hz: %i, with a period of %i milliseconds.", psmsUpdateRate, psmsMillisecondPeriod);
		
		//use ctrl accelerometers?
		//ctrlAccelEnable = vr::VRSettings()->GetBool(k_pch_Controllers_Section, k_pch_Controller_AccelEnable_Bool);
		//CURRENTLY DOES NOTHING I'LL REWORK THIS SOME DAY MAYBE OR SOMETHING

		//get tracker update rate and smoothness thing
		ContSmoothK = vr::VRSettings()->GetFloat(k_pch_Driver_Section, k_pch_TRACKER_SMOOTH_CTRL_Float);
		HMDSmoothK = vr::VRSettings()->GetFloat(k_pch_Driver_Section, k_pch_TRACKER_SMOOTH_HMD_Float);

		//set initial states for controllers and hmd.
		ResetPos(false);
		//Attempt to connect to psmoveservice
		connectToPSMOVE();
	}
}

void CdataHandler::stopData() 
{
	hid_close(hHID);
	hid_exit();
	HIDConnected = false;
	HIDInit = false;
}