#include "dataHandler.h"

// + (char)'W'
// 

void setCalibOffset(float DataW, float DataY, Quaternion &Offset, const char* settingsKey)
{
	std::string key_y = settingsKey;
	std::string key_w = settingsKey;

	Offset.W = DataW;
	Offset.Y = -DataY;
	
	key_w += "W";
	key_y += "Y";

	const char* keyW = key_w.c_str();
	const char* keyY = key_y.c_str();

	if (Offset.W == 0.f && Offset.Y == 0.f) {
		Offset.W = 1.f;
		Offset.Y = 0.f;
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, keyW, Offset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, keyY, Offset.Y);
	}
	else {
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, keyW, Offset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, keyY, Offset.Y);
	}
}

void CdataHandler::SetCentering()
{
	/// <summary>
	/// HMD
	/// </summary>

	setCalibOffset(HMDData.Rotation.W, HMDData.Rotation.Y, HMDOffset, k_pch_Calibration_HMD);

	/// <summary>
	///				Controllers
	/// </summary>

	setCalibOffset(RightCtrlData.Rotation.W, RightCtrlData.Rotation.Y, RightCtrlOffset, k_pch_Calibration_CONTRight);
	setCalibOffset(LeftCtrlData.Rotation.W, LeftCtrlData.Rotation.Y, LeftCtrlOffset, k_pch_Calibration_CONTLeft);

	/// <summary>
	///					Trackers
	/// </summary>

	setCalibOffset(TrackerWaistData.Rotation.W, TrackerWaistData.Rotation.Y, WaistTrackerOffset, k_pch_Calibration_TRKWaist);
	setCalibOffset(TrackerLeftData.Rotation.W, TrackerLeftData.Rotation.Y, LeftTrackerOffset, k_pch_Calibration_TRKLeft);
	setCalibOffset(TrackerRightData.Rotation.W, TrackerRightData.Rotation.Y, RightTrackerOffset, k_pch_Calibration_TRKRight);
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

Quaternion CdataHandler::SetOffsetQuat(Quaternion Input, Quaternion offsetQuat, Quaternion configOffset)
{
	if (offsetQuat.W == 0.f && offsetQuat.Y == 0.f) {  //Don't try to use an enpty offset quaternion 
		offsetQuat.W = 1.f;
		offsetQuat.Y = 0.f;
	}

	Quaternion inputCal = Quaternion::Normalized(offsetQuat * Input);

	Quaternion Output = Quaternion::Normalized(inputCal * configOffset);

	return Output;
}