#include "dataHandler.h"

void CdataHandler::SetCentering()
{
	/// <summary>
	/// HMD
	/// </summary>

	setCalibOffset(HMDData.qW, HMDData.qY, HMDOffset, k_pch_Calibration_HMD);

	/// <summary>
	///				Controllers
	/// </summary>

	setCalibOffset(RightCtrlData.qW, RightCtrlData.qY, RightCtrlOffset, k_pch_Calibration_CONTRight);
	setCalibOffset(LeftCtrlData.qW, LeftCtrlData.qY, LeftCtrlOffset, k_pch_Calibration_CONTLeft);

	/// <summary>
	///					Trackers
	/// </summary>

	setCalibOffset(TrackerWaistData.qW, TrackerWaistData.qY, WaistTrackerOffset, k_pch_Calibration_TRKWaist);
	setCalibOffset(TrackerLeftData.qW, TrackerLeftData.qY, LeftTrackerOffset, k_pch_Calibration_TRKLeft);
	setCalibOffset(TrackerRightData.qW, TrackerRightData.qY, RightTrackerOffset, k_pch_Calibration_TRKRight);
}

inline void setCalibOffset(float DataW, float DataY, Quaternion Offset, const char* settingsKey)
{
	Offset.W = DataW;
	Offset.Y = -DataY;

	if (Offset.W == 0.f && Offset.Y == 0.f) {
		Offset.W = 1.f;
		Offset.Y = 0.f;
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, settingsKey + char("W"), Offset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, settingsKey + char("Y"), Offset.Y);
	}
	else {
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, settingsKey + char("W"), Offset.W);
		vr::VRSettings()->SetFloat(k_pch_Calibration_Section, settingsKey + char("Y"), Offset.Y);
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

Quaternion CdataHandler::SetOffsetQuat(double qW, double qX, double qY, double qZ, Quaternion offsetQuat, Quaternion configOffset)
{
	Quaternion Input;
	Input.W = qW;
	Input.X = qX;
	Input.Y = qY;
	Input.Z = qZ;

	if (offsetQuat.W == 0.f && offsetQuat.Y == 0.f) {  //Don't try to use an enpty offset quaternion 
		offsetQuat.W = 1.f;
		offsetQuat.Y = 0.f;;
	}

	Quaternion inputCal = Quaternion::Normalized(offsetQuat * Input);

	Quaternion Output = Quaternion::Normalized(inputCal * configOffset);

	return Output;
}