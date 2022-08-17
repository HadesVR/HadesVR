#include "dataHandler.h"

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

void CdataHandler::ReloadCalibration() {
	//get config rotation offsets
	HMDConfigRotationOffset = Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_PitchOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_YawOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_RollOffset_Float) * M_PI / 180);

	CtrlRightConfigRotationOffset = Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerRight_PitchOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerRight_YawOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerRight_RollOffset_Float) * M_PI / 180);
	CtrlLeftConfigRotationOffset = Quaternion::FromEuler(vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerLeft_PitchOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerLeft_YawOffset_Float) * M_PI / 180, vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerLeft_RollOffset_Float) * M_PI / 180);

	//get config position offsets
	HMDConfigPositionOffset = Vector3(vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_XOffset_Float), vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_YOffset_Float), vr::VRSettings()->GetFloat(k_pch_HMD_Section, k_pch_HMD_ZOffset_Float));

	CtrlRightConfigPositionOffset = Vector3(vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerRight_XOffset_Float), vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerRight_YOffset_Float), vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerRight_ZOffset_Float));
	CtrlLeftConfigPositionOffset = Vector3(vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerLeft_XOffset_Float), vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerLeft_YOffset_Float), vr::VRSettings()->GetFloat(k_pch_Controllers_Section, k_pch_ControllerLeft_ZOffset_Float));

	DriverLog("[Settings] Reloaded Calibration settings");
}

void CdataHandler::ResetPos(bool hmdOnly) {
	if (!hmdOnly) {
		RightCtrlData.Position = Vector3::Zero();
		RightCtrlData.oldPosition = Vector3::Zero();
		RightCtrlData.Velocity = Vector3::Zero();
		RightCtrlData.oldVelocity = Vector3::Zero();

		LeftCtrlData.Position = Vector3::Zero();
		LeftCtrlData.oldPosition = Vector3::Zero();
		LeftCtrlData.Velocity = Vector3::Zero();
		LeftCtrlData.oldVelocity = Vector3::Zero();
	}
	HMDData.Position = Vector3::Zero();
	HMDData.oldPosition = Vector3::Zero();
	HMDData.Velocity = Vector3::Zero();
	HMDData.oldVelocity = Vector3::Zero();
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