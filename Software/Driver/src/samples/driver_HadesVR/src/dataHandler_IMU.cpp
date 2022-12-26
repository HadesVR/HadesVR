#include "dataHandler.h"

void CdataHandler::UpdateIMUPosition(_TrackingData& _data, V3Kalman& _k)
{
	//don't update on old data
	if (_data.Accel == _data.oldAccel) return;

	//get deltatime
	auto now = std::chrono::high_resolution_clock::now();
	deltatime = std::chrono::duration_cast<std::chrono::microseconds>(now - _data.lastIMUUpdate).count() / 1000000.0f;
	_data.lastIMUUpdate = now;

	//Rotate gravity vector https://web.archive.org/web/20121004000626/http://www.varesano.net/blog/fabio/simple-gravity-compensation-9-dom-imus
	Vector3 g = Vector3((2.0f * (_data.RawRotation.Y * _data.RawRotation.Z - _data.RawRotation.X * _data.RawRotation.W)),
						(2.0f * (_data.RawRotation.Y * _data.RawRotation.X + _data.RawRotation.Z * _data.RawRotation.W)),
						(_data.RawRotation.Y * _data.RawRotation.Y - _data.RawRotation.X * _data.RawRotation.X - _data.RawRotation.Z * _data.RawRotation.Z + _data.RawRotation.W * _data.RawRotation.W));

	//remove gravity vector (or at least attempt to)
	Vector3 lin_Acc = _data.Accel - g;

	//convert to m/s^2
	lin_Acc *= 9.80665f;

	//swap axis
	lin_Acc = Vector3(lin_Acc.Y, lin_Acc.X, lin_Acc.Z);

	//rotate vector to quaternion so it all matches up nicely
	Quaternion pq = Quaternion::Inverse(Quaternion(_data.VectorRotation.X, _data.VectorRotation.Z, _data.VectorRotation.Y, _data.VectorRotation.W));
	lin_Acc = (pq * lin_Acc);

	//integrate to get velocity											
	_data.Velocity += (lin_Acc * deltatime);
	_data.Velocity *= 0.9;
	//update angular velocity
	_data.AngularVelocity += (_data.AngularAccel * deltatime);
	_data.AngularVelocity *= 0.75;

	//again to get position
	Vector3 instant_pos = (_data.Velocity * deltatime);

	//update filter
	_k.updateMeasIMU(instant_pos);

	//update old accel
	_data.oldAccel = _data.Accel;
}

void CdataHandler::UpdateVelocity(_TrackingData& _data, bool _wasTracked)
{
	//get deltatime
	auto now = std::chrono::high_resolution_clock::now();
	deltatime = std::chrono::duration_cast<std::chrono::microseconds>(now - _data.lastCamUpdate).count() / 1000000.0f;
	_data.lastCamUpdate = now;	

	//update velocity
	if (_wasTracked) 
	{
	_data.Velocity = (_data.Position - _data.oldPosition) / deltatime; //don't update velocity on new camera tracking point.
	}
	//update position
	_data.oldPosition = _data.Position;
}

void setCalibOffset(float DataW, float DataY, Quaternion& Offset, const char* settingsKey)
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

void CdataHandler::SetCentering(bool reset)
{
	if (reset) {
		setCalibOffset(1.f, 0.f, HMDData.TrackingData.RotationUserOffset, k_pch_Calibration_HMD);
		setCalibOffset(1.f, 0.f, RightCtrlData.TrackingData.RotationUserOffset, k_pch_Calibration_CONTRight);
		setCalibOffset(1.f, 0.f, LeftCtrlData.TrackingData.RotationUserOffset, k_pch_Calibration_CONTLeft);
		return;
	}
	/// <summary>
	/// HMD
	/// </summary>

	setCalibOffset(HMDData.TrackingData.RawRotation.W, HMDData.TrackingData.RawRotation.Y, HMDData.TrackingData.RotationUserOffset, k_pch_Calibration_HMD);

	/// <summary>
	///				Controllers
	/// </summary>

	setCalibOffset(RightCtrlData.TrackingData.RawRotation.W, RightCtrlData.TrackingData.RawRotation.Y, RightCtrlData.TrackingData.RotationUserOffset, k_pch_Calibration_CONTRight);
	setCalibOffset(LeftCtrlData.TrackingData.RawRotation.W, LeftCtrlData.TrackingData.RawRotation.Y, LeftCtrlData.TrackingData.RotationUserOffset, k_pch_Calibration_CONTLeft);

	/// <summary>
	///					Trackers
	/// </summary>
	/* TODO: IMPLEMENT
	setCalibOffset(TrackerWaistData.Rotation.W, TrackerWaistData.Rotation.Y, WaistTrackerOffset, k_pch_Calibration_TRKWaist);
	setCalibOffset(TrackerLeftData.Rotation.W, TrackerLeftData.Rotation.Y, LeftTrackerOffset, k_pch_Calibration_TRKLeft);
	setCalibOffset(TrackerRightData.Rotation.W, TrackerRightData.Rotation.Y, RightTrackerOffset, k_pch_Calibration_TRKRight);
	*/
}

void CdataHandler::ResetPos(bool hmdOnly) {
	if (!hmdOnly) {
		RightCtrlData.TrackingData.Position = Vector3::Zero();
		RightCtrlData.TrackingData.oldPosition = Vector3::Zero();
		RightCtrlData.TrackingData.Velocity = Vector3::Zero();

		LeftCtrlData.TrackingData.Position = Vector3::Zero();
		LeftCtrlData.TrackingData.oldPosition = Vector3::Zero();
		LeftCtrlData.TrackingData.Velocity = Vector3::Zero();
	}
	HMDData.TrackingData.Position = Vector3::Zero();
	HMDData.TrackingData.oldPosition = Vector3::Zero();
	HMDData.TrackingData.Velocity = Vector3::Zero();
}

void CdataHandler::SetOffsetQuat(_TrackingData& _data)
{
	if (
		_data.RotationUserOffset.W == 0.f && _data.RotationUserOffset.Y == 0.f) {  //Don't try to use an enpty offset quaternion 
		_data.RotationUserOffset.W = 1.f;
		_data.RotationUserOffset.Y = 0.f;
	}
	
	Quaternion temp = Quaternion::Normalized(_data.RawRotation * _data.RotationDriftOffset);			//calculate temp quaternion by offsetting raw data by the drift correction offset

	_data.VectorRotation = Quaternion::Normalized(temp * _data.RotationUserOffset);						//vectorrotation is the corrected rotation offset by the user offset

	_data.OutputRotation = Quaternion::Normalized(_data.VectorRotation * _data.RotationConfigOffset);	//final output rotation is the one calculated before offset by the config offset 
}