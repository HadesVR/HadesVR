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

	if (Vector3::Magnitude(_data.Velocity) > 100.f && orientationFilterInit == true) {		//something is very wrong
		DriverLog("[Warning] Velocity reset on update because it's magnitude was over 100");
		ResetPos(true, false);									//reset
	}

	//update angular velocity
	_data.AngularVelocity += (_data.AngularAccel * deltatime);
	_data.AngularVelocity *= 0.75;

	//again to get position
	Vector3 instant_pos = (_data.Velocity * deltatime);

	//update filter
	_k.updateMeasIMU(instant_pos);

	//update temp position for drift correction stuff
	_data.TempIMUPos += instant_pos;

	//update old accel
	_data.oldAccel = _data.Accel;
}

void CdataHandler::UpdateVelocity(_TrackingData& _data, bool _wasTracked, Vector3 newCameraPos)
{
	//get deltatime
	auto now = std::chrono::high_resolution_clock::now();
	deltatime = std::chrono::duration_cast<std::chrono::microseconds>(now - _data.lastCamUpdate).count() / 1000000.0f;
	_data.lastCamUpdate = now;	
	//update velocity
	if (_wasTracked)  //don't update velocity on new camera tracking point.
	{
		_data.Velocity = (((_data.Position - _data.oldPosition) / deltatime));			 //update real velocity (IMU fused, filtered)
		_data.LastCameraVelocity = _data.CameraVelocity;																	 //update previous camera only velocity
		_data.CameraVelocity = (newCameraPos - _data.LastCameraPos) / deltatime;											 //update current camera only velocity
	}
	else {
		_data.Velocity = Vector3::Zero();											 //No velocity when not tracked
		_data.LastCameraVelocity = Vector3::Zero();							         //No velocity when not tracked
		_data.CameraVelocity = Vector3::Zero();								         //No velocity when not tracked
	}
	//update position
	_data.oldPosition = _data.Position;
}

void CdataHandler::UpdateDriftCorrection(_TrackingData& _data, Vector3 newCameraPos, float percent, float lowerTreshold, float upperTreshold, bool debug)
{
	float vm = Vector3::Magnitude(_data.Velocity);
	if (vm >= lowerTreshold && vm <= upperTreshold && Vector3::Magnitude(_data.CameraVelocity) != 0.f && Vector3::Magnitude(_data.TempIMUPos) >= vm / psmsUpdateRate)
	{
		Quaternion Offset;
		Quaternion Result;
		float Scale = percent;
		Vector3 instantCamVelocity = (_data.CameraVelocity - _data.LastCameraVelocity);				

		double  VectorsDot = Vector3::Dot(instantCamVelocity, _data.TempIMUPos);		
		Vector3 VectorsCross = Vector3::Cross(instantCamVelocity, _data.TempIMUPos);

		//////////////////////////////////////////////////////////////////////////////////
		Result.W = (VectorsDot + 1.f);													//
		Result.X = (VectorsCross.X);													//
		Result.Y = (VectorsCross.Y);													//
		Result.Z = (VectorsCross.Z);													//
		Result = Quaternion::Normalized(Result);										//
																						//
		if (fabsf(Result.Y) < 0.017f)	//don't update if if under 2 degrees of error	//
		{																				//
			_data.TempIMUPos = Vector3::Zero();											//
			return;																		//
		}																				//
		if (fabsf(Result.Y) > 0.131f)   //if error is > 15 degrees, update completely	//
		{																				//
			Scale = 1.f;																//
			if (debug) DriverLog("[Debug] Drift correction: Off by over 15 degrees!!!");//
		}																				//
		Offset.W = _data.RotationDriftOffset.W + (VectorsDot + 1.f * Scale);			//		This is disgusting		
		Offset.X = _data.RotationDriftOffset.X + (VectorsCross.X * Scale);				//
		Offset.Y = _data.RotationDriftOffset.Y + (VectorsCross.Y * Scale);				//		We don't belive in doing things properly over here.
		Offset.Z = _data.RotationDriftOffset.Z + (VectorsCross.Z * Scale);				//
		_data.RotationDriftOffset = Quaternion::Normalized(Offset);						//
		//////////////////////////////////////////////////////////////////////////////////		
		Vector3 vecRes = Quaternion::ToEuler(Result);									
		if (debug) DriverLog("[Debug] Drift correction: qW: %f,qX: %f,qY: %f,qZ: %f, vX: %f,vY: %f,vZ: %f", Result.W, Result.X, Result.Y, Result.Z, vecRes.X, vecRes.Y, vecRes.Z);
	}
	_data.TempIMUPos = Vector3::Zero();
	//if (debug) DriverLog("[Debug] Drift correction: Velocity: %f, imumag:%f, cammag:%f", vm / 60, Vector3::Magnitude(_data.TempIMUPos), Vector3::Magnitude(_data.CameraVelocity));
}

void CdataHandler::SaveUserOffset(float DataW, float DataY, Quaternion& Offset, const char* settingsKey)
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
		SaveUserOffset(1.f, 0.f, HMDData.TrackingData.RotationUserOffset, k_pch_Calibration_HMD);
		SaveUserOffset(1.f, 0.f, RightCtrlData.TrackingData.RotationUserOffset, k_pch_Calibration_CONTRight);
		SaveUserOffset(1.f, 0.f, LeftCtrlData.TrackingData.RotationUserOffset, k_pch_Calibration_CONTLeft);
		return;
	}
	/// <summary>
	/// HMD
	/// </summary>

	SaveUserOffset(HMDData.TrackingData.RawRotation.W, HMDData.TrackingData.RawRotation.Y, HMDData.TrackingData.RotationUserOffset, k_pch_Calibration_HMD);

	/// <summary>
	///				Controllers
	/// </summary>

	SaveUserOffset(RightCtrlData.TrackingData.RawRotation.W, RightCtrlData.TrackingData.RawRotation.Y, RightCtrlData.TrackingData.RotationUserOffset, k_pch_Calibration_CONTRight);
	SaveUserOffset(LeftCtrlData.TrackingData.RawRotation.W, LeftCtrlData.TrackingData.RawRotation.Y, LeftCtrlData.TrackingData.RotationUserOffset, k_pch_Calibration_CONTLeft);

	/// <summary>
	///					Trackers
	/// </summary>
	/* TODO: IMPLEMENT
	setCalibOffset(TrackerWaistData.Rotation.W, TrackerWaistData.Rotation.Y, WaistTrackerOffset, k_pch_Calibration_TRKWaist);
	setCalibOffset(TrackerLeftData.Rotation.W, TrackerLeftData.Rotation.Y, LeftTrackerOffset, k_pch_Calibration_TRKLeft);
	setCalibOffset(TrackerRightData.Rotation.W, TrackerRightData.Rotation.Y, RightTrackerOffset, k_pch_Calibration_TRKRight);
	*/
}

void CdataHandler::ResetPos(bool controllers, bool hmd) {
	if (controllers) {
		RightCtrlData.TrackingData.Position = Vector3::Zero();
		RightCtrlData.TrackingData.oldPosition = Vector3::Zero();
		RightCtrlData.TrackingData.Velocity = Vector3::Zero();

		LeftCtrlData.TrackingData.Position = Vector3::Zero();
		LeftCtrlData.TrackingData.oldPosition = Vector3::Zero();
		LeftCtrlData.TrackingData.Velocity = Vector3::Zero();
	}
	if (hmd) {
		HMDData.TrackingData.Position = Vector3::Zero();
		HMDData.TrackingData.oldPosition = Vector3::Zero();
		HMDData.TrackingData.Velocity = Vector3::Zero();
	}
}

void CdataHandler::SetOffsetQuat(_TrackingData& _data)
{
	if (
		_data.RotationUserOffset.W == 0.f && _data.RotationUserOffset.Y == 0.f) {  //Don't try to use an enpty offset quaternion 
		_data.RotationUserOffset.W = 1.f;
		_data.RotationUserOffset.Y = 0.f;
	}
	_data.VectorRotation = Quaternion::Normalized(_data.RawRotation * _data.RotationDriftOffset);			//calculate temp quaternion by offsetting raw data by the drift correction offset
	Quaternion temp = Quaternion::Normalized(_data.VectorRotation * _data.RotationConfigOffset);			//temp is the vector offset rotated by the config offset
	_data.OutputRotation = Quaternion::Normalized(Quaternion::Normalized(_data.RotationUserOffset) * temp);	//final output rotation is the one calculated before offse rotated by the user offset 
} 