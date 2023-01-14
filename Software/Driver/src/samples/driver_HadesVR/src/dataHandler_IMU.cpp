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

	//update IMU velocity for drift correction stuff
	_data.IMUVelocity += (lin_Acc * deltatime);

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
		_data.LastCameraVelocity = _data.CameraVelocity;								 //update previous camera only velocity
		if (_data.LastCameraPos != Vector3::Zero()) {
			_data.CameraVelocity = (newCameraPos - _data.LastCameraPos) / deltatime;	 //update current camera only velocity
		}
		else {
			_data.CameraVelocity = Vector3::Zero();
		}
		_data.LastCameraPos = newCameraPos;
	}
	else {
		_data.Velocity = Vector3::Zero();											 //No velocity when not tracked
		_data.LastCameraVelocity = Vector3::Zero();							         //No velocity when not tracked
		_data.CameraVelocity = Vector3::Zero();								         //No velocity when not tracked
		_data.LastCameraPos = Vector3::Zero();
	}
	//update position
	_data.oldPosition = _data.Position;
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
	Quaternion HmdQuat = Quaternion::Normalized(Quaternion::FromEuler(0, HMDData.TrackingData.RotationYawDriftOffset, 0) * HMDData.TrackingData.RawRotation);
	Quaternion RightCont = Quaternion::Normalized(Quaternion::FromEuler(0, RightCtrlData.TrackingData.RotationYawDriftOffset, 0) * RightCtrlData.TrackingData.RawRotation);
	Quaternion LeftCont = Quaternion::Normalized(Quaternion::FromEuler(0, LeftCtrlData.TrackingData.RotationYawDriftOffset, 0) * LeftCtrlData.TrackingData.RawRotation);
	/// HMD
	SaveUserOffset(HmdQuat.W, HmdQuat.Y, HMDData.TrackingData.RotationUserOffset, k_pch_Calibration_HMD);
	///	Controllers
	SaveUserOffset(RightCont.W, RightCont.Y, RightCtrlData.TrackingData.RotationUserOffset, k_pch_Calibration_CONTRight);
	SaveUserOffset(LeftCont.W, LeftCont.Y, LeftCtrlData.TrackingData.RotationUserOffset, k_pch_Calibration_CONTLeft);
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
	if (_data.RotationUserOffset.W == 0.f && _data.RotationUserOffset.Y == 0.f) {  //Don't try to use an enpty offset quaternion 
		_data.RotationUserOffset = Quaternion::Identity();
	}
	Quaternion temp = Quaternion::Normalized(Quaternion::FromEuler(0, _data.RotationYawDriftOffset, 0) * _data.RawRotation);			//calculate temp quaternion by offsetting raw data by the drift correction offset
	_data.VectorRotation = Quaternion::Normalized(Quaternion::Normalized(_data.RotationUserOffset) * temp);
	_data.OutputRotation = Quaternion::Normalized(Quaternion::Normalized(_data.VectorRotation * _data.RotationConfigOffset));	//final output rotation is the one calculated before offse rotated by the config offset 
}

void CdataHandler::UpdateDriftCorrection(_TrackingData& _data, float lowerTreshold, float upperTreshold, bool debug)
{
	if (_data.DriftKalman.getPN() == 0.f || _data.Velocity == Vector3::Zero() || _data.IMUVelocity == Vector3::Zero() || _data.CameraVelocity == Vector3::Zero()) {
		_data.IMUVelocity = Vector3::Zero();
		return;
	}
	float vm = sqrtf(_data.Velocity.X * _data.Velocity.X + _data.Velocity.Y * _data.Velocity.Y);								//don't care about Z axis.
	float im = sqrtf(_data.IMUVelocity.X * _data.IMUVelocity.X + _data.IMUVelocity.Y * _data.IMUVelocity.Y);

	if (vm >= lowerTreshold && vm <= upperTreshold && im >= vm / psmsUpdateRate)
	{
		Quaternion localRotation = Quaternion::Inverse(Quaternion::Normalized(Quaternion::Normalized(_data.RotationUserOffset) * _data.RawRotation));
		Quaternion offset = shortestRotation(Vector3::Normalized(localRotation * Vector3(_data.IMUVelocity.Y, _data.IMUVelocity.X, _data.IMUVelocity.Z)), Vector3::Normalized(_data.CameraVelocity / psmsUpdateRate));		//this implementation is a bit yucky
		Vector3 rot = Quaternion::ToEuler(offset);																																//however, it works.
		if (fabsf(_data.RotationYawDriftOffset - rot.Y) > 2.5f)
		{
			rot.Y += M_PI;
		}
		_data.RotationYawDriftOffset = _data.DriftKalman.update(rot.Y);
		//_data.RotationYawDriftOffset = rot.Y;
		if (debug) DriverLog("[Debug] Drift correction: Final correction: %f, Noisy boi correction: %f", _data.RotationYawDriftOffset * 180.0 / M_PI, rot.Y * 180.0 / M_PI);
	}
	_data.IMUVelocity = Vector3::Zero();
}
// Returns the quaternion rotation that rotates vector `a` to vector `b`, Thanks chatGPT
Quaternion CdataHandler::shortestRotation(const Vector3& a, const Vector3& b) {
	// Compute the dot product between `a` and `b`
	double dotProduct = Vector3::Dot(a,b);
	// If the dot product is <0, invert one of the components.
	if (dotProduct < 0) {
		return shortestRotation(-a, b);
	}
	// Compute the cross product between `a` and `b`
	Vector3 crossProduct = Vector3::Cross(a, b);
	// Compute the w component of the quaternion
	double w = sqrt(Vector3::Magnitude(a) * Vector3::Magnitude(b)) + dotProduct;
	// Construct Quaternion
	Quaternion o = Quaternion(crossProduct.X, crossProduct.Y, crossProduct.Z, w);
	// Normalize the quaternion
	o = Quaternion::Normalized(o);
	// Return the normalized quaternion
	return o;
}