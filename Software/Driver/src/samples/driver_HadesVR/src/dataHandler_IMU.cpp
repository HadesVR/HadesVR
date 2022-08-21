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
	Vector3 g = Vector3((2.0f * (_data.Rotation.Y * _data.Rotation.Z - _data.Rotation.X * _data.Rotation.W)),
						(2.0f * (_data.Rotation.Y * _data.Rotation.X + _data.Rotation.Z * _data.Rotation.W)),
						(_data.Rotation.Y * _data.Rotation.Y - _data.Rotation.X * _data.Rotation.X - _data.Rotation.Z * _data.Rotation.Z + _data.Rotation.W * _data.Rotation.W));

	//remove gravity vector (or at least attempt to)
	Vector3 lin_Acc = _data.Accel - g;

	//convert to m/s^2
	lin_Acc *= 9.80665f;

	//swap axis
	lin_Acc = Vector3(lin_Acc.Y, lin_Acc.X, lin_Acc.Z);

	//rotate vector to quaternion so it all matches up nicely
	Quaternion pq = Quaternion::Inverse(Quaternion(_data.CorrectedRotation.X, _data.CorrectedRotation.Z, _data.CorrectedRotation.Y, _data.CorrectedRotation.W));
	lin_Acc = (pq * lin_Acc);

	//integrate to get velocity											
	_data.Velocity += (lin_Acc * deltatime);

	//again to get position
	Vector3 instant_pos = (_data.Velocity * deltatime);

	//update filter
	_k.updateMeasIMU(instant_pos);

	//update old accel
	_data.oldAccel = _data.Accel;
}

void CdataHandler::UpdateVelocity(_TrackingData& _data) {

	//get deltatime
	auto now = std::chrono::high_resolution_clock::now();
	deltatime = std::chrono::duration_cast<std::chrono::microseconds>(now - _data.lastCamUpdate).count() / 1000000.0f;
	_data.lastCamUpdate = now;

	//update velocity
	_data.Velocity = (_data.Position - _data.oldPosition) / deltatime;

	//update position
	_data.oldPosition = _data.Position;
}