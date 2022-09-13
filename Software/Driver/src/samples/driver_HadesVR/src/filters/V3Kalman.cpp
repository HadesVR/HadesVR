//=============================================================================================
// V3Kalman.c
//=============================================================================================
// * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
// * Created by Denys Sene, January, 1, 2017.
// * Released under MIT License-
// 
// * Updated by Liquid to support three single variable models at once for position filtering.
// * Updated: July, 2, 2022
//=============================================================================================


#include "filters/V3Kalman.h"

V3Kalman::V3Kalman() 
{

}

void V3Kalman::update() 
{
	gain.X = Camera_estm_err.X / (Camera_estm_err.X + Camera_meas_err);
	current_est.X = last_est.X + gain.X * (reading.X - last_est.X);
	Camera_estm_err.X = (1.0 - gain.X) * Camera_estm_err.X + (fabs(last_est.X - current_est.X) * Camera_process_n);
	last_est.X = current_est.X;

	gain.Y = Camera_estm_err.Y / (Camera_estm_err.Y + Camera_meas_err);
	current_est.Y = last_est.Y + gain.Y * (reading.Y - last_est.Y);
	Camera_estm_err.Y = (1.0 - gain.Y) * Camera_estm_err.Y + (fabs(last_est.Y - current_est.Y) * Camera_process_n);
	last_est.Y = current_est.Y;

	gain.Z = Camera_estm_err.Z / (Camera_estm_err.Z + Camera_meas_err);
	current_est.Z = last_est.Z + gain.Z * (reading.Z - last_est.Z);
	Camera_estm_err.Z = (1.0 - gain.Z) * Camera_estm_err.Z + (fabs(last_est.Z - current_est.Z) * Camera_process_n);
	last_est.Z = current_est.Z;
}

void V3Kalman::updateIMU()
{
	gain.X = IMU_estm_err.X / (IMU_estm_err.X + IMU_meas_err);
	current_est.X = last_est.X + gain.X * (reading.X - last_est.X);
	IMU_estm_err.X = (1.0 - gain.X) * IMU_estm_err.X + (fabs(last_est.X - current_est.X) * IMU_process_n);
	last_est.X = current_est.X;

	gain.Y = IMU_estm_err.Y / (IMU_estm_err.Y + IMU_meas_err);
	current_est.Y = last_est.Y + gain.Y * (reading.Y - last_est.Y);
	IMU_estm_err.Y = (1.0 - gain.Y) * IMU_estm_err.Y + (fabs(last_est.Y - current_est.Y) * IMU_process_n);
	last_est.Y = current_est.Y;

	gain.Z = IMU_estm_err.Z / (IMU_estm_err.Z + IMU_meas_err);
	current_est.Z = last_est.Z + gain.Z * (reading.Z - last_est.Z);
	IMU_estm_err.Z = (1.0 - gain.Z) * IMU_estm_err.Z + (fabs(last_est.Z - current_est.Z) * IMU_process_n);
	last_est.Z = current_est.Z;
}

void V3Kalman::updateMeasCam(Vector3 in) 
{
	reading = in;
	update();
}

void V3Kalman::updateMeasIMU(Vector3 in)
{
	reading += in;
	updateIMU();
}

void V3Kalman::setSettings(float cam_meas_e, float cam_estm_e, float cam_p_n, float imu_meas_e, float imu_estm_e, float imu_p_n)
{
	Camera_meas_err = cam_meas_e;
	Camera_process_n = cam_p_n;
	Camera_estm_err.X = cam_estm_e;
	Camera_estm_err.Y = cam_estm_e;
	Camera_estm_err.Z = cam_estm_e;
	IMU_meas_err = imu_meas_e;
	IMU_process_n = imu_p_n;
	IMU_estm_err.X = imu_estm_e;
	IMU_estm_err.Y = imu_estm_e;
	IMU_estm_err.Z = imu_estm_e;
}

Vector3 V3Kalman::getEstimation() 
{
	return current_est;
}