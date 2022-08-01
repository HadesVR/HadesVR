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
	meas_err = 1.f;
	process_n = 1.f;
	estm_err.X = 1.f;
	estm_err.Y = 1.f;
	estm_err.Z = 1.f;
}

void V3Kalman::update() 
{
	gain.X = estm_err.X / (estm_err.X + meas_err);
	current_est.X = last_est.X + gain.X * (reading.X - last_est.X);
	estm_err.X = (1.0 - gain.X) * estm_err.X + fabs(last_est.X - current_est.X) * process_n;
	last_est.X = current_est.X;

	gain.Y = estm_err.Y / (estm_err.Y + meas_err);
	current_est.Y = last_est.Y + gain.Y * (reading.Y - last_est.Y);
	estm_err.Y = (1.0 - gain.Y) * estm_err.Y + fabs(last_est.Y - current_est.Y) * process_n;
	last_est.Y = current_est.Y;

	gain.Z = estm_err.Z / (estm_err.Z + meas_err);
	current_est.Z = last_est.Z + gain.Z * (reading.Z - last_est.Z);
	estm_err.Z = (1.0 - gain.Z) * estm_err.Z + fabs(last_est.Z - current_est.Z) * process_n;
	last_est.Z = current_est.Z;
}

void V3Kalman::updateMeas(Vector3 in) 
{
	reading = in;
	update();
}

void V3Kalman::setSettings(float meas_e, float estm_e, float p_n)
{
	meas_err = meas_e;
	process_n = p_n;
	estm_err.X = estm_e;
	estm_err.Y = estm_e;
	estm_err.Z = estm_e;
}

Vector3 V3Kalman::getEstimation() 
{
	update();
	return current_est;
}