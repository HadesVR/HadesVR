//=============================================================================================
// SimpleKalman.c
//=============================================================================================
// * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
// * Created by Denys Sene, January, 1, 2017.
// * Released under MIT License-
//=============================================================================================


#include "filters/SimpleKalman.h"

SimpleKalman::SimpleKalman()
{

}

float SimpleKalman::update(double meas)
{
	gain = estm_err / (estm_err + meas_err);
	current_est = last_est + gain * (meas - last_est);
	estm_err = (1.0 - gain) * estm_err + (fabs(last_est - current_est) * process_n);
	last_est = current_est;
	return current_est;
}

void SimpleKalman::setSettings(float meas_e, float estm_e, float p_n)
{
	meas_err = meas_e;
	estm_err = estm_e;
	process_n = p_n;
}