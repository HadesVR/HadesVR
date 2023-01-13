//=============================================================================================
// SimpleKalman.h
//=============================================================================================
// * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
// * Created by Denys Sene, January, 1, 2017.
// * Released under MIT License-
//=============================================================================================

#pragma once
#ifndef __SimpleKalman_h__
#define __SimpleKalman_h__
#include <math.h>

class SimpleKalman {
  private:
    float meas_err = 0;
    float process_n = 0;
    float estm_err = 0;

    float current_est = 0;
    float last_est = 0;

    float gain = 0;

  public:
    SimpleKalman(void);
    void setSettings(float meas_e, float estm_e, float p_n);
    float getPN() { return process_n; }
    float update(double meas);
};
#endif
