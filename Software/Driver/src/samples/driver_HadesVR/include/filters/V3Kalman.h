//=============================================================================================
// V3Kalman.h
//=============================================================================================
// * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
// * Created by Denys Sene, January, 1, 2017.
// * Released under MIT License-
// 
// * Updated by Liquid to support three single variable models at once for position filtering.
// * Updated: July, 2, 2022
//=============================================================================================

#pragma once
#ifndef __V3Kalman_h__
#define __V3Kalman_h__
#include <math.h>
#include <Vector3.hpp>

class V3Kalman {
  private:
    float meas_err;
    float process_n;
    Vector3 estm_err;
    Vector3 current_est = Vector3::Zero();
    Vector3 last_est = Vector3::Zero();
    Vector3 gain = Vector3::Zero();

    Vector3 reading = Vector3::Zero();

  public:
    V3Kalman(void);
    void setSettings(float meas_e, float estm_e, float p_n);
    void updateMeas(Vector3 in);
    void update();
    Vector3 getEstimation();
};
#endif
