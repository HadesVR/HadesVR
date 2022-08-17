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
    float Camera_meas_err;
    float Camera_process_n;
    Vector3 Camera_estm_err;
    float IMU_meas_err;
    float IMU_process_n;
    Vector3 IMU_estm_err;
    Vector3 current_est = Vector3::Zero();
    Vector3 last_est = Vector3::Zero();
    Vector3 gain = Vector3::Zero();

    Vector3 reading = Vector3::Zero();

  public:
    V3Kalman(void);
    void setSettings(float cam_meas_e, float cam_estm_e, float cam_p_n, float imu_meas_e, float imu_estm_e, float imu_p_n);
    void updateMeasCam(Vector3 in);
    void updateMeasIMU(Vector3 in);
    void update();
    void updateIMU();
    Vector3 getEstimation();
};
#endif
