#include "mpu.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define FSR 2000
//#define GYRO_SENS       ( 131.0f * 250.f / (float)FSR )
#define GYRO_SENS       16.375f
#define QUAT_SENS       1073741824.f //2^30

#define EPSILON         0.0001f
#define PI_2            1.57079632679489661923f

struct s_mympu mympu;

struct s_quat {
  float w, x, y, z;
};

union u_quat {
  struct s_quat _f;
  long _l[4];
} q;

static int ret;
static short gyro[3];
static short comp[3];
static short sensors;
static unsigned char fifoCount;

int mympu_open(unsigned int rate) {
  mpu_select_device(0);
  
  mpu_init_structures();

  ret = mpu_init(NULL);


  ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);


  ret = mpu_set_gyro_fsr(FSR);


  ret = mpu_set_accel_fsr(16);


  ret = mpu_set_compass_sample_rate(50);


  mpu_get_power_state((unsigned char *)&ret);


  ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);


  dmp_select_device(0);
  dmp_init_structures();

  ret = dmp_load_motion_driver_firmware();


  ret = dmp_set_fifo_rate(rate);


  ret = mpu_set_dmp_state(1);


  ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
  //	ret = dmp_enable_feature(DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL);


  return 0;
}

static inline float rad2deg( float rad )
{
  //return (180.f/PI) * rad;
  return 57.2957795131f * rad;
}

static float test, sqy, sqz, sqw;
static void quaternionToEuler( const struct s_quat *q, float* x, float* y, float* z )
{
  sqy = q->y * q->y;
  sqz = q->z * q->z;
  sqw = q->w * q->w;

  test = q->x * q->z - q->w * q->y;

  if ( test > 0.5f - EPSILON )
  {
    *x = 2.f * atan2( q->y, q->w );
    *y = PI_2;
    *z = 0;
  }
  else if ( test < -0.5f + EPSILON )
  {
    *x = -2.f * atan2( q->y, q->w );
    *y = -PI_2;
    *z = 0;
  }
  else
  {
    *x = atan2( 2.f * ( q->x * q->w + q->y * q->z ), 1.f - 2.f * ( sqz + sqw ) );
    *y = asin( 2.f * test );
    *z = atan2( 2.f * ( q->x * q->y - q->z * q->w ), 1.f - 2.f * ( sqy + sqz ) );
  }
}

static inline float wrap_180(float x) {
  return (x < -180.f ? x + 360.f : (x > 180.f ? x - 180.f : x));
}

int mympu_update_compass() {

  ret = mpu_get_compass_reg(comp,NULL);
  
  mympu.comp[0] = comp[0];
  mympu.comp[1] = comp[1];
  mympu.comp[2] = comp[2];

  return 0;
}

int mympu_update() {
  
  do {
    ret = dmp_read_fifo(gyro, NULL, q._l, NULL, &sensors, &fifoCount);
    /* will return:
    	0 - if ok
    	1 - no packet available
    	2 - if BIT_FIFO_OVERFLOWN is set
    	3 - if frame corrupted
           <0 - if error
    */

    if (ret != 0) return ret;
  } while (fifoCount > 1);

  q._f.w = (float)q._l[0] / (float)QUAT_SENS;
  q._f.x = (float)q._l[1] / (float)QUAT_SENS;
  q._f.y = (float)q._l[2] / (float)QUAT_SENS;
  q._f.z = (float)q._l[3] / (float)QUAT_SENS;

  // copy quaternions to mympu
  mympu.qW = q._f.w;
  mympu.qX = q._f.x;
  mympu.qY = q._f.y;
  mympu.qZ = q._f.z;

  return 0;
}
