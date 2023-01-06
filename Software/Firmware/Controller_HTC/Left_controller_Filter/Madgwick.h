//=============================================================================================
// Madgwick.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef __Madgwick_h__
#define __Madgwick_h__
#include <math.h>
#include <stdint.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick {
  private:
    float delta_t = 0; // Used to control display output rate
    uint32_t now = 0;        // used to calculate integration interval
    uint32_t last_update = 0; // used to calculate integration interval
    static float invSqrt(float x);
    float beta;				// algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    

    //-------------------------------------------------------------------------------------------
    // Function declarations
  public:
    Madgwick(void);
    void changeBeta(float newBeta) { beta = newBeta; }
    void begin(float confBeta) { beta = confBeta; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    float getQuatW() {
      return q0;
    }

    float getQuatX() {
      return q1;
    }

    float getQuatY() {
      return q2;
    }

    float getQuatZ() {
      return q3;
    }

};
#endif
