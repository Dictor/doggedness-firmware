#ifndef DOGGEDNESS_POSTURE
#define DOGGEDNESS_POSTURES

#include <array>

//=====================================================================================================
// MahonyAHRS is from below with GPLv2
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

namespace Doggedness {
namespace posture {
class MahonyAHRS {
 private:
  float dt_;
  float twoKp_, twoKi_;  // 2 * P, I gain
  std::array<float, 4> q_;  // quaternion of sensor frame relative to auxiliary frame
  std::array<float, 3> integralFB_;  // integral error terms scaled by Ki
  static float invSqrt(float x);

 public:
  MahonyAHRS(float dt, float Kp = 0.5, float Ki = 0.0);
  void Update(float gx, float gy, float gz, float ax, float ay, float az);
  void Update(float gx, float gy, float gz, float ax, float ay, float az,
              float mx, float my, float mz);
  std::array<float, 4> GetQuaternion();
  std::array<double, 3> GetEuler();
};
};  // namespace posture
};  // namespace RbfpidBalbot

#endif