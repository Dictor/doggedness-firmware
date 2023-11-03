#include "../inc/posture.h"

#include <zephyr/logging/log.h>

#include <cmath>

LOG_MODULE_REGISTER(posture);

using namespace std;
using namespace Doggedness;
using namespace Doggedness::posture;

MahonyAHRS::MahonyAHRS(float dt, float Kp, float Ki)
    : dt_(dt),
      twoKp_(2 * Kp),
      twoKi_(2 * Ki),
      integralFB_({0.0, 0.0, 0.0}),
      q_({1.0, 0.0, 0.0, 0.0}) {}

void MahonyAHRS::Update(float gx, float gy, float gz, float ax, float ay,
                        float az) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q_[1] * q_[3] - q_[0] * q_[2];
    halfvy = q_[0] * q_[1] + q_[2] * q_[3];
    halfvz = q_[0] * q_[0] - 0.5f + q_[3] * q_[3];

    // Error is sum of cross product between estimated and measured direction of
    // gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi_ > 0.0f) {
      integralFB_[0] += twoKi_ * halfex * dt_;  // integral error scaled by Ki
      integralFB_[1] += twoKi_ * halfey * dt_;
      integralFB_[2] += twoKi_ * halfez * dt_;
      gx += integralFB_[0];  // apply integral feedback
      gy += integralFB_[1];
      gz += integralFB_[2];
    } else {
      integralFB_[0] = 0.0f;  // prevent integral windup
      integralFB_[1] = 0.0f;
      integralFB_[2] = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp_ * halfex;
    gy += twoKp_ * halfey;
    gz += twoKp_ * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt_);  // pre-multiply common factors
  gy *= (0.5f * dt_);
  gz *= (0.5f * dt_);
  qa = q_[0];
  qb = q_[1];
  qc = q_[2];
  q_[0] += (-qb * gx - qc * gy - q_[3] * gz);
  q_[1] += (qa * gx + qc * gz - q_[3] * gy);
  q_[2] += (qa * gy - qb * gz + q_[3] * gx);
  q_[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm =
      invSqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
  q_[0] *= recipNorm;
  q_[1] *= recipNorm;
  q_[2] *= recipNorm;
  q_[3] *= recipNorm;
}

void MahonyAHRS::Update(float gx, float gy, float gz, float ax, float ay,
                        float az, float mx, float my, float mz) {
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in
  // magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Update(gx, gy, gz, ax, ay, az);
    return;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q_[0] * q_[0];
    q0q1 = q_[0] * q_[1];
    q0q2 = q_[0] * q_[2];
    q0q3 = q_[0] * q_[3];
    q1q1 = q_[1] * q_[1];
    q1q2 = q_[1] * q_[2];
    q1q3 = q_[1] * q_[3];
    q2q2 = q_[2] * q_[2];
    q2q3 = q_[2] * q_[3];
    q3q3 = q_[3] * q_[3];

    // Reference direction of Earth's magnetic field
    hx = 2.0f *
         (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f *
         (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f *
         (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction and measured
    // direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if (twoKi_ > 0.0f) {
      integralFB_[0] += twoKi_ * halfex * dt_;  // integral error scaled by Ki
      integralFB_[1] += twoKi_ * halfey * dt_;
      integralFB_[2] += twoKi_ * halfez * dt_;
      gx += integralFB_[0];  // apply integral feedback
      gy += integralFB_[1];
      gz += integralFB_[2];
    } else {
      integralFB_[0] = 0.0f;  // prevent integral windup
      integralFB_[1] = 0.0f;
      integralFB_[2] = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp_ * halfex;
    gy += twoKp_ * halfey;
    gz += twoKp_ * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt_);  // pre-multiply common factors
  gy *= (0.5f * dt_);
  gz *= (0.5f * dt_);
  qa = q_[0];
  qb = q_[1];
  qc = q_[2];
  q_[0] += (-qb * gx - qc * gy - q_[3] * gz);
  q_[1] += (qa * gx + qc * gz - q_[3] * gy);
  q_[2] += (qa * gy - qb * gz + q_[3] * gx);
  q_[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm =
      invSqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
  q_[0] *= recipNorm;
  q_[1] *= recipNorm;
  q_[2] *= recipNorm;
  q_[3] *= recipNorm;
}

float MahonyAHRS::invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

array<float, 4> MahonyAHRS::GetQuaternion() { return q_; }

// from :
// https://github.com/brztitouan/euler-angles-quaternions-library-conversion/blob/fa075d741d6aa84c02a3052d61116e04e5ec7943/src/euler.cpp#L30
// TODO : Update LICENSE about it
array<double, 3> MahonyAHRS::GetEuler() {
  const double q00 = q_[0] * q_[0];
  const double q11 = q_[1] * q_[1];
  const double q22 = q_[2] * q_[2];
  const double q33 = q_[3] * q_[3];
  const double unitLength =
      q00 + q11 + q22 + q33;  // Normalised == 1, otherwise correction divisor.
  const double abcd = q_[0] * q_[1] + q_[2] * q_[3];
  const double eps =
      1e-7;  // TODO: pick from your math lib instead of hardcoding.
  const double pi = 3.14159265358979323846;  // TODO: pick from your math lib
                                             // instead of hardcoding.
  double yaw, pitch, roll;
  if (abcd > (0.5 - eps) * unitLength) {
    yaw = 2 * atan2(q_[2], q_[0]);
    pitch = pi;
    roll = 0;
  } else if (abcd < (-0.5 + eps) * unitLength) {
    yaw = -2 * ::atan2(q_[2], q_[0]);
    pitch = -pi;
    roll = 0;
  } else {
    const double adbc = q_[0] * q_[3] - q_[1] * q_[2];
    const double acbd = q_[0] * q_[2] - q_[1] * q_[3];
    yaw = ::atan2(2 * adbc, 1 - 2 * (q33 + q11));
    pitch = ::asin(2 * abcd / unitLength);
    roll = ::atan2(2 * acbd, 1 - 2 * (q22 + q11));
  }

  return std::array<double, 3> {pitch, roll, yaw};
}