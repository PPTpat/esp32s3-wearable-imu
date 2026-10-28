#include "dsp/madgwick.hpp"

#include <algorithm>

namespace dsp {

namespace {
constexpr float kMinAccelNorm = 1.0e-3F;
}

void Madgwick::update(const Vec3 &gyro, const Vec3 &accel, float dt) {
  if (!(dt > 0.0F)) {
    return;
  }

  const float q0 = q_.w;
  const float q1 = q_.x;
  const float q2 = q_.y;
  const float q3 = q_.z;

  // Rate of change of the quaternion from the gyroscope: 0.5 * q (x) (0, w).
  float d0 = 0.5F * (-q1 * gyro.x - q2 * gyro.y - q3 * gyro.z);
  float d1 = 0.5F * (q0 * gyro.x + q2 * gyro.z - q3 * gyro.y);
  float d2 = 0.5F * (q0 * gyro.y - q1 * gyro.z + q3 * gyro.x);
  float d3 = 0.5F * (q0 * gyro.z + q1 * gyro.y - q2 * gyro.x);

  const float an = norm(accel);
  if (an > kMinAccelNorm) {
    const float ax = accel.x / an;
    const float ay = accel.y / an;
    const float az = accel.z / an;

    // Objective function: estimated gravity direction minus measured direction.
    const float f1 = 2.0F * (q1 * q3 - q0 * q2) - ax;
    const float f2 = 2.0F * (q0 * q1 + q2 * q3) - ay;
    const float f3 = 1.0F - 2.0F * (q1 * q1 + q2 * q2) - az;

    // Gradient = J^T * f.
    float s0 = -2.0F * q2 * f1 + 2.0F * q1 * f2;
    float s1 = 2.0F * q3 * f1 + 2.0F * q0 * f2 - 4.0F * q1 * f3;
    float s2 = -2.0F * q0 * f1 + 2.0F * q3 * f2 - 4.0F * q2 * f3;
    float s3 = 2.0F * q1 * f1 + 2.0F * q2 * f2;

    const float sn = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (sn > 0.0F) {
      s0 /= sn;
      s1 /= sn;
      s2 /= sn;
      s3 /= sn;
      d0 -= beta_ * s0;
      d1 -= beta_ * s1;
      d2 -= beta_ * s2;
      d3 -= beta_ * s3;
    }
  }

  float n0 = q0 + d0 * dt;
  float n1 = q1 + d1 * dt;
  float n2 = q2 + d2 * dt;
  float n3 = q3 + d3 * dt;

  const float qn = std::sqrt(n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3);
  if (!(qn > 0.0F)) {
    reset(); // numerical failure: fall back to a valid state
    return;
  }
  q_ = Quaternion{n0 / qn, n1 / qn, n2 / qn, n3 / qn};
}

Euler Madgwick::euler() const {
  const float w = q_.w;
  const float x = q_.x;
  const float y = q_.y;
  const float z = q_.z;

  const float sinPitch = std::clamp(2.0F * (w * y - z * x), -1.0F, 1.0F);

  Euler e;
  e.rollDeg =
      std::atan2(2.0F * (w * x + y * z), 1.0F - 2.0F * (x * x + y * y)) *
      kRadToDeg;
  e.pitchDeg = std::asin(sinPitch) * kRadToDeg;
  e.yawDeg = std::atan2(2.0F * (w * z + x * y), 1.0F - 2.0F * (y * y + z * z)) *
             kRadToDeg;
  return e;
}

} // namespace dsp