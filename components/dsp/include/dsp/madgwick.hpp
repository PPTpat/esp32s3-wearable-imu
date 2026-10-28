/**
 * @file madgwick.hpp
 * @brief Madgwick orientation filter for a 6-axis IMU (gyroscope +
 * accelerometer).
 */
#pragma once

#include "dsp/vec3.hpp"

namespace dsp {

/** @brief Unit quaternion (w, x, y, z): rotation from the earth frame to the
 * sensor frame. */
struct Quaternion {
  float w = 1.0F;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

/** @brief Aerospace (Z-Y-X) Euler angles in degrees. */
struct Euler {
  float rollDeg = 0.0F;  ///< Rotation about X.
  float pitchDeg = 0.0F; ///< Rotation about Y.
  float yawDeg = 0.0F; ///< Rotation about Z (drifts: there is no magnetometer).
};

/**
 * @brief Gradient-descent orientation filter (S. Madgwick, 2010), IMU variant.
 *
 * Gyroscope integration is corrected by pulling the estimated gravity direction
 * towards the measured accelerometer vector. Roll and pitch are absolute; yaw
 * has no reference and drifts with the gyroscope bias.
 *
 * Frame convention: a sensor lying flat reads (0, 0, +1 g) and gives the
 * identity quaternion.
 */
class Madgwick {
public:
  /** @param beta Correction gain (about 0.03 .. 0.5); higher trusts the
   * accelerometer more. */
  explicit Madgwick(float beta = 0.1F) : beta_(beta) {}

  /**
   * @brief Advance the filter by one sample.
   * @param gyroRadPerSec Angular rate in rad/s.
   * @param accelG        Acceleration in any consistent unit (it is
   * normalised); a zero vector skips the correction and leaves pure gyroscope
   * integration.
   * @param dtSeconds     Time since the previous sample; non-positive values
   * are ignored.
   */
  void update(const Vec3 &gyroRadPerSec, const Vec3 &accelG, float dtSeconds);

  /** @brief Current orientation. */
  const Quaternion &quaternion() const { return q_; }

  /** @brief Current orientation as Euler angles. */
  Euler euler() const;

  /** @brief Return to the identity orientation. */
  void reset() { q_ = Quaternion{}; }

  /** @brief Change the correction gain. */
  void setBeta(float beta) { beta_ = beta; }

private:
  float beta_;
  Quaternion q_{};
};

} // namespace dsp