/**
 * @file vec3.hpp
 * @brief Small math types and constants shared by the DSP algorithms.
 */
#pragma once

#include <cmath>

namespace dsp {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kDegToRad = kPi / 180.0F;
constexpr float kRadToDeg = 180.0F / kPi;

/** @brief Three-component vector. */
struct Vec3 {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

/** @brief Euclidean length of @p v. */
inline float norm(const Vec3 &v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

} // namespace dsp