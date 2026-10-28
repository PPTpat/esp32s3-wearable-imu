#include <cmath>
#include <cstdint>

#include "dsp/madgwick.hpp"
#include "unity.h"

using dsp::Euler;
using dsp::Madgwick;
using dsp::Vec3;

namespace {
constexpr float kDt = 1.0F / 104.0F;

/** Accelerometer reading of a sensor rolled by @p deg about X (gravity seen in
 * the sensor frame). */
Vec3 accelForRoll(float deg) {
  const float r = deg * dsp::kDegToRad;
  return Vec3{0.0F, std::sin(r), std::cos(r)};
}

/** Accelerometer reading of a sensor pitched by @p deg about Y. */
Vec3 accelForPitch(float deg) {
  const float r = deg * dsp::kDegToRad;
  return Vec3{-std::sin(r), 0.0F, std::cos(r)};
}
} // namespace

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

static void test_flat_sensor_stays_at_identity() {
  Madgwick f;
  for (int i = 0; i < 500; ++i) {
    f.update(Vec3{}, Vec3{0.0F, 0.0F, 1.0F}, kDt);
  }
  const Euler e = f.euler();
  TEST_ASSERT_FLOAT_WITHIN(0.1F, 0.0F, e.rollDeg);
  TEST_ASSERT_FLOAT_WITHIN(0.1F, 0.0F, e.pitchDeg);
  TEST_ASSERT_FLOAT_WITHIN(0.1F, 0.0F, e.yawDeg);
}

static void test_converges_to_static_roll() {
  Madgwick f;
  for (int i = 0; i < 2000; ++i) { // about 19 s
    f.update(Vec3{}, accelForRoll(30.0F), kDt);
  }
  const Euler e = f.euler();
  TEST_ASSERT_FLOAT_WITHIN(0.5F, 30.0F, e.rollDeg);
  TEST_ASSERT_FLOAT_WITHIN(0.5F, 0.0F, e.pitchDeg);
}

static void test_converges_to_static_pitch() {
  Madgwick f;
  for (int i = 0; i < 2000; ++i) {
    f.update(Vec3{}, accelForPitch(20.0F), kDt);
  }
  const Euler e = f.euler();
  TEST_ASSERT_FLOAT_WITHIN(0.5F, 20.0F, e.pitchDeg);
  TEST_ASSERT_FLOAT_WITHIN(0.5F, 0.0F, e.rollDeg);
}

static void test_gyro_integrates_yaw() {
  Madgwick f;
  const Vec3 gyro{0.0F, 0.0F, 90.0F * dsp::kDegToRad};
  for (int i = 0; i < 104; ++i) { // exactly one second
    f.update(gyro, Vec3{0.0F, 0.0F, 1.0F}, kDt);
  }
  TEST_ASSERT_FLOAT_WITHIN(1.0F, 90.0F, f.euler().yawDeg);
}

static void test_tracks_rotation_when_gyro_and_accel_agree() {
  Madgwick f;
  const Vec3 gyro{45.0F * dsp::kDegToRad, 0.0F, 0.0F};
  float trueRoll = 0.0F;
  for (int i = 0; i < 208; ++i) { // two seconds at 45 deg/s = 90 deg
    trueRoll += 45.0F * kDt;
    f.update(gyro, accelForRoll(trueRoll), kDt);
  }
  TEST_ASSERT_FLOAT_WITHIN(2.0F, 90.0F, f.euler().rollDeg);
}

static void test_accelerometer_corrects_gyro_bias() {
  // A constant gyro bias would drift the roll forever without the
  // accelerometer.
  Madgwick f(0.2F);
  const Vec3 bias{2.0F * dsp::kDegToRad, 0.0F, 0.0F};
  for (int i = 0; i < 5000; ++i) { // about 48 s
    f.update(bias, Vec3{0.0F, 0.0F, 1.0F}, kDt);
  }
  TEST_ASSERT_FLOAT_WITHIN(3.0F, 0.0F, f.euler().rollDeg);
}

static void test_quaternion_stays_normalised() {
  Madgwick f;
  std::uint32_t seed = 12345U;
  auto rnd = [&seed]() {
    seed = seed * 1664525U + 1013904223U;
    return static_cast<float>(seed >> 8) / 16777216.0F - 0.5F; // -0.5 .. 0.5
  };
  for (int i = 0; i < 10000; ++i) {
    f.update(Vec3{rnd() * 4.0F, rnd() * 4.0F, rnd() * 4.0F},
             Vec3{rnd(), rnd(), 1.0F + rnd()}, kDt);
  }
  const auto &q = f.quaternion();
  const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  TEST_ASSERT_FLOAT_WITHIN(1.0e-4F, 1.0F, n);
}

static void test_zero_accel_falls_back_to_gyro_only() {
  Madgwick f;
  for (int i = 0; i < 104; ++i) {
    f.update(Vec3{0.0F, 0.0F, 90.0F * dsp::kDegToRad}, Vec3{}, kDt);
  }
  const Euler e = f.euler();
  TEST_ASSERT_TRUE(std::isfinite(e.rollDeg) && std::isfinite(e.pitchDeg) &&
                   std::isfinite(e.yawDeg));
  TEST_ASSERT_FLOAT_WITHIN(1.0F, 90.0F, e.yawDeg);
}

static void test_non_positive_dt_is_ignored() {
  Madgwick f;
  f.update(Vec3{1.0F, 1.0F, 1.0F}, Vec3{0.0F, 0.0F, 1.0F}, 0.0F);
  f.update(Vec3{1.0F, 1.0F, 1.0F}, Vec3{0.0F, 0.0F, 1.0F}, -0.01F);
  TEST_ASSERT_EQUAL_FLOAT(1.0F, f.quaternion().w);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_flat_sensor_stays_at_identity);
  RUN_TEST(test_converges_to_static_roll);
  RUN_TEST(test_converges_to_static_pitch);
  RUN_TEST(test_gyro_integrates_yaw);
  RUN_TEST(test_tracks_rotation_when_gyro_and_accel_agree);
  RUN_TEST(test_accelerometer_corrects_gyro_bias);
  RUN_TEST(test_quaternion_stays_normalised);
  RUN_TEST(test_zero_accel_falls_back_to_gyro_only);
  RUN_TEST(test_non_positive_dt_is_ignored);
  return UNITY_END();
}