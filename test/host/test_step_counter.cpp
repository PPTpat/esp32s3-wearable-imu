#include <cmath>
#include <cstdint>
#include <vector>

#include "dsp/step_counter.hpp"
#include "dsp/vec3.hpp"
#include "unity.h"

using dsp::StepCounter;

namespace {
constexpr float kFs = 104.0F;

std::uint32_t countSteps(const std::vector<float> &signal) {
  StepCounter c;
  for (const float s : signal) {
    c.update(s);
  }
  return c.steps();
}

void appendStill(std::vector<float> &s, float seconds) {
  const int n = static_cast<int>(seconds * kFs);
  for (int i = 0; i < n; ++i) {
    s.push_back(1.0F);
  }
}

/** N sinusoidal steps at @p stepHz with the given amplitude, followed by
 * stillness. */
std::vector<float> sineWalk(int steps, float stepHz, float amplitudeG) {
  std::vector<float> s;
  const int n = static_cast<int>(static_cast<float>(steps) / stepHz * kFs);
  for (int i = 0; i < n; ++i) {
    const float t = static_cast<float>(i) / kFs;
    s.push_back(1.0F + amplitudeG * std::sin(2.0F * dsp::kPi * stepHz * t));
  }
  appendStill(s, 1.5F);
  return s;
}

float gauss(float t, float mu, float sigma) {
  const float d = (t - mu) / sigma;
  return std::exp(-0.5F * d * d);
}
} // namespace

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

static void test_counts_sinusoidal_steps() {
  TEST_ASSERT_EQUAL_UINT32(20, countSteps(sineWalk(20, 1.8F, 0.35F)));
}

static void test_counts_gentle_but_clear_steps() {
  TEST_ASSERT_EQUAL_UINT32(20, countSteps(sineWalk(20, 1.8F, 0.25F)));
}

static void test_ignores_movement_below_threshold() {
  TEST_ASSERT_EQUAL_UINT32(0, countSteps(sineWalk(20, 1.8F, 0.08F)));
}

static void test_second_bump_of_one_step_is_not_counted() {
  // Each step has a main peak and a smaller bump 150 ms later (period 0.55 s).
  std::vector<float> s;
  const float period = 0.55F;
  const int steps = 20;
  const int n = static_cast<int>(static_cast<float>(steps) * period * kFs);
  for (int i = 0; i < n; ++i) {
    const float t = static_cast<float>(i) / kFs;
    const float phase = std::fmod(t, period);
    s.push_back(1.0F + 0.5F * gauss(phase, 0.15F, 0.04F) +
                0.3F * gauss(phase, 0.30F, 0.04F));
  }
  appendStill(s, 1.5F);
  TEST_ASSERT_EQUAL_UINT32(20, countSteps(s));
}

static void test_still_signal_gives_no_steps() {
  std::vector<float> s;
  appendStill(s, 30.0F);
  TEST_ASSERT_EQUAL_UINT32(0, countSteps(s));
}

static void test_small_noise_gives_no_steps() {
  std::vector<float> s;
  std::uint32_t seed = 777U;
  for (int i = 0; i < static_cast<int>(30.0F * kFs); ++i) {
    seed = seed * 1664525U + 1013904223U;
    const float noise = (static_cast<float>(seed >> 8) / 16777216.0F - 0.5F) *
                        0.06F; // +/- 0.03 g
    s.push_back(1.0F + noise);
  }
  TEST_ASSERT_EQUAL_UINT32(0, countSteps(s));
}

static void test_update_reports_the_counted_sample() {
  StepCounter c;
  std::uint32_t reported = 0;
  for (const float v : sineWalk(10, 1.8F, 0.35F)) {
    if (c.update(v)) {
      ++reported;
    }
  }
  TEST_ASSERT_EQUAL_UINT32(c.steps(), reported);
}

static void test_reset_clears_the_count() {
  StepCounter c;
  for (const float v : sineWalk(10, 1.8F, 0.35F)) {
    c.update(v);
  }
  TEST_ASSERT_TRUE(c.steps() > 0);
  c.reset();
  TEST_ASSERT_EQUAL_UINT32(0, c.steps());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_counts_sinusoidal_steps);
  RUN_TEST(test_counts_gentle_but_clear_steps);
  RUN_TEST(test_ignores_movement_below_threshold);
  RUN_TEST(test_second_bump_of_one_step_is_not_counted);
  RUN_TEST(test_still_signal_gives_no_steps);
  RUN_TEST(test_small_noise_gives_no_steps);
  RUN_TEST(test_update_reports_the_counted_sample);
  RUN_TEST(test_reset_clears_the_count);
  return UNITY_END();
}