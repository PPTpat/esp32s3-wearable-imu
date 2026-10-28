#include <cmath>
#include <cstdint>

#include "dsp/fall_detector.hpp"
#include "dsp/vec3.hpp"
#include "unity.h"

using dsp::FallDetector;
using dsp::FallStage;

namespace {
constexpr float kFs = 104.0F;

int samples(float ms) { return static_cast<int>(ms * 0.001F * kFs + 0.5F); }

/** Feed @p ms milliseconds of a constant magnitude; returns the number of fall
 * events. */
int feed(FallDetector &d, float value, float ms) {
  int events = 0;
  for (int i = 0; i < samples(ms); ++i) {
    if (d.update(value)) {
      ++events;
    }
  }
  return events;
}

/** Free fall, impact with a little ringing, then rest. Returns events. */
int fallSequence(FallDetector &d, float freeFallMs, float restMs) {
  int events = 0;
  events += feed(d, 1.0F, 1000.0F);
  events += feed(d, 0.1F, freeFallMs);
  events += feed(d, 4.0F, 30.0F);
  events += feed(d, 1.8F, 30.0F);
  events += feed(d, 0.7F, 30.0F);
  events += feed(d, 1.0F, restMs);
  return events;
}
} // namespace

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

static void test_rest_does_not_trigger() {
  FallDetector d;
  TEST_ASSERT_EQUAL_INT(0, feed(d, 1.0F, 10000.0F));
  TEST_ASSERT_TRUE(d.stage() == FallStage::Idle);
}

static void test_free_fall_impact_and_stillness_triggers_once() {
  FallDetector d;
  TEST_ASSERT_EQUAL_INT(1, fallSequence(d, 150.0F, 3000.0F));
  TEST_ASSERT_EQUAL_UINT32(1, d.falls());
  TEST_ASSERT_TRUE(d.stage() == FallStage::Idle);
}

static void test_event_comes_after_the_stillness_period() {
  FallDetector d;
  feed(d, 1.0F, 500.0F);
  feed(d, 0.1F, 150.0F);
  feed(d, 4.0F, 30.0F);
  // Settling (300 ms) and 1 s of stillness are required: no event after 1 s of
  // rest, one after 2 s.
  TEST_ASSERT_EQUAL_INT(0, feed(d, 1.0F, 1000.0F));
  TEST_ASSERT_EQUAL_INT(1, feed(d, 1.0F, 1000.0F));
}

static void test_impact_without_free_fall_does_not_trigger() {
  FallDetector d;
  feed(d, 1.0F, 1000.0F);
  feed(d, 4.0F, 30.0F);
  TEST_ASSERT_EQUAL_INT(0, feed(d, 1.0F, 4000.0F));
  TEST_ASSERT_EQUAL_UINT32(0, d.falls());
}

static void test_too_short_free_fall_does_not_trigger() {
  FallDetector d;
  TEST_ASSERT_EQUAL_INT(0, fallSequence(d, 30.0F, 4000.0F));
}

static void test_movement_after_impact_does_not_trigger() {
  FallDetector d;
  int events = 0;
  events += feed(d, 1.0F, 1000.0F);
  events += feed(d, 0.1F, 150.0F);
  events += feed(d, 4.0F, 30.0F);
  // The person keeps moving: magnitude swings between 0.5 g and 1.5 g for 4 s
  // (clearly not still, but never low enough to look like another free fall).
  for (int i = 0; i < samples(4000.0F); ++i) {
    const float t = static_cast<float>(i) / kFs;
    if (d.update(1.0F + 0.5F * std::sin(2.0F * dsp::kPi * 2.0F * t))) {
      ++events;
    }
  }
  TEST_ASSERT_EQUAL_INT(0, events);
  TEST_ASSERT_TRUE(d.stage() == FallStage::Idle);
}

static void test_late_impact_does_not_trigger() {
  FallDetector d;
  feed(d, 1.0F, 1000.0F);
  feed(d, 0.1F, 150.0F);
  feed(d, 1.0F, 800.0F); // impact window is 500 ms
  feed(d, 4.0F, 30.0F);
  TEST_ASSERT_EQUAL_INT(0, feed(d, 1.0F, 4000.0F));
}

static void test_two_separate_falls_give_two_events() {
  FallDetector d;
  int events = fallSequence(d, 150.0F, 3000.0F);
  events += fallSequence(d, 150.0F, 3000.0F);
  TEST_ASSERT_EQUAL_INT(2, events);
  TEST_ASSERT_EQUAL_UINT32(2, d.falls());
}

static void test_reset_clears_state() {
  FallDetector d;
  fallSequence(d, 150.0F, 3000.0F);
  d.reset();
  TEST_ASSERT_EQUAL_UINT32(0, d.falls());
  TEST_ASSERT_TRUE(d.stage() == FallStage::Idle);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_rest_does_not_trigger);
  RUN_TEST(test_free_fall_impact_and_stillness_triggers_once);
  RUN_TEST(test_event_comes_after_the_stillness_period);
  RUN_TEST(test_impact_without_free_fall_does_not_trigger);
  RUN_TEST(test_too_short_free_fall_does_not_trigger);
  RUN_TEST(test_movement_after_impact_does_not_trigger);
  RUN_TEST(test_late_impact_does_not_trigger);
  RUN_TEST(test_two_separate_falls_give_two_events);
  RUN_TEST(test_reset_clears_state);
  return UNITY_END();
}