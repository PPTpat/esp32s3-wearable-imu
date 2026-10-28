/**
 * @file step_counter.hpp
 * @brief Step counter based on peak detection of the acceleration magnitude.
 */
#pragma once

#include <cstdint>

namespace dsp {

/** @brief Step counter settings. The defaults are starting values, not tuned on
 * real recordings. */
struct StepCounterConfig {
  float sampleRateHz = 104.0F; ///< Rate at which update() is called.
  float minThresholdG =
      0.12F; ///< Smallest filtered peak height that can count as a step.
  float adaptiveRatio = 0.5F; ///< A peak must also exceed this fraction of the
                              ///< recent peak envelope.
  float minIntervalMs = 300.0F; ///< Minimum time between two steps (rejects the
                                ///< second bump of one step).
};

/**
 * @brief Counts steps from the acceleration magnitude.
 *
 * Pipeline: slow running mean removes gravity (high-pass, about 1 s), a
 * one-pole low-pass (about 4 Hz) removes jitter, and a local maximum above an
 * adaptive threshold, at least minIntervalMs after the previous one, counts as
 * a step. A step is reported one sample after the peak, because a peak needs a
 * lower sample after it to be recognised.
 */
class StepCounter {
public:
  explicit StepCounter(const StepCounterConfig &cfg = StepCounterConfig{});

  /**
   * @brief Feed one sample.
   * @param accelMagnitudeG Length of the acceleration vector, in g.
   * @return true if a step was counted because of this sample.
   */
  bool update(float accelMagnitudeG);

  /** @brief Steps counted so far. */
  std::uint32_t steps() const { return steps_; }

  /** @brief Forget all state and the step count. */
  void reset();

private:
  StepCounterConfig cfg_;
  float hpAlpha_;
  float lpAlpha_;
  float envDecay_;
  std::uint32_t minIntervalSamples_;

  bool initialised_ = false;
  float mean_ = 0.0F;
  float y1_ = 0.0F; ///< Previous filtered sample.
  float y2_ = 0.0F; ///< Filtered sample before that.
  float lp_ = 0.0F;
  float envelope_ = 0.0F;
  std::uint32_t sampleIndex_ = 0;
  std::uint32_t lastStepIndex_ = 0;
  bool haveStep_ = false;
  std::uint32_t steps_ = 0;
};

} // namespace dsp