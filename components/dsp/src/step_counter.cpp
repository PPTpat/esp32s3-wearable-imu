#include "dsp/step_counter.hpp"

#include <algorithm>
#include <cmath>

#include "dsp/vec3.hpp"

namespace dsp {

namespace {
constexpr float kMeanTimeConstantS = 1.0F; // gravity / slow drift removal
constexpr float kLowPassCutoffHz = 4.0F;   // jitter removal
constexpr float kEnvelopeDecayS = 2.0F;    // memory of recent peak heights
} // namespace

StepCounter::StepCounter(const StepCounterConfig &cfg) : cfg_(cfg) {
  const float fs = cfg_.sampleRateHz;
  hpAlpha_ = 1.0F - std::exp(-1.0F / (fs * kMeanTimeConstantS));
  lpAlpha_ = 1.0F - std::exp(-2.0F * kPi * kLowPassCutoffHz / fs);
  envDecay_ = std::exp(-1.0F / (fs * kEnvelopeDecayS));
  minIntervalSamples_ =
      static_cast<std::uint32_t>(cfg_.minIntervalMs * 0.001F * fs);
}

void StepCounter::reset() { *this = StepCounter(cfg_); }

bool StepCounter::update(float accelMagnitudeG) {
  if (!initialised_) {
    mean_ = accelMagnitudeG;
    initialised_ = true;
  }
  mean_ += hpAlpha_ * (accelMagnitudeG - mean_);
  const float dynamic = accelMagnitudeG - mean_;
  lp_ += lpAlpha_ * (dynamic - lp_);
  const float y0 = lp_;

  envelope_ = std::max(y0, envelope_ * envDecay_);
  const float threshold =
      std::max(cfg_.minThresholdG, cfg_.adaptiveRatio * envelope_);

  // y1_ is a peak if it is higher than its predecessor and not lower than its
  // successor.
  bool counted = false;
  if (y1_ > y2_ && y1_ >= y0 && y1_ > threshold) {
    const std::uint32_t peakIndex = sampleIndex_ - 1U;
    if (!haveStep_ || peakIndex - lastStepIndex_ >= minIntervalSamples_) {
      ++steps_;
      lastStepIndex_ = peakIndex;
      haveStep_ = true;
      counted = true;
    }
  }

  y2_ = y1_;
  y1_ = y0;
  ++sampleIndex_;
  return counted;
}

} // namespace dsp