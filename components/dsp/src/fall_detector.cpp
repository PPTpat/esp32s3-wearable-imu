#include "dsp/fall_detector.hpp"

#include <cmath>

namespace dsp {

std::uint32_t FallDetector::toSamples(float ms, float fs) {
  return static_cast<std::uint32_t>(ms * 0.001F * fs + 0.5F);
}

FallDetector::FallDetector(const FallDetectorConfig &cfg)
    : cfg_(cfg),
      freeFallMinSamples_(toSamples(cfg.freeFallMinMs, cfg.sampleRateHz)),
      impactWindowSamples_(toSamples(cfg.impactWindowMs, cfg.sampleRateHz)),
      settleSamples_(toSamples(cfg.settleMs, cfg.sampleRateHz)),
      stillSamples_(toSamples(cfg.stillMs, cfg.sampleRateHz)),
      observeSamples_(toSamples(cfg.observeMs, cfg.sampleRateHz)) {}

void FallDetector::reset() { *this = FallDetector(cfg_); }

bool FallDetector::update(float a) {
  switch (stage_) {
  case FallStage::Idle:
    counter_ = (a < cfg_.freeFallG) ? counter_ + 1U : 0U;
    if (counter_ >= freeFallMinSamples_) {
      stage_ = FallStage::FreeFall;
    }
    break;

  case FallStage::FreeFall:
    if (a >= cfg_.freeFallG) {
      // Free fall is over; the impact may be this very sample.
      stage_ = FallStage::WaitImpact;
      counter_ = 0;
      if (a >= cfg_.impactG) {
        stage_ = FallStage::Settling;
      }
    }
    break;

  case FallStage::WaitImpact:
    ++counter_;
    if (a >= cfg_.impactG) {
      stage_ = FallStage::Settling;
      counter_ = 0;
    } else if (counter_ > impactWindowSamples_) {
      stage_ = FallStage::Idle;
      counter_ = 0;
    }
    break;

  case FallStage::Settling:
    ++counter_;
    if (counter_ >= settleSamples_) {
      stage_ = FallStage::WaitStill;
      counter_ = 0;
      stillCounter_ = 0;
    }
    break;

  case FallStage::WaitStill:
    ++counter_;
    stillCounter_ =
        (std::fabs(a - 1.0F) <= cfg_.stillBandG) ? stillCounter_ + 1U : 0U;
    if (stillCounter_ >= stillSamples_) {
      ++falls_;
      stage_ = FallStage::Idle;
      counter_ = 0;
      stillCounter_ = 0;
      return true;
    }
    if (counter_ > observeSamples_) {
      stage_ = FallStage::Idle;
      counter_ = 0;
      stillCounter_ = 0;
    }
    break;
  }
  return false;
}

} // namespace dsp