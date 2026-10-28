/**
 * @file fall_detector.hpp
 * @brief Fall detector: free fall, then impact, then stillness.
 */
#pragma once

#include <cstdint>

namespace dsp {

/** @brief Fall detector settings. The defaults are starting values, not tuned
 * on real recordings. */
struct FallDetectorConfig {
  float sampleRateHz = 104.0F; ///< Rate at which update() is called.
  float freeFallG = 0.4F;      ///< Magnitude below this counts as free fall.
  float freeFallMinMs = 60.0F; ///< Free fall must last at least this long.
  float impactG = 2.5F;        ///< Magnitude above this counts as an impact.
  float impactWindowMs =
      500.0F; ///< Longest gap between the end of free fall and the impact.
  float settleMs =
      300.0F; ///< Ringing after the impact is ignored for this long.
  float stillBandG =
      0.25F; ///< "Still" means the magnitude stays within 1 g +/- this band.
  float stillMs = 1000.0F; ///< Required duration of stillness after the impact.
  float observeMs =
      3500.0F; ///< How long to wait for stillness before giving up.
};

/** @brief Detector state, mainly for tests and debugging. */
enum class FallStage : std::uint8_t {
  Idle,       ///< Watching for free fall.
  FreeFall,   ///< Free fall in progress.
  WaitImpact, ///< Free fall ended, waiting for the impact.
  Settling,   ///< Impact seen, skipping the ringing.
  WaitStill,  ///< Waiting for the person / device to stop moving.
};

/**
 * @brief Detects falls from the acceleration magnitude.
 *
 * A fall is reported when a free-fall phase (low magnitude) is followed by an
 * impact (high magnitude) and then by a period of stillness. An impact alone,
 * or movement after the impact, does not trigger. At 104 Hz very short impact
 * peaks can be missed.
 */
class FallDetector {
public:
  explicit FallDetector(const FallDetectorConfig &cfg = FallDetectorConfig{});

  /**
   * @brief Feed one sample.
   * @param accelMagnitudeG Length of the acceleration vector, in g.
   * @return true on the sample where a fall is confirmed (once per fall).
   */
  bool update(float accelMagnitudeG);

  /** @brief Number of confirmed falls. */
  std::uint32_t falls() const { return falls_; }

  /** @brief Current state. */
  FallStage stage() const { return stage_; }

  /** @brief Forget all state and the fall count. */
  void reset();

private:
  static std::uint32_t toSamples(float ms, float fs);

  FallDetectorConfig cfg_;
  std::uint32_t freeFallMinSamples_;
  std::uint32_t impactWindowSamples_;
  std::uint32_t settleSamples_;
  std::uint32_t stillSamples_;
  std::uint32_t observeSamples_;

  FallStage stage_ = FallStage::Idle;
  std::uint32_t counter_ = 0;      ///< Samples spent in the current stage.
  std::uint32_t stillCounter_ = 0; ///< Consecutive still samples.
  std::uint32_t falls_ = 0;
};

} // namespace dsp