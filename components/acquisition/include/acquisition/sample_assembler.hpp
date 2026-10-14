/**
 * @file sample_assembler.hpp
 * @brief Pairs gyroscope and accelerometer FIFO words into Samples.
 */
#pragma once

#include <cstdint>

#include "acquisition/sample.hpp"
#include "lsm6dsox/lsm6dsox.hpp"

namespace acquisition {

/**
 * @brief Stateful FIFO word -> Sample converter (pure logic, host-testable).
 *
 * The FIFO delivers gyro and accel words separately, in either order. A Sample
 * is emitted once both halves of a pair arrived. A second word of the same
 * kind before the pair completes replaces the first one and is counted in
 * unpaired(). Temperature, timestamp and unknown tags are ignored.
 */
class SampleAssembler {
  public:
    /**
     * @brief Feed one FIFO word.
     * @param word FIFO word read from the sensor.
     * @param[out] out Completed sample, valid only when true is returned.
     * @return true if @p out holds a new sample.
     */
    bool push(const lsm6dsox::FifoWord &word, Sample &out);

    /** @brief Number of words that were overwritten before being paired. */
    std::uint32_t unpaired() const { return unpaired_; }

    /** @brief Drop partial data (use after a FIFO overrun or reconfiguration). */
    void reset();

  private:
    Sample pending_{};
    bool hasAccel_ = false;
    bool hasGyro_ = false;
    std::uint32_t nextSeq_ = 0;
    std::uint32_t unpaired_ = 0;
};

} // namespace acquisition